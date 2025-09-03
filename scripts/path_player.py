#!/usr/bin/env python3
"""
Reproduce un recorrido YAML de forma continua (NavigateThroughPoses) o discreta (FollowWaypoints).

Uso típico (continuo, sin paradas, mirando delante):
  ros2 run path_tools path_player.py --file /routes/nombre.yaml \
      --mode continuous --forward-only --resample 0.05
"""
import os, sys, math, yaml, rclpy, time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, FollowWaypoints

def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

def unwrap(prev, curr):
    while curr - prev > math.pi:  curr -= 2*math.pi
    while curr - prev < -math.pi: curr += 2*math.pi
    return curr

def forward_headings(xy):
    # Tangente del tramo para cada punto, con desenrollado de ángulos
    n = len(xy)
    yaws = []
    for i in range(n):
        if i < n-1:
            dx = xy[i+1][0] - xy[i][0]
            dy = xy[i+1][1] - xy[i][1]
        else:
            dx = xy[i][0] - xy[i-1][0]
            dy = xy[i][1] - xy[i-1][1]
        yaws.append(math.atan2(dy, dx))
    for i in range(1, n):
        yaws[i] = unwrap(yaws[i-1], yaws[i])
    return yaws

def resample_polyline(xy, step):
    if not step or step <= 0.0 or len(xy) < 2:
        return xy
    out = [xy[0]]
    remain = 0.0
    for i in range(1, len(xy)):
        x0, y0 = out[-1]
        x1, y1 = xy[i]
        seg = math.hypot(x1-x0, y1-y0)
        if seg < 1e-6:
            continue
        ux, uy = (x1-x0)/seg, (y1-y0)/seg
        dist = seg
        while dist + remain >= step - 1e-9:
            take = step - remain
            x0 += ux * take
            y0 += uy * take
            out.append((x0, y0))
            dist -= take
            remain = 0.0
        remain += dist
    if out[-1] != xy[-1]:
        out.append(xy[-1])
    return out

class Player(Node):
    def __init__(self, yaml_file: str, mode='continuous', forward_only=True, resample=0.05):
        super().__init__("path_player")
        self.yaml_file = yaml_file
        self.mode = mode
        self.forward_only = forward_only
        self.resample = resample

        # Clientes de acción
        self.ntp_client = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self.fw_client  = ActionClient(self, FollowWaypoints,       "/follow_waypoints")

        # Cargar YAML → lista de PoseStamped
        self.poses = self._load_poses(yaml_file)

        # Publicar /initialpose con la orientación del primer tramo (forward)
        self._publish_initialpose(self.poses[0])

        # Enviar acción
        if self.mode == 'continuous':
            self._send_nav_through_poses()
        else:
            self._send_follow_waypoints()

    # --- YAML → Poses --------------------------------------------
    def _load_poses(self, path):
        data = yaml.safe_load(open(path))
        wps = data["waypoints"]
        xy = [(float(w["x"]), float(w["y"])) for w in wps]

        # Re-muestreo para densidad uniforme (evita “huecos”)
        xy = resample_polyline(xy, self.resample) if self.resample else xy

        # Yaw: por defecto “mirando delante” (tangente)
        if self.forward_only:
            yaws = forward_headings(xy)
        else:
            # Usa yaw del YAML si existe; si falta, deriva de segmento
            yaws = []
            for i, w in enumerate(wps):
                if "yaw" in w:
                    yaws.append(float(w["yaw"]))
                else:
                    if i < len(wps)-1:
                        dx = wps[i+1]["x"] - w["x"]; dy = wps[i+1]["y"] - w["y"]
                    else:
                        dx = w["x"] - wps[i-1]["x"]; dy = w["y"] - wps[i-1]["y"]
                    yaws.append(math.atan2(dy, dx))

        poses = []
        now = self.get_clock().now().to_msg()
        for (x, y), yaw in zip(xy, yaws):
            q = quat_from_yaw(yaw)
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = now
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            poses.append(ps)
        self.get_logger().info(f"Leyendo {path} → {len(poses)} poses (mode={self.mode}, forward_only={self.forward_only}, resample={self.resample} m)")
        return poses

    # --- Pose inicial --------------------------------------------
    def _publish_initialpose(self, ps: PoseStamped):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose = ps.pose
        # Covarianzas: 5 cm y ~5°
        msg.pose.covariance[0]  = 0.05**2
        msg.pose.covariance[7]  = 0.05**2
        msg.pose.covariance[35] = (math.radians(5))**2
        pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        # Publica 3 veces por si algún nodo arranca más tarde
        for _ in range(3):
            pub.publish(msg)
            time.sleep(0.1)
        self.get_logger().info("📍 Pose inicial publicada")

    # --- Acciones -------------------------------------------------
    def _send_nav_through_poses(self):
        self.get_logger().info("🚀 /navigate_through_poses (modo continuo)")
        if not self.ntp_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("⛔ No está disponible /navigate_through_poses")
            rclpy.shutdown(); return
        goal = NavigateThroughPoses.Goal()
        goal.poses = self.poses
        future = self.ntp_client.send_goal_async(goal, feedback_callback=self._feedback_ntp)
        future.add_done_callback(self._result_common)

    def _send_follow_waypoints(self):
        self.get_logger().info("🚀 /follow_waypoints (modo discreto)")
        if not self.fw_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("⛔ No está disponible /follow_waypoints")
            rclpy.shutdown(); return
        goal = FollowWaypoints.Goal()
        goal.poses = self.poses
        future = self.fw_client.send_goal_async(goal, feedback_callback=self._feedback_fw)
        future.add_done_callback(self._result_common)

    # --- Feedback / Result ---------------------------------------
    def _feedback_ntp(self, fb):
        # fb.feedback.current_pose, distance_remaining, etc. (según versión)
        pass

    def _feedback_fw(self, fb):
        # fb.feedback.current_waypoint
        pass

    def _result_common(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("⛔ Acción rechazada")
            rclpy.shutdown(); return
        self.get_logger().info("✅ Acción aceptada, navegando…")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(lambda *_: (self.get_logger().info("🏁 Recorrido terminado"), rclpy.shutdown()))

def main():
    # CLI simple compatible con tu --file
    if "--file" not in sys.argv:
        print("Requiere --file /ruta/archivo.yaml [--mode continuous|discrete] [--forward-only] [--resample 0.05]", file=sys.stderr)
        sys.exit(1)
    yaml_file = sys.argv[sys.argv.index("--file") + 1]
    # Flags (defaults)
    mode = "continuous" if "--mode" not in sys.argv else sys.argv[sys.argv.index("--mode")+1]
    forward_only = ("--forward-only" in sys.argv) or ("--no-forward" not in sys.argv)
    resample = 0.05
    if "--resample" in sys.argv:
        try:
            resample = float(sys.argv[sys.argv.index("--resample")+1])
        except Exception:
            pass

    rclpy.init()
    node = Player(yaml_file, mode=mode, forward_only=forward_only, resample=resample)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
