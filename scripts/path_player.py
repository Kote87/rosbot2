#!/usr/bin/env python3
"""
Reproduce un recorrido YAML en modo fluido con Nav2.
Uso:
  ros2 run path_tools path_player.py --file /routes/nombre.yaml
"""
import sys, time, yaml, math, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, FollowWaypoints
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid


def quaternion_from_yaw(yaw):
    """Devuelve (x,y,z,w) para yaw dado (roll=pitch=0)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_qzw(qz, qw):
    return 2.0 * math.atan2(qz, qw)


def unwrap(prev, curr):
    # Suavizado de saltos ±pi para continuidad
    while curr - prev > math.pi:
        curr -= 2 * math.pi
    while curr - prev < -math.pi:
        curr += 2 * math.pi
    return curr


def resample_polyline(xy, step=0.05):
    # Re-muestreo simple: puntos cada 'step' metros (para “entrelazar”)
    if not xy or len(xy) < 2 or step <= 0.0:
        return xy
    out = [xy[0]]
    acc = 0.0
    for i in range(1, len(xy)):
        x0, y0 = out[-1]
        x1, y1 = xy[i]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-6:
            continue
        ux, uy = (x1 - x0) / seg, (y1 - y0) / seg
        d = seg
        while d - acc >= step - 1e-9:
            x0 += ux * (step - acc)
            y0 += uy * (step - acc)
            out.append((x0, y0))
            d -= step - acc
            acc = 0.0
        acc += d
    if out[-1] != xy[-1]:
        out.append(xy[-1])
    return out


def forward_headings(xy):
    # Orienta cada pose hacia la tangente del recorrido (mirar adelante)
    n = len(xy)
    yaws = []
    for i in range(n):
        if i < n - 1:
            dx = xy[i + 1][0] - xy[i][0]
            dy = xy[i + 1][1] - xy[i][1]
        else:
            dx = xy[i][0] - xy[i - 1][0]
            dy = xy[i][1] - xy[i - 1][1]
        yaws.append(math.atan2(dy, dx))
    for i in range(1, n):
        yaws[i] = unwrap(yaws[i - 1], yaws[i])
    return yaws


class Player(Node):
    def __init__(self, yaml_file: str):
        super().__init__("path_player")
        self.declare_parameter("global_frame", "map")
        self.global_frame = self.get_parameter("global_frame").value
        # --- Cargar y preparar poses (fluido) ---
        self.poses = self._load_as_poses(yaml_file)
        # Acciones / servicios Nav2
        self.ntp_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses"
        )
        self.fw_client = ActionClient(
            self, FollowWaypoints, "/follow_waypoints"
        )  # fallback
        self.clear_local = self.create_client(
            Empty, "/local_costmap/clear_entirely_local_costmap"
        )
        self.clear_global = self.create_client(
            Empty, "/global_costmap/clear_entirely_global_costmap"
        )
        self.map_msg = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos_profile=1)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )
        # Reintentos en arranque
        self.max_retries = 3
        self.retry = 0

        # Publica pose inicial (con avance si hace falta), limpia y envía goal continuo
        self._publish_initial_pose_forward()
        self._clear_costmaps()
        self._send_ntp_goal()

    # --- YAML → lista PoseStamped (fluida) -------------------------
    def _load_as_poses(self, f):
        data = yaml.safe_load(open(f))
        wps = data["waypoints"]
        xy = [(float(w["x"]), float(w["y"])) for w in wps]
        # Re‑muestreo (entrelazar) a 5 cm
        xy = resample_polyline(xy, step=0.05)
        yaws = forward_headings(xy)
        poses = []
        now = self.get_clock().now().to_msg()
        for (x, y), yaw in zip(xy, yaws):
            q = quaternion_from_yaw(yaw)
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame
            ps.header.stamp = now
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            poses.append(ps)
        self.get_logger().info(f"Leyendo {f} – {len(poses)} poses (fluido)")
        return poses

    # --- Mapa / celdas libres -------------------------------------
    def _on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _is_free_on_static(self, x, y):
        m = self.map_msg
        if m is None:
            return True
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or my < 0 or mx >= m.info.width or my >= m.info.height:
            return False
        val = m.data[my * m.info.width + mx]
        return val == 0  # 0: libre; 100: ocupada; -1: desconocido

    # --- Pose inicial con avance “hacia delante” -------------------
    def _publish_initial_pose_forward(self):
        ps0 = self.poses[0]
        # vector hacia delante
        if len(self.poses) >= 2:
            dx = self.poses[1].pose.position.x - ps0.pose.position.x
            dy = self.poses[1].pose.position.y - ps0.pose.position.y
            n = math.hypot(dx, dy) or 1.0
            fx, fy = dx / n, dy / n
        else:
            yaw = yaw_from_qzw(ps0.pose.orientation.z, ps0.pose.orientation.w)
            fx, fy = math.cos(yaw), math.sin(yaw)
        # probamos offsets: 0.00→0.60 m
        x0, y0 = ps0.pose.position.x, ps0.pose.position.y
        chosen = (x0, y0)
        for off in [0.00, 0.15, 0.30, 0.45, 0.60]:
            cx, cy = x0 + fx * off, y0 + fy * off
            if self._is_free_on_static(cx, cy):
                chosen = (cx, cy)
                break
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = self.global_frame
        ip.pose.pose = ps0.pose
        ip.pose.pose.position.x, ip.pose.pose.position.y = chosen
        ip.pose.covariance[0] = 0.05**2
        ip.pose.covariance[7] = 0.05**2
        ip.pose.covariance[35] = (math.radians(5)) ** 2
        for _ in range(3):
            self.init_pub.publish(ip)
            time.sleep(0.1)
        if (chosen[0], chosen[1]) != (x0, y0):
            self.get_logger().info(
                f"📍 Pose inicial adelantada {math.hypot(chosen[0]-x0, chosen[1]-y0):.2f} m"
            )
        else:
            self.get_logger().info("📍 Pose inicial publicada")

    def _republish_initial_pose_bump(self):
        # Para reintentos: empuja +0.25 m por intento
        ps0 = self.poses[0]
        if len(self.poses) >= 2:
            dx = self.poses[1].pose.position.x - ps0.pose.position.x
            dy = self.poses[1].pose.position.y - ps0.pose.position.y
            n = math.hypot(dx, dy) or 1.0
            fx, fy = dx / n, dy / n
        else:
            yaw = yaw_from_qzw(ps0.pose.orientation.z, ps0.pose.orientation.w)
            fx, fy = math.cos(yaw), math.sin(yaw)
        bump = 0.25 * self.retry
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = self.global_frame
        ip.pose.pose = ps0.pose
        ip.pose.pose.position.x = ps0.pose.position.x + fx * bump
        ip.pose.pose.position.y = ps0.pose.position.y + fy * bump
        self.init_pub.publish(ip)
        time.sleep(0.1)

    def _clear_costmaps(self):
        for client in (self.clear_local, self.clear_global):
            if client.wait_for_service(timeout_sec=1.0):
                try:
                    client.call_async(Empty.Request())
                except Exception:
                    pass
        self.get_logger().info("🧹 Costmaps limpiados")

    # --- Acción CONTINUA: NavigateThroughPoses ---------------------
    def _send_ntp_goal(self):
        self.get_logger().info("🚀  /navigate_through_poses (fluido)")
        if not self.ntp_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "⛔  No disponible /navigate_through_poses; fallback a /follow_waypoints"
            )
            return self._send_fw_goal()
        goal = NavigateThroughPoses.Goal()
        goal.poses = self.poses
        fut = self.ntp_client.send_goal_async(goal, feedback_callback=self._fb_ntp)
        fut.add_done_callback(self._goal_resp_ntp)

    def _goal_resp_ntp(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("⛔  Goal rechazado")
            return self._retry_or_finish()
        self.get_logger().info("✅  Goal aceptado")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self._on_ntp_result)

    def _fb_ntp(self, fb):
        # feedback opcional (distance_remaining, etc.)
        pass

    def _on_ntp_result(self, future):
        resp = future.result()
        status = getattr(resp, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁  Recorrido terminado (SUCCESS)")
            rclpy.shutdown()
            return
        self.get_logger().warn(f"⚠️  Goal terminó con estado {status}. Reintentando…")
        self._retry_or_finish()

    # --- Fallback discreto si NTP no existe -----------------------
    def _send_fw_goal(self):
        self.get_logger().info("🚀  /follow_waypoints (fallback)")
        if not self.fw_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("⛔  Tampoco está /follow_waypoints")
            return rclpy.shutdown()
        goal = FollowWaypoints.Goal()
        goal.poses = self.poses
        fut = self.fw_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_resp_fw)

    def _goal_resp_fw(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("⛔  Goal rechazado (FW)")
            return self._retry_or_finish()
        self.get_logger().info("✅  Goal aceptado (FW)")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self._on_fw_result)

    def _on_fw_result(self, future):
        resp = future.result()
        status = getattr(resp, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁  Recorrido terminado (FW SUCCESS)")
            rclpy.shutdown()
            return
        self.get_logger().warn(f"⚠️  FW terminó con estado {status}. Reintentando…")
        self._retry_or_finish()

    # --- Reintentos ------------------------------------------------
    def _retry_or_finish(self):
        if self.retry >= self.max_retries:
            self.get_logger().error("⛔  Reintentos agotados. Recorrido terminado.")
            rclpy.shutdown()
            return
        self.retry += 1
        self.get_logger().warn(
            f"🔁 Reintento {self.retry}/{self.max_retries}: limpiar costmaps y adelantar pose"
        )
        self._clear_costmaps()
        self._republish_initial_pose_bump()
        time.sleep(0.2)
        self._send_ntp_goal()


def main():
    if "--file" not in sys.argv:
        print("Requiere --file /ruta/archivo.yaml", file=sys.stderr)
        sys.exit(1)
    yaml_file = sys.argv[sys.argv.index("--file") + 1]
    rclpy.init()
    node = Player(yaml_file)
    rclpy.spin(node)
