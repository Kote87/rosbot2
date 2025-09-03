#!/usr/bin/env python3
"""
Reproduce un recorrido YAML mediante la acción /follow_waypoints.
Uso:
  ros2 run path_tools path_player.py --file /routes/nombre.yaml
"""
import sys, time, yaml, math, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
import geometry_msgs.msg
from nav2_msgs.action import FollowWaypoints
from builtin_interfaces.msg import Time as TimeMsg
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid


def quaternion_from_yaw(yaw):
    """Devuelve (x,y,z,w) para yaw dado (roll=pitch=0)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_qzw(qz, qw):
    # roll=pitch=0 asumido
    return 2.0 * math.atan2(qz, qw)


class Player(Node):
    def __init__(self, yaml_file: str):
        super().__init__("path_player")
        self.declare_parameter("global_frame", "map")
        self.global_frame = self.get_parameter("global_frame").value
        # Cargar YAML
        self.waypoints = self._load_yaml(yaml_file)
        self.client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self.get_logger().info(f"Leyendo {yaml_file} – {len(self.waypoints)} puntos")

        # ---- NUEVO: servicios para limpiar costmaps y mapa estático ----
        self.clear_local = self.create_client(Empty, "/local_costmap/clear_entirely_local_costmap")
        self.clear_global = self.create_client(Empty, "/global_costmap/clear_entirely_global_costmap")
        self.map_msg = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos_profile=1)

        # Publisher de pose inicial para poder reintentar
        self.init_pub = self.create_publisher(
            geometry_msgs.msg.PoseWithCovarianceStamped, "/initialpose", qos_profile=1
        )

        # Parámetros de reintento
        self.max_retries = 3
        self.retry_count = 0

        # Publica pose inicial (con "empuje" hacia delante si hace falta) y lanza goal
        self._publish_initial_pose_forward()
        self._clear_costmaps()
        self._send_goal()

    # --- YAML → lista PoseStamped ----------------------------------
    def _load_yaml(self, f):
        data = yaml.safe_load(open(f))
        poses = []
        for i, w in enumerate(data["waypoints"]):
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame
            ps.header.stamp = TimeMsg(sec=0, nanosec=0)
            ps.pose.position.x = float(w["x"])
            ps.pose.position.y = float(w["y"])
            q = quaternion_from_yaw(float(w["yaw"]))
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            poses.append(ps)
        return poses

    # --- Mapa ------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _is_free_on_static(self, x, y):
        """Devuelve True si (x,y) cae en celda libre (0) del /map; False si desconocido (-1) u ocupada (100)."""
        m = self.map_msg
        if m is None:
            return True  # si aún no llegó el mapa, no bloqueamos
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or my < 0 or mx >= m.info.width or my >= m.info.height:
            return False
        val = m.data[my * m.info.width + mx]
        return val == 0

    # --- Pose inicial con pequeño avance hacia delante -------------
    def _publish_initial_pose_forward(self):
        ps0 = self.waypoints[0]
        # vector hacia delante: preferimos segundo punto; si no, usamos yaw del primero
        if len(self.waypoints) >= 2:
            dx = self.waypoints[1].pose.position.x - ps0.pose.position.x
            dy = self.waypoints[1].pose.position.y - ps0.pose.position.y
            norm = math.hypot(dx, dy) or 1.0
            fx, fy = dx / norm, dy / norm
        else:
            yaw = yaw_from_qzw(ps0.pose.orientation.z, ps0.pose.orientation.w)
            fx, fy = math.cos(yaw), math.sin(yaw)

        # probamos offsets hacia delante: 0.00, 0.15, 0.30, 0.45, 0.60 m
        candidates = [0.00, 0.15, 0.30, 0.45, 0.60]
        x0, y0 = ps0.pose.position.x, ps0.pose.position.y
        chosen = (x0, y0)
        for off in candidates:
            cx, cy = x0 + fx * off, y0 + fy * off
            if self._is_free_on_static(cx, cy):
                chosen = (cx, cy)
                break

        ipose = geometry_msgs.msg.PoseWithCovarianceStamped()
        ipose.header.frame_id = self.global_frame
        ipose.pose.pose = ps0.pose
        ipose.pose.pose.position.x = chosen[0]
        ipose.pose.pose.position.y = chosen[1]
        # Covarianzas: 5 cm y ~5°
        ipose.pose.covariance[0] = 0.05 ** 2
        ipose.pose.covariance[7] = 0.05 ** 2
        ipose.pose.covariance[35] = (math.radians(5)) ** 2
        for _ in range(3):
            self.init_pub.publish(ipose)
            time.sleep(0.1)
        if (chosen[0], chosen[1]) != (x0, y0):
            self.get_logger().info(
                f"📍 Pose inicial adelantada {math.hypot(chosen[0]-x0, chosen[1]-y0):.2f} m para evitar celda letal."
            )
        else:
            self.get_logger().info("📍 Pose inicial publicada")

    def _republish_initial_pose_more_forward(self):
        """En reintentos, probar un poco más de avance."""
        self.get_logger().warn("Reintentando: adelantando pose inicial +0.25 m")
        # Avanza 0.25 m adicionales desde el primer punto
        ps0 = self.waypoints[0]
        if len(self.waypoints) >= 2:
            dx = self.waypoints[1].pose.position.x - ps0.pose.position.x
            dy = self.waypoints[1].pose.position.y - ps0.pose.position.y
            norm = math.hypot(dx, dy) or 1.0
            fx, fy = dx / norm, dy / norm
        else:
            yaw = yaw_from_qzw(ps0.pose.orientation.z, ps0.pose.orientation.w)
            fx, fy = math.cos(yaw), math.sin(yaw)
        bump = 0.25 * self.retry_count
        ipose = geometry_msgs.msg.PoseWithCovarianceStamped()
        ipose.header.frame_id = self.global_frame
        ipose.pose.pose = ps0.pose
        ipose.pose.pose.position.x = ps0.pose.position.x + fx * bump
        ipose.pose.pose.position.y = ps0.pose.position.y + fy * bump
        self.init_pub.publish(ipose)
        time.sleep(0.1)

    def _clear_costmaps(self):
        for client, name in [(self.clear_local, "local"), (self.clear_global, "global")]:
            if client.wait_for_service(timeout_sec=1.0):
                try:
                    client.call_async(Empty.Request())
                except Exception:
                    pass
            else:
                self.get_logger().warn(f"No disponible clear_entirely_{name}_costmap")

    # --- Acción FollowWaypoints ------------------------------------
    def _send_goal(self):
        self.client.wait_for_server()
        goal_msg = FollowWaypoints.Goal(poses=self.waypoints)
        self.get_logger().info("🚀  Enviando acción /follow_waypoints …")
        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("⛔  Acción rechazada")
            self._retry_or_finish()
            return
        self.get_logger().info("✅  Acción aceptada")
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._on_result)

    def _feedback(self, feedback):
        idx = feedback.feedback.current_waypoint
        self.get_logger().debug(f"➡️  Llegando a waypoint {idx}")

    def _on_result(self, future):
        resp = future.result()
        status = getattr(resp, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁  Recorrido terminado (SUCCESS)")
            rclpy.shutdown()
            return
        # Si falló (abort/cancel/unknown), reintenta: limpiar, adelantar pose y volver a enviar
        self.get_logger().warn(
            f"⚠️  Goal terminó con estado {status}. Intentando recuperación…"
        )
        self._retry_or_finish()

    def _retry_or_finish(self):
        if self.retry_count >= self.max_retries:
            self.get_logger().error("⛔  Reintentos agotados. Recorrido terminado.")
            rclpy.shutdown()
            return
        self.retry_count += 1
        self._clear_costmaps()
        self._republish_initial_pose_more_forward()
        time.sleep(0.2)
        self._send_goal()


def main():
    if "--file" not in sys.argv:
        print("Requiere --file /ruta/archivo.yaml", file=sys.stderr)
        sys.exit(1)
    yaml_file = sys.argv[sys.argv.index("--file") + 1]
    rclpy.init()
    node = Player(yaml_file)
    rclpy.spin(node)


if __name__ == "__main__":
    main()

