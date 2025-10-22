#!/usr/bin/env python3
import sys, math, yaml, rclpy, signal
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from builtin_interfaces.msg import Time as TimeMsg
from nav2_msgs.action import NavigateThroughPoses
import os


def q_from_yaw(yaw: float):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))


class NTPClient(Node):
    def __init__(self, yaml_file: str, frame_id: str = "map"):
        super().__init__("nav_through_poses_client")
        self._shutdown_called = False
        self._ac = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self._poses = self._load_yaml(yaml_file, frame_id)
        self.get_logger().info(f"Leyendo {yaml_file} – {len(self._poses)} puntos (NTP)")
        self._send_goal()

    def _load_yaml(self, path, frame_id):
        data = yaml.safe_load(open(path, "r"))
        poses = []
        for w in data.get("waypoints", []):
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = TimeMsg(sec=0, nanosec=0)  # se resella al enviar
            ps.pose.position.x = float(w["x"])
            ps.pose.position.y = float(w["y"])
            q = q_from_yaw(float(w.get("yaw", 0.0)))
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            poses.append(ps)
        if not poses:
            raise RuntimeError("El YAML no contiene 'waypoints'.")
        return poses

    def _send_goal(self):
        self.get_logger().info("🚀  Esperando servidor /navigate_through_poses …")
        self._ac.wait_for_server()
        # Sella timestamps actuales para evitar que downstream considere mensajes viejos
        now = self.get_clock().now().to_msg()
        for ps in self._poses:
            ps.header.stamp = now

        goal = NavigateThroughPoses.Goal()
        goal.poses = self._poses
        # Fuerza un BT que no hace GoalReached al principio (evita éxito instantáneo)
        bt_file = "/scripts/bt/ntp_force_follow.xml"
        if os.path.exists(bt_file):
            goal.behavior_tree = bt_file
            self.get_logger().info(f"Usando BT NTP forzado: {bt_file}")
        else:
            goal.behavior_tree = ""  # fallback: BT por defecto
            self.get_logger().warn("BT forzado no encontrado; usando BT por defecto")

        self.get_logger().info("🚀  Enviando acción /navigate_through_poses …")
        sfut = self._ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        sfut.add_done_callback(self._on_goal_response)

    def _on_feedback(self, fb):
        i = fb.feedback.current_pose
        self.get_logger().debug(
            f"➡️  Progreso NTP: x={i.pose.position.x:.2f}, y={i.pose.position.y:.2f}"
        )

    def _on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("⛔  Acción NTP rechazada")
            self._safe_shutdown()
            return
        self.get_logger().info("✅  Acción NTP aceptada")
        rf = gh.get_result_async()
        rf.add_done_callback(
            lambda *_: (
                self.get_logger().info("🏁  NTP terminado"),
                self._safe_shutdown(),
            )
        )

    def _safe_shutdown(self):
        if self._shutdown_called:
            return
        self._shutdown_called = True
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main():
    if "--file" not in sys.argv:
        print(
            "Uso: nav_through_poses.py --file /routes/mi_ruta.yaml [--frame map]",
            file=sys.stderr,
        )
        sys.exit(1)
    yaml_file = sys.argv[sys.argv.index("--file") + 1]
    frame = "map"
    if "--frame" in sys.argv:
        frame = sys.argv[sys.argv.index("--frame") + 1]
    rclpy.init()
    node = NTPClient(yaml_file, frame)
    # Ctrl-C → cancelación limpia sin 'double shutdown'
    signal.signal(signal.SIGINT, lambda *_: node._safe_shutdown())
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        # rclpy.shutdown() ya se invoca en _safe_shutdown()


if __name__ == "__main__":
    main()
