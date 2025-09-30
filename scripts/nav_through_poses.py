#!/usr/bin/env python3
import sys, math, yaml, rclpy, signal
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from builtin_interfaces.msg import Time as TimeMsg
from nav2_msgs.action import NavigateThroughPoses

def q_from_yaw(yaw: float):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))

def dist2(a: PoseStamped, b: PoseStamped) -> float:
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    return dx*dx + dy*dy

def yaw_towards(a: PoseStamped, b: PoseStamped) -> float:
    dx = b.pose.position.x - a.pose.position.x
    dy = b.pose.position.y - a.pose.position.y
    return math.atan2(dy, dx)

class NTPClient(Node):
    def __init__(self, yaml_file: str, frame_id: str = "map", min_dist: float = 0.8):
        super().__init__("nav_through_poses_client")
        self._shutdown_called = False
        self._ac = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        raw = self._load_yaml(yaml_file, frame_id)
        self._poses = self._downsample_and_reorient(raw, min_dist)
        self.get_logger().info(f"Leyendo {yaml_file} – {len(self._poses)} puntos (NTP)")
        self._send_goal()

    def _load_yaml(self, path, frame_id):
        data = yaml.safe_load(open(path, "r"))
        poses = []
        for w in data.get("waypoints", []):
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = TimeMsg(sec=0, nanosec=0)
            ps.pose.position.x = float(w["x"])
            ps.pose.position.y = float(w["y"])
            # yaw original del YAML (por si el último lo quieres conservar)
            yaw = float(w.get("yaw", 0.0))
            q = q_from_yaw(yaw)
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            poses.append(ps)
        if not poses:
            raise RuntimeError("El YAML no contiene 'waypoints'.")
        return poses

    def _downsample_and_reorient(self, poses, min_dist):
        """Guarda el primero y el último; intermedios sólo si están a >= min_dist.
           Además, reorienta cada intermedio hacia su siguiente."""
        keep = [poses[0]]
        min2 = min_dist * min_dist
        last_kept = poses[0]
        for i in range(1, len(poses)-1):
            if dist2(poses[i], last_kept) >= min2:
                keep.append(poses[i])
                last_kept = poses[i]
        keep.append(poses[-1])

        # Reorientar intermedios hacia el siguiente
        for i in range(0, len(keep)-1):
            yaw = yaw_towards(keep[i], keep[i+1])
            q = q_from_yaw(yaw)
            keep[i].pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        # El último mantiene su yaw (del YAML)
        return keep

    def _send_goal(self):
        self.get_logger().info("🚀  Esperando servidor /navigate_through_poses …")
        self._ac.wait_for_server()
        now = self.get_clock().now().to_msg()
        for ps in self._poses:
            ps.header.stamp = now

        goal = NavigateThroughPoses.Goal(poses=self._poses, behavior_tree="")
        self.get_logger().info("🚀  Enviando acción /navigate_through_poses …")
        sfut = self._ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        sfut.add_done_callback(self._on_goal_response)

    def _on_feedback(self, fb):
        i = fb.feedback.current_pose
        self.get_logger().debug(f"➡️  Progreso NTP: x={i.pose.position.x:.2f}, y={i.pose.position.y:.2f}")

    def _on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("⛔  Acción NTP rechazada")
            self._safe_shutdown(); return
        self.get_logger().info("✅  Acción NTP aceptada")
        rf = gh.get_result_async()
        rf.add_done_callback(lambda *_: (self.get_logger().info("🏁  NTP terminado"), self._safe_shutdown()))

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
        print("Uso: nav_through_poses.py --file /routes/mi_ruta.yaml [--frame map] [--min-dist 0.8]", file=sys.stderr)
        sys.exit(1)
    yaml_file = sys.argv[sys.argv.index("--file") + 1]
    frame = "map"
    if "--frame" in sys.argv:
        frame = sys.argv[sys.argv.index("--frame") + 1]
    min_dist = 0.8
    if "--min-dist" in sys.argv:
        min_dist = float(sys.argv[sys.argv.index("--min-dist") + 1])
    rclpy.init()
    node = NTPClient(yaml_file, frame, min_dist)
    signal.signal(signal.SIGINT, lambda *_: node._safe_shutdown())
    try:
        rclpy.spin(node)
    finally:
        try: node.destroy_node()
        except Exception: pass

if __name__ == "__main__":
    main()
