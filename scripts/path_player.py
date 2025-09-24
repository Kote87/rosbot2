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
from nav2_msgs.action import FollowWaypoints
from builtin_interfaces.msg import Time as TimeMsg


def quaternion_from_yaw(yaw):
    """Devuelve (x,y,z,w) para yaw dado (roll=pitch=0)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class Player(Node):
    def __init__(self, yaml_file: str):
        super().__init__("path_player")
        self.declare_parameter("global_frame", "map")
        self.global_frame = self.get_parameter("global_frame").value
        # Cargar YAML
        self.waypoints = self._load_yaml(yaml_file)
        self.client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self.get_logger().info(f"Leyendo {yaml_file} – {len(self.waypoints)} puntos")
        self._send_goal()

    # --- YAML → lista PoseStamped ----------------------------------
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

    # --- Acción FollowWaypoints ------------------------------------
    def _send_goal(self):
        self.client.wait_for_server()
        goal_msg = FollowWaypoints.Goal(poses=self.waypoints)
        self.get_logger().info("🚀  Enviando acción /follow_waypoints …")
        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)
        send_future.add_done_callback(self._result_cb)

    def _feedback(self, feedback):
        idx = feedback.feedback.current_waypoint
        self.get_logger().debug(f"➡️  Llegando a waypoint {idx}")

    def _result_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("⛔  Acción rechazada")
            rclpy.shutdown()
            return
        self.get_logger().info("✅  Acción aceptada")
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(
            lambda *_: (self.get_logger().info("🏁  Recorrido terminado"), rclpy.shutdown())
        )


def main():
    if "--file" not in sys.argv:
        print("Requiere --file /ruta/archivo.yaml", file=sys.stderr)
        sys.exit(1)
    yaml_file = sys.argv[sys.argv.index("--file") + 1]
    rclpy.init()
    Player(yaml_file)
    rclpy.spin(Player)


if __name__ == "__main__":
    main()
