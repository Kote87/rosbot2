#!/usr/bin/env python3
"""
Reproduce un recorrido YAML mediante la acción /follow_waypoints.
Uso:
  ros2 run path_tools path_player.py --file /routes/nombre.yaml
"""
import sys, time, yaml, math, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
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
        self._waypoints = self._load_yaml(yaml_file)
        self.client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, qos_profile=10
        )
        self.get_logger().info(
            f"Leyendo {yaml_file} – {len(self._waypoints)} puntos"
        )
        self._send_waypoints()

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
    def _send_waypoints(self):
        first = self._waypoints[0]
        if self.current_pose and self._distance_xy(self.current_pose, first) > 0.5:
            self.get_logger().info("⚠️  Lejos del inicio → goal de aproximación")
            self._send_nav_goal(first)

        self.client.wait_for_server()
        goal_msg = FollowWaypoints.Goal(poses=self._waypoints)
        self.get_logger().info("🚀  Enviando acción /follow_waypoints …")
        send_future = self.client.send_goal_async(
            goal_msg, feedback_callback=self._feedback
        )
        send_future.add_done_callback(self._result_cb)

    def _feedback(self, feedback):
        idx = feedback.feedback.current_waypoint
        self.get_logger().debug(f"➡️  Llegando a waypoint {idx}")

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    @staticmethod
    def _distance_xy(pose, target: PoseStamped):
        pos = target.pose.position
        return math.hypot(pose.position.x - pos.x, pose.position.y - pos.y)

    def _send_nav_goal(self, pose: PoseStamped):
        self.nav_client.wait_for_server()
        goal = NavigateToPose.Goal(pose=pose)
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("⛔  Goal de aproximación rechazado")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

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
    node = Player(yaml_file)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
