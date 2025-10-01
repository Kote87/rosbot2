#!/usr/bin/env python3
import sys, math, yaml, argparse, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from builtin_interfaces.msg import Time as TimeMsg


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def make_pose(frame_id, x, y, yaw):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = TimeMsg(sec=0, nanosec=0)
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    q = quaternion_from_yaw(float(yaw))
    ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return ps


class Player(Node):
    def __init__(self, args):
        super().__init__("path_player")
        self.args = args
        self.declare_parameter("global_frame", "map")
        self.global_frame = self.get_parameter("global_frame").value

        # Cargar YAML
        with open(args.file, "r") as f:
            data = yaml.safe_load(f)
        self.waypoints = [
            make_pose(self.global_frame, w["x"], w["y"], w["yaw"])
            for w in data["waypoints"]
        ]
        self.get_logger().info(f"Leyendo {args.file} – {len(self.waypoints)} puntos")

        # Clientes de acción
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.wps_client = ActionClient(self, FollowWaypoints, "/follow_waypoints")

        # Publisher de initialpose opcional
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

        self._run()

    def _publish_initialpose(self, pose: PoseStamped):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = pose.header.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose = pose.pose
        # varianzas moderadas
        m.pose.covariance[0] = 0.25  # x
        m.pose.covariance[7] = 0.25  # y
        m.pose.covariance[35] = 0.12 # yaw
        self.init_pub.publish(m)
        self.get_logger().info("📍 Publicado initialpose (hint)")

    def _go_to(self, pose: PoseStamped) -> bool:
        self.nav_client.wait_for_server()
        g = NavigateToPose.Goal()
        g.pose = pose
        self.get_logger().info("🚀 NavigateToPose → y:{:.2f} x:{:.2f}".format(pose.pose.position.y, pose.pose.position.x))
        send_future = self.nav_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("⛔ NavigateToPose rechazado")
            return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result()
        self.get_logger().info("✅ NavigateToPose terminado con status {}".format(result.status))
        return True

    def _run(self):
        # Opcional: initialpose
        if self.args.set_initialpose != "none":
            if self.args.set_initialpose == "first":
                self._publish_initialpose(self.waypoints[0])
            elif self.args.set_initialpose == "zero":
                self._publish_initialpose(make_pose(self.global_frame, 0.0, 0.0, 0.0))

        # Opcional: ir al inicio
        if self.args.go_to_start != "none":
            if self.args.go_to_start == "first":
                self._go_to(self.waypoints[0])
            elif self.args.go_to_start == "zero":
                self._go_to(make_pose(self.global_frame, 0.0, 0.0, 0.0))

        # Sella timestamps y lanza FollowWaypoints
        self.wps_client.wait_for_server()
        now = self.get_clock().now().to_msg()
        for ps in self.waypoints:
            ps.header.stamp = now
        goal = FollowWaypoints.Goal(poses=self.waypoints)
        self.get_logger().info("🏁 Enviando /follow_waypoints …")
        send_future = self.wps_client.send_goal_async(goal, feedback_callback=self._feedback)
        send_future.add_done_callback(self._result_cb)

    def _feedback(self, fb):
        idx = fb.feedback.current_waypoint
        self.get_logger().info(f"➡️  Waypoint {idx}")

    def _result_cb(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("⛔  FollowWaypoints rechazado")
            rclpy.shutdown(); return
        self.get_logger().info("✅  FollowWaypoints aceptado")
        res_future = handle.get_result_async()
        res_future.add_done_callback(lambda *_: (self.get_logger().info("🏁 Terminado"), rclpy.shutdown()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", required=True)
    ap.add_argument("--go-to-start", choices=["first", "zero", "none"], default="first",
                    help="Navega al inicio antes de seguir la ruta (first=primer waypoint, zero=(0,0,0), none=directo a waypoints)")
    ap.add_argument("--set-initialpose", choices=["first", "zero", "none"], default="none",
                    help="Publica initialpose antes de navegar")
    args = ap.parse_args()

    rclpy.init()
    Player(args)
    rclpy.spin(rclpy.get_default_context())


if __name__ == "__main__":
    main()
