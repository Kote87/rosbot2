#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import Spin
from std_srvs.srv import Empty


class AutoLocalize(Node):
    def __init__(self, timeout: float = 45.0, std_xy: float = 0.30, std_yaw: float = 0.5) -> None:
        super().__init__("auto_localize")
        self.timeout = timeout
        self.std_xy = std_xy
        self.std_yaw = std_yaw
        self.best = None

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._pose_cb, 10
        )
        self.spin_client = ActionClient(self, Spin, "/spin")
        self.cli_gl1 = self.create_client(Empty, "/reinitialize_global_localization")
        self.cli_gl2 = self.create_client(Empty, "/amcl/global_localization")

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        cov = msg.pose.covariance
        sx = math.sqrt(max(cov[0], 0.0))
        sy = math.sqrt(max(cov[7], 0.0))
        syaw = math.sqrt(max(cov[35], 0.0))
        self.best = (sx, sy, syaw)

    def _call_global_localization(self) -> bool:
        if self.cli_gl1.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Calling /reinitialize_global_localization")
            req = Empty.Request()
            self.cli_gl1.call_async(req)
            return True
        if self.cli_gl2.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Calling /amcl/global_localization")
            req = Empty.Request()
            self.cli_gl2.call_async(req)
            return True
        self.get_logger().warn("No global localization service available")
        return False

    def _spin(self, turns: float = 2.5, yaw_rate: float = 1.0):
        self.spin_client.wait_for_server()
        goal = Spin.Goal()
        goal.target_yaw = float(2.0 * math.pi * turns)
        duration = int(2 * turns * math.pi / max(yaw_rate, 1e-3)) + 10
        goal.time_allowance = Duration(seconds=duration).to_msg()
        return self.spin_client.send_goal_async(goal)

    def run(self) -> int:
        start = time.time()
        self._call_global_localization()
        goal_future = self._spin(turns=2.5, yaw_rate=0.8)

        while rclpy.ok() and (time.time() - start) < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.best:
                sx, sy, syaw = self.best
                if sx < self.std_xy and sy < self.std_xy and syaw < self.std_yaw:
                    self.get_logger().info(
                        f"✅ AMCL converged: σx={sx:.2f}, σy={sy:.2f}, σyaw={syaw:.2f}"
                    )
                    try:
                        goal_handle = goal_future.result()
                        if goal_handle:
                            goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                    return 0
        self.get_logger().warn("⏱️  AMCL convergence timeout — proceeding anyway")
        return 0


def main() -> None:
    rclpy.init()
    node = AutoLocalize(timeout=45.0, std_xy=0.30, std_yaw=0.5)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
