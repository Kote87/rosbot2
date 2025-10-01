#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import Spin
from std_srvs.srv import Empty

class AutoLocalize(Node):
    def __init__(self, timeout=45.0, std_xy=0.30, std_yaw=0.5):
        super().__init__("auto_localize")
        self.timeout, self.std_xy, self.std_yaw = timeout, std_xy, std_yaw
        self.best = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._pose_cb, qos)
        self.spin_client = ActionClient(self, Spin, "/spin")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cli_gl1 = self.create_client(Empty, "/reinitialize_global_localization")
        self.cli_gl2 = self.create_client(Empty, "/amcl/global_localization")

    def _pose_cb(self, m):
        c = m.pose.covariance
        sx, sy, syaw = (max(c[0],0.0)**0.5, max(c[7],0.0)**0.5, max(c[35],0.0)**0.5)
        self.best = (sx, sy, syaw)

    def _call_global_localization(self):
        req = Empty.Request()
        if self.cli_gl1.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Calling /reinitialize_global_localization")
            self.cli_gl1.call_async(req); return True
        if self.cli_gl2.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Calling /amcl/global_localization")
            self.cli_gl2.call_async(req); return True
        self.get_logger().warn("No global localization service available"); return False

    def _spin_action_or_fallback(self, secs=20.0):
        # Espera breve a que /spin exista
        ok = self.spin_client.wait_for_server(timeout_sec=5.0)
        if ok:
            from rclpy.duration import Duration
            goal = Spin.Goal(); goal.target_yaw = float(2.5*2.0*math.pi)
            goal.time_allowance = Duration(seconds=int(secs)+10).to_msg()
            self.spin_client.send_goal_async(goal)
            return
        # Fallback: gira con cmd_vel
        self.get_logger().warn("No /spin action — fallback to cmd_vel yaw rotation")
        t0 = time.time()
        tw = Twist(); tw.angular.z = 0.8
        while rclpy.ok() and (time.time() - t0) < secs:
            self.cmd_pub.publish(tw)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())  # stop

    def run(self):
        t0 = time.time()
        self._call_global_localization()
        self._spin_action_or_fallback(secs=25.0)

        while rclpy.ok() and (time.time() - t0) < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.best:
                sx, sy, syaw = self.best
                if sx < self.std_xy and sy < self.std_xy and syaw < self.std_yaw:
                    self.get_logger().info(f"✅ AMCL converged: σx={sx:.2f}, σy={sy:.2f}, σyaw={syaw:.2f}")
                    return 0
        self.get_logger().warn("⏱️ AMCL convergence timeout — proceeding anyway")
        return 0

def main():
    rclpy.init(); node = AutoLocalize()
    try: code = node.run()
    finally:
        node.destroy_node(); rclpy.shutdown()
    raise SystemExit(code)

if __name__ == "__main__":
    main()
