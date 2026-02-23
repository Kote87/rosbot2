#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose, FollowWaypoints, NavigateThroughPoses


class Probe(Node):
    def __init__(self, timeout: float):
        super().__init__("nav2_action_probe")
        self.timeout = timeout

        self.clients = [
            ("navigate_to_pose", ActionClient(self, NavigateToPose, "/navigate_to_pose")),
            ("follow_waypoints", ActionClient(self, FollowWaypoints, "/follow_waypoints")),
            ("navigate_through_poses", ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")),
        ]

    def run(self) -> int:
        ok_all = True
        print("\n=== Nav2 Action Probe (sin movimiento) ===")
        for name, c in self.clients:
            ok = c.wait_for_server(timeout_sec=self.timeout)
            print(("✅" if ok else "⛔"), f"/{name} server", "OK" if ok else f"NO (timeout {self.timeout}s)")
            ok_all = ok_all and ok
        return 0 if ok_all else 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.timeout)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
