#!/usr/bin/env python3
import sys
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses


class PlayerNTP(Node):
    def __init__(self, yaml_file: str) -> None:
        super().__init__('path_player_ntp')
        self.cli = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.get_logger().info(f"Leyendo {yaml_file}")

        with open(yaml_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        poses = []
        for wp in data.get('waypoints', []):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(wp['x'])
            ps.pose.position.y = float(wp['y'])
            ps.pose.orientation.z = float(wp.get('oz', 0.0))
            ps.pose.orientation.w = float(wp.get('ow', 1.0))
            poses.append(ps)

        rclpy.spin_until_future_complete(self, self.cli.wait_for_server())

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        self.get_logger().info(f"🚀 Enviando NTP con {len(poses)} poses")
        send_future = self.cli.send_goal_async(goal)
        send_future.add_done_callback(self._goal_sent)

    def _goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ NTP no aceptado')
            rclpy.shutdown()
            return

        self.get_logger().info('✅ NTP aceptado')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_finished)

    def _goal_finished(self, future):
        _ = future.result()
        self.get_logger().info('🏁 NTP terminado')
        rclpy.shutdown()


def main() -> None:
    if '--file' not in sys.argv:
        print('Uso: path_player_ntp.py --file /routes/mi_ruta.yaml')
        sys.exit(1)

    yaml_path = sys.argv[sys.argv.index('--file') + 1]

    rclpy.init()
    node = PlayerNTP(yaml_path)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
