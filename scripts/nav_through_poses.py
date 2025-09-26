#!/usr/bin/env python3
import sys, math, yaml, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from builtin_interfaces.msg import Time as TimeMsg
from nav2_msgs.action import NavigateThroughPoses


def q_from_yaw(yaw):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))


class NTP(Node):
    def __init__(self, yaml_file: str):
        super().__init__('nav_through_poses_client')
        self.declare_parameter('global_frame', 'map')
        self.global_frame = self.get_parameter('global_frame').value
        self._poses = self._load(yaml_file)
        self._ac = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.get_logger().info(f'Leyendo {yaml_file} – {len(self._poses)} puntos (NTP)')
        self._send()

    def _load(self, path):
        data = yaml.safe_load(open(path))
        out = []
        for i, w in enumerate(data['waypoints']):
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame
            ps.header.stamp = TimeMsg(sec=0, nanosec=0)
            ps.pose.position.x = float(w['x'])
            ps.pose.position.y = float(w['y'])
            q = q_from_yaw(float(w['yaw']))
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            out.append(ps)
        return out

    def _send(self):
        self.get_logger().info('🚀  Enviando acción /navigate_through_poses …')
        self._ac.wait_for_server()
        goal = NavigateThroughPoses.Goal()
        goal.poses = self._poses
        goal.behavior_tree = ''  # usa el BT por defecto del bringup
        sfut = self._ac.send_goal_async(goal, feedback_callback=self._fb)
        sfut.add_done_callback(self._result)

    def _fb(self, fb):
        i = fb.feedback.current_pose
        self.get_logger().info(f'➡️  Progreso NTP: {i.pose.position.x:.2f},{i.pose.position.y:.2f}')

    def _result(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('⛔  Acción rechazada')
            rclpy.shutdown(); return
        self.get_logger().info('✅  Acción aceptada')
        rf = gh.get_result_async()
        rf.add_done_callback(lambda *_: (self.get_logger().info('🏁  NTP terminado'), rclpy.shutdown()))


def main():
    if "--file" not in sys.argv:
        print("Requiere --file /routes/archivo.yaml", file=sys.stderr); sys.exit(1)
    f = sys.argv[sys.argv.index("--file")+1]
    rclpy.init(); node = NTP(f); rclpy.spin(node)


if __name__ == '__main__':
    main()
