#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

qos_rel = QoSProfile(depth=10)
qos_rel.reliability = QoSReliabilityPolicy.RELIABLE
qos_rel.history = QoSHistoryPolicy.KEEP_LAST
qos_rel.durability = QoSDurabilityPolicy.VOLATILE

qos_tf = QoSProfile(depth=100)
qos_tf.reliability = QoSReliabilityPolicy.RELIABLE
qos_tf.history = QoSHistoryPolicy.KEEP_LAST
qos_tf.durability = QoSDurabilityPolicy.VOLATILE

class Bridge(Node):
    def __init__(self):
        super().__init__('tf_odom_bridge')
        self.br = TransformBroadcaster(self, qos=qos_tf)
        self.sub = self.create_subscription(Odometry, '/odometry/filtered', self.cb, qos_rel)
        self.get_logger().info('TF bridge activo: publicando odom→base_link a partir de /odometry/filtered')

    def cb(self, msg: Odometry):
        child = msg.child_frame_id or 'base_link'
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = child
        t.transform.translation.x = float(msg.pose.pose.position.x)
        t.transform.translation.y = float(msg.pose.pose.position.y)
        t.transform.translation.z = float(msg.pose.pose.position.z)
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    try:
        rclpy.spin(Bridge())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
