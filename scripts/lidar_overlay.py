#!/usr/bin/env python3
import rclpy, cv2, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from sensor_msgs_py import point_cloud2 as pc2
from laser_geometry import LaserProjection
from tf2_ros import Buffer, TransformListener, Time
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

class Overlay(Node):
    def __init__(self):
        super().__init__('lidar_overlay')
        self.lp, self.br = LaserProjection(), CvBridge()
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.scan = self.cam_info = None
        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(LaserScan,  '/scan',  self.save_scan, qos)
        self.create_subscription(CameraInfo, '/oak/rgb/camera_info', self.save_info, qos)
        self.create_subscription(Image,      '/oak/rgb/image_raw',  self.img_cb, qos)
        self.pub = self.create_publisher(Image, '/overlay', 10)

    def save_scan(self, m):  self.scan = m
    def save_info(self, m):  self.cam_info = m

    def img_cb(self, img):
        if not (self.scan and self.cam_info): return
        tr = self.tf.lookup_transform(self.cam_info.header.frame_id,
                                      self.scan.header.frame_id,
                                      Time())
        t = tr.transform.translation
        trans = np.array([t.x, t.y, t.z])
        q = tr.transform.rotation
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        cloud = self.lp.projectLaser(self.scan)
        pts = pc2.read_points(cloud, field_names=('x','y','z'), skip_nans=True)

        cv_img = self.br.imgmsg_to_cv2(img)
        h, w   = cv_img.shape[:2]
        fx, fy, cx, cy = (self.cam_info.k[0], self.cam_info.k[4],
                          self.cam_info.k[2], self.cam_info.k[5])

        for x0, y0, z0 in pts:
            x, y, z = rot @ np.array([x0, y0, z0]) + trans
            if z <= 0: continue
            u, v = int(fx * x / z + cx), int(fy * y / z + cy)
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(cv_img, (u, v), 2, (0, 255, 0), -1)

        out = self.br.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out.header = img.header          # copia frame_id / timestamp
        self.pub.publish(out)

def main():
    rclpy.init(); rclpy.spin(Overlay()); rclpy.shutdown()
if __name__ == '__main__':
    main()
