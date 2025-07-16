#!/usr/bin/env python3
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

L = 9.25  # largo (m)
W = 1.25  # ancho (m)
PAUSE = 0.5  # s entre vueltas


def mk_pose(x, y, th):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z = math.sin(th / 2)
    p.pose.orientation.w = math.cos(th / 2)
    return p


def build_rect(start):
    x0, y0 = start.pose.position.x, start.pose.position.y
    th0 = 2 * math.atan2(start.pose.orientation.z, start.pose.orientation.w)
    pts = []
    pts.append((x0 + L * math.cos(th0), y0 + L * math.sin(th0), th0 + math.pi / 2))
    pts.append(
        (
            pts[-1][0] + W * math.cos(th0 + math.pi / 2),
            pts[-1][1] + W * math.sin(th0 + math.pi / 2),
            th0 + math.pi,
        )
    )
    pts.append(
        (
            pts[-1][0] + L * math.cos(th0 + math.pi),
            pts[-1][1] + L * math.sin(th0 + math.pi),
            th0 + 1.5 * math.pi,
        )
    )
    pts.append(
        (
            pts[-1][0] + W * math.cos(th0 + 1.5 * math.pi),
            pts[-1][1] + W * math.sin(th0 + 1.5 * math.pi),
            th0,
        )
    )
    return [mk_pose(*p) for p in pts]


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    waypoints = build_rect(nav.getCurrentPose())
    while rclpy.ok():
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            time.sleep(0.2)
        time.sleep(PAUSE)


if __name__ == "__main__":
    main()
