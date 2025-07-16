#!/usr/bin/env python3
import math, rclpy, time, threading
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.duration import Duration

RECT_L = 9.25   # largo (m)
RECT_W = 1.25   # ancho (m)
PAUSE  = 0.5    # s entre tramos

def pose_from_xytheta(x, y, theta, frame="map"):
    """Devuelve PoseStamped con quaternion."""
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.z = math.sin(theta/2.0)
    ps.pose.orientation.w = math.cos(theta/2.0)
    return ps

def compute_waypoints(p0):
    """Genera lista de 4 poses (esquinas) a partir de la pose inicial."""
    x, y = p0.pose.position.x, p0.pose.position.y
    # θ0 en radianes
    q = p0.pose.orientation
    theta0 = 2*math.atan2(q.z, q.w)
    # vértices
    pts = []
    # 1) adelante L
    x1 = x + RECT_L*math.cos(theta0)
    y1 = y + RECT_L*math.sin(theta0)
    pts.append((x1, y1, theta0 + math.pi/2))
    # 2) izquierda W
    x2 = x1 + RECT_W*math.cos(theta0 + math.pi/2)
    y2 = y1 + RECT_W*math.sin(theta0 + math.pi/2)
    pts.append((x2, y2, theta0 + math.pi))
    # 3) atrás L (es realmente adelante en −x)
    x3 = x2 + RECT_L*math.cos(theta0 + math.pi)
    y3 = y2 + RECT_L*math.sin(theta0 + math.pi)
    pts.append((x3, y3, theta0 + 1.5*math.pi))
    # 4) derecha W
    x4 = x3 + RECT_W*math.cos(theta0 + 1.5*math.pi)
    y4 = y3 + RECT_W*math.sin(theta0 + 1.5*math.pi)
    pts.append((x4, y4, theta0))  # vuelve a orientación original
    return [pose_from_xytheta(*p) for p in pts]

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    init = nav.getCurrentPose()
    waypoints = compute_waypoints(init)

    while rclpy.ok():
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            time.sleep(0.2)
        time.sleep(PAUSE)          # pequeña pausa y bucle de nuevo

if __name__ == "__main__":
    main()
