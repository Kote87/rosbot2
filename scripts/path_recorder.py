#!/usr/bin/env python3
"""
Graba waypoints a partir de /amcl_pose mientras el robot se mueve.
Uso:
  ros2 run path_tools path_recorder.py --output /routes/nombre.yaml
Se detiene con Ctrl‑C.
"""
import sys, math, yaml, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quaternion(q):
    """Convierte un quaternion (x,y,z,w) en yaw (rad)."""
    x, y, z, w = q
    # Fórmula estándar yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

DIST_THRESHOLD = 0.50  # metros entre muestras


class Recorder(Node):
    def __init__(self, out_file: str):
        super().__init__("path_recorder")
        self.out_file = out_file
        self.last_pose = None
        self.path = []
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, qos_profile=10
        )
        self.get_logger().info(f"▶️  Grabando → {out_file}")

    # --- callbacks -------------------------------------------------
    def on_pose(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        if self.last_pose is None:
            self._store(pose)
            return
        if self._dist(pose, self.last_pose) >= DIST_THRESHOLD:
            self._store(pose)

    # --- helpers ---------------------------------------------------
    @staticmethod
    def _dist(p, q):
        return math.hypot(p.position.x - q.position.x, p.position.y - q.position.y)

    def _store(self, pose):
        q = pose.orientation
        yaw = yaw_from_quaternion([q.x, q.y, q.z, q.w])
        self.path.append(dict(x=pose.position.x, y=pose.position.y, yaw=float(yaw)))
        self.last_pose = pose

    # --- finish ----------------------------------------------------
    def shutdown(self):
        if not self.path:
            self.get_logger().warn("Ningún punto grabado; fichero vacío.")
            return
        yaml.safe_dump({"waypoints": self.path}, open(self.out_file, "w"), sort_keys=False)
        self.get_logger().info(f"⏹️  Guardado {len(self.path)} puntos → {self.out_file}")


def main():
    if "--output" not in sys.argv:
        print("Requiere --output /ruta/archivo.yaml", file=sys.stderr)
        sys.exit(1)
    out_file = sys.argv[sys.argv.index("--output") + 1]
    rclpy.init()
    node = Recorder(out_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
