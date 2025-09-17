#!/usr/bin/env python3
"""
Reproduce un recorrido YAML de forma FLUIDA:
 - Intenta primero NavigateThroughPoses (NTP)
 - Fallback a FollowWaypoints si NTP no existe
 - Limpia costmaps si falla y reintenta saltando waypoints
 - Permite saltarse N puntos iniciales (--skip-first), por defecto 1
Uso:
  python3 /scripts/path_player.py --file /routes/nombre.yaml [--skip-first N]
"""
import argparse, sys, math, time, yaml, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, FollowWaypoints
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class Player(Node):
    def __init__(self, yaml_file: str, skip_first: int = 1, max_retries: int = 3):
        super().__init__("path_player")
        self.declare_parameter("global_frame", "map")
        self.global_frame = self.get_parameter("global_frame").value
        self.max_retries = max_retries
        self.skip_first = max(0, int(skip_first))
        self.poses = self._load_poses(yaml_file)
        if self.skip_first > 0 and self.skip_first < len(self.poses):
            self.get_logger().info(f"⤴️  Saltando primeros {self.skip_first} waypoints")
            self.poses = self.poses[self.skip_first:]
        self.ntp = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self.fw = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self.clear_local = self.create_client(Empty, "/local_costmap/clear_entirely_local_costmap")
        self.clear_global = self.create_client(Empty, "/global_costmap/clear_entirely_global_costmap")
        # Start
        self._run()

    def _load_poses(self, path):
        data = yaml.safe_load(open(path))
        poses = []
        for w in data["waypoints"]:
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame
            ps.header.stamp = TimeMsg(sec=0, nanosec=0)  # sin tiempo → Nav2 rellena
            ps.pose.position.x = float(w["x"])
            ps.pose.position.y = float(w["y"])
            q = quaternion_from_yaw(float(w["yaw"]))
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            poses.append(ps)
        self.get_logger().info(f"Leyendo {path} – {len(poses)} puntos")
        return poses

    def _clear_costmaps(self):
        for cli in (self.clear_local, self.clear_global):
            if cli.wait_for_service(timeout_sec=1.0):
                try:
                    cli.call_async(Empty.Request())
                except Exception:
                    pass
        self.get_logger().info("🧹 Costmaps limpiados")

    def _run(self):
        # Intenta NTP (fluido), sino FW (discreto)
        if self.ntp.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("🚀  /navigate_through_poses")
            self._send_ntp(self.poses, retry=0)
        elif self.fw.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("ℹ️  NTP no disponible, usando /follow_waypoints")
            self._send_fw(self.poses, retry=0)
        else:
            self.get_logger().error("⛔  No hay servidores NTP ni FW disponibles")
            rclpy.shutdown()

    # ---------- NTP -----------
    def _send_ntp(self, poses, retry):
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        fut = self.ntp.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_ntp_accepted(f, poses, retry))

    def _on_ntp_accepted(self, future, poses, retry):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("⛔  NTP goal rechazado")
            return self._maybe_retry_or_fallback(poses, retry)
        self.get_logger().info("✅  NTP goal aceptado")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(lambda fr: self._on_ntp_result(fr, poses, retry))

    def _on_ntp_result(self, future, poses, retry):
        res = future.result()
        status = getattr(res, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁  Recorrido terminado (NTP SUCCESS)")
            rclpy.shutdown()
            return
        self.get_logger().warn(
            f"⚠️  NTP terminó con estado {status}. Saltando un punto y reintentando…"
        )
        self._clear_costmaps()
        self._skip_and_retry(poses, retry, use_ntp=True)

    # ---------- FW ------------
    def _send_fw(self, poses, retry):
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        fut = self.fw.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_fw_accepted(f, poses, retry))

    def _on_fw_accepted(self, future, poses, retry):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("⛔  FW goal rechazado")
            return self._maybe_retry_or_fallback(poses, retry, already_fw=True)
        self.get_logger().info("✅  FW goal aceptado")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(lambda fr: self._on_fw_result(fr, poses, retry))

    def _on_fw_result(self, future, poses, retry):
        res = future.result()
        status = getattr(res, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁  Recorrido terminado (FW SUCCESS)")
            rclpy.shutdown()
            return
        self.get_logger().warn(
            f"⚠️  FW terminó con estado {status}. Saltando un punto y reintentando…"
        )
        self._clear_costmaps()
        self._skip_and_retry(poses, retry, use_ntp=False)

    # ---------- Helpers -------
    def _maybe_retry_or_fallback(self, poses, retry, already_fw=False):
        if not already_fw and self.fw.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("↩️  Fallback a /follow_waypoints")
            return self._send_fw(poses, retry)
        self._skip_and_retry(poses, retry, use_ntp=not already_fw)

    def _skip_and_retry(self, poses, retry, use_ntp):
        if retry >= self.max_retries or len(poses) <= 1:
            self.get_logger().error("⛔  Reintentos agotados o sin más puntos. Terminando.")
            rclpy.shutdown()
            return
        new_poses = poses[1:]
        self.get_logger().info(
            f"⤴️  Saltando 1 waypoint (quedan {len(new_poses)}) y reintentando… ({retry + 1}/{self.max_retries})"
        )
        time.sleep(0.2)
        if use_ntp and self.ntp.wait_for_server(timeout_sec=0.5):
            self._send_ntp(new_poses, retry + 1)
        else:
            self._send_fw(new_poses, retry + 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", required=True, help="Ruta al YAML de la ruta")
    ap.add_argument("--skip-first", type=int, default=1, help="Saltarse N waypoints iniciales (def=1)")
    args = ap.parse_args()
    rclpy.init()
    node = Player(args.file, skip_first=args.skip_first)
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
