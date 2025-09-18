#!/usr/bin/env python3
"""
Reproduce un recorrido YAML de forma FLUIDA con “salto condicionado”:
 - Arranque: /initialpose alineada al 1er tramo + limpieza de costmaps + micro‑avance recto
 - NO salta waypoints hasta que haya PROGRESO REAL (>= min_progress_to_skip, def 0.08 m)
 - Si ya hubo progreso y sigue bloqueado, puede saltar 1 punto y continuar
 - NTP por defecto; fallback a FollowWaypoints si NTP no existe
Uso:
  python3 /scripts/path_player.py --file /routes/nombre.yaml \
    [--skip-first 0] \
    [--initpose {first,ahead,none}] [--init-bump 0.15] \
    [--allow-skip true] [--min-progress-to-skip 0.08]
"""
import argparse, sys, math, time, yaml, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, FollowWaypoints
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class Player(Node):
    def __init__(
        self,
        yaml_file: str,
        skip_first: int = 0,
        initpose: str = "first",
        init_bump: float = 0.15,
        allow_skip: bool = True,
        min_progress_to_skip: float = 0.08,
        max_retries: int = 4,
    ):
        super().__init__("path_player")
        self.declare_parameter("global_frame", "map")
        self.global_frame = self.get_parameter("global_frame").value
        self.max_retries = max_retries
        self.skip_first = max(0, int(skip_first))
        self.initpose_mode = initpose  # "first" | "ahead" | "none"
        self.init_bump = max(0.0, float(init_bump))
        self.allow_skip = bool(allow_skip)
        self.min_progress_to_skip = max(0.0, float(min_progress_to_skip))

        self.poses = self._load_poses(yaml_file)
        if self.skip_first > 0 and self.skip_first < len(self.poses):
            self.get_logger().info(
                f"⤴️  Saltando primeros {self.skip_first} waypoints (solicitado)"
            )
            self.poses = self.poses[self.skip_first :]
        self.ntp = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self.fw = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self.clear_local = self.create_client(
            Empty, "/local_costmap/clear_entirely_local_costmap"
        )
        self.clear_global = self.create_client(
            Empty, "/global_costmap/clear_entirely_global_costmap"
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        # AMCL para medir progreso por intento
        self._amcl_last = None
        self._amcl_attempt_start = None
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )

        self._prepare_and_run()

    def _load_poses(self, path):
        data = yaml.safe_load(open(path))
        poses = []
        for w in data["waypoints"]:
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame
            ps.header.stamp = TimeMsg(sec=0, nanosec=0)
            ps.pose.position.x = float(w["x"])
            ps.pose.position.y = float(w["y"])
            q = quaternion_from_yaw(float(w["yaw"]))
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            poses.append(ps)
        self.get_logger().info(f"Leyendo {path} – {len(poses)} puntos")
        return poses

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self._amcl_last = (float(p.x), float(p.y))

    def _progress_since_attempt(self) -> float:
        if self._amcl_last is None or self._amcl_attempt_start is None:
            return 0.0
        dx = self._amcl_last[0] - self._amcl_attempt_start[0]
        dy = self._amcl_last[1] - self._amcl_attempt_start[1]
        return math.hypot(dx, dy)

    def _clear_costmaps(self):
        for cli in (self.clear_local, self.clear_global):
            if cli.wait_for_service(timeout_sec=1.0):
                try:
                    cli.call_async(Empty.Request())
                except Exception:
                    pass
        self.get_logger().info("🧹 Costmaps limpiados")

    def _publish_initialpose(self, mode: str, bump: float):
        if not self.poses:
            return
        p0 = self.poses[0].pose.position
        yaw = self._yaw_first_segment()
        x, y = float(p0.x), float(p0.y)
        if mode == "ahead" and len(self.poses) >= 2 and bump > 0.0:
            x += math.cos(yaw) * bump
            y += math.sin(yaw) * bump
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = self.global_frame
        ip.pose.pose.position.x = x
        ip.pose.pose.position.y = y
        ip.pose.pose.orientation.z = qz
        ip.pose.pose.orientation.w = qw
        ip.pose.covariance[0] = 0.05**2
        ip.pose.covariance[7] = 0.05**2
        ip.pose.covariance[35] = math.radians(5) ** 2
        pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        for _ in range(3):
            pub.publish(ip)
            time.sleep(0.1)
        self.get_logger().info(
            f"📍 /initialpose → ({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}°  mode={mode}, bump={bump:.2f}m"
        )

    def _yaw_first_segment(self) -> float:
        if len(self.poses) >= 2:
            a = self.poses[0].pose.position
            b = self.poses[1].pose.position
            return math.atan2(float(b.y) - float(a.y), float(b.x) - float(a.x))
        q = self.poses[0].pose.orientation
        return 2.0 * math.atan2(float(q.z), float(q.w))

    def _nudge_forward(self, v=0.10, duration=0.6):
        end = time.time() + float(duration)
        tw = Twist()
        tw.linear.x = float(v)
        self.get_logger().info(f"👣 Nudge forward v={v} dur={duration}s")
        while time.time() < end:
            self.cmd_pub.publish(tw)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())

    def _prepare_and_run(self):
        # Inicializa pose alineada y limpia
        if self.initpose_mode in ("first", "ahead"):
            self._publish_initialpose(
                self.initpose_mode,
                self.init_bump if self.initpose_mode == "ahead" else 0.0,
            )
        self._clear_costmaps()
        time.sleep(0.2)
        # Marca inicio de intento para medir progreso
        self._amcl_attempt_start = self._amcl_last
        if self.ntp.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("🚀  /navigate_through_poses")
            self._send_ntp(self.poses, retry=0, phase="start")
        elif self.fw.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("ℹ️  NTP no disponible, usando /follow_waypoints")
            self._send_fw(self.poses, retry=0, phase="start")
        else:
            self.get_logger().error("⛔  No hay servidores NTP ni FW disponibles")
            rclpy.shutdown()

    # ---------- NTP -----------
    def _send_ntp(self, poses, retry, phase="run"):
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        fut = self.ntp.send_goal_async(goal)
        fut.add_done_callback(
            lambda f: self._on_ntp_accepted(f, poses, retry, phase)
        )

    def _on_ntp_accepted(self, future, poses, retry, phase):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("⛔  NTP goal rechazado")
            return self._maybe_retry_or_fallback(poses, retry, phase)
        self.get_logger().info("✅  NTP goal aceptado")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(
            lambda fr: self._on_ntp_result(fr, poses, retry, phase)
        )

    def _on_ntp_result(self, future, poses, retry, phase):
        res = future.result()
        status = getattr(res, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁  Recorrido terminado (NTP SUCCESS)")
            rclpy.shutdown()
            return
        self.get_logger().warn(f"⚠️  NTP terminó con estado {status}.")
        self._handle_failure(poses, retry, prefer_ntp=True, phase=phase)

    # ---------- FW ------------
    def _send_fw(self, poses, retry, phase="run"):
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        fut = self.fw.send_goal_async(goal)
        fut.add_done_callback(
            lambda f: self._on_fw_accepted(f, poses, retry, phase)
        )

    def _on_fw_accepted(self, future, poses, retry, phase):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("⛔  FW goal rechazado")
            return self._maybe_retry_or_fallback(poses, retry, phase, already_fw=True)
        self.get_logger().info("✅  FW goal aceptado")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(
            lambda fr: self._on_fw_result(fr, poses, retry, phase)
        )

    def _on_fw_result(self, future, poses, retry, phase):
        res = future.result()
        status = getattr(res, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁  Recorrido terminado (FW SUCCESS)")
            rclpy.shutdown()
            return
        self.get_logger().warn(f"⚠️  FW terminó con estado {status}.")
        self._handle_failure(poses, retry, prefer_ntp=False, phase=phase)

    # ---------- Helpers -------
    def _maybe_retry_or_fallback(self, poses, retry, phase, already_fw=False):
        if not already_fw and self.fw.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("↩️  Fallback a /follow_waypoints")
            return self._send_fw(poses, retry, phase)
        if self.ntp.wait_for_server(timeout_sec=0.5):
            return self._send_ntp(poses, retry, phase)
        self.get_logger().error("⛔  No hay servidores NTP/FW")
        rclpy.shutdown()

    def _handle_failure(self, poses, retry, prefer_ntp: bool, phase: str):
        moved = self._progress_since_attempt()
        self.get_logger().info(
            f"📏 Progreso desde inicio del intento: {moved:.3f} m"
        )
        self._clear_costmaps()
        # 1) Si NO hay progreso real, reintenta MISMO set sin saltar: re-alinear + nudge
        if moved < self.min_progress_to_skip and retry < self.max_retries:
            if self.initpose_mode != "none":
                self._publish_initialpose("ahead", max(self.init_bump, 0.15))
            time.sleep(0.2)
            self._nudge_forward(v=0.10, duration=0.6)
            time.sleep(0.2)
            # reinicia contador de progreso por intento
            self._amcl_attempt_start = self._amcl_last
            if prefer_ntp and self.ntp.wait_for_server(timeout_sec=0.5):
                return self._send_ntp(poses, retry + 1, phase)
            return self._send_fw(poses, retry + 1, phase)
        # 2) Si YA hubo progreso suficiente:
        if not self.allow_skip:
            self.get_logger().info(
                "🔁 allow-skip=false → reintento MISMOS puntos (sin saltar)"
            )
            if retry < self.max_retries:
                if prefer_ntp and self.ntp.wait_for_server(timeout_sec=0.5):
                    return self._send_ntp(poses, retry + 1, phase)
                return self._send_fw(poses, retry + 1, phase)
            self.get_logger().error("⛔  Reintentos agotados. Terminando.")
            rclpy.shutdown()
            return
        # 3) Aquí sí: saltar 1 waypoint y seguir
        if retry >= self.max_retries or len(poses) <= 1:
            self.get_logger().error("⛔  Reintentos agotados o sin más puntos. Terminando.")
            rclpy.shutdown()
            return
        new_poses = poses[1:]
        self.get_logger().info(
            f"⤴️  (allow-skip) Saltando 1 waypoint (quedan {len(new_poses)}) y reintentando… ({retry + 1}/{self.max_retries})"
        )
        time.sleep(0.2)
        # al saltar, resetea base de progreso del nuevo intento
        self._amcl_attempt_start = self._amcl_last
        if prefer_ntp and self.ntp.wait_for_server(timeout_sec=0.5):
            self._send_ntp(new_poses, retry + 1, phase)
        else:
            self._send_fw(new_poses, retry + 1, phase)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", required=True, help="Ruta al YAML de la ruta")
    ap.add_argument(
        "--skip-first", type=int, default=0, help="Saltarse N waypoints iniciales (def=0)"
    )
    ap.add_argument(
        "--initpose",
        choices=["first", "ahead", "none"],
        default="first",
        help="Publicar /initialpose alineada al inicio",
    )
    ap.add_argument(
        "--init-bump",
        type=float,
        default=0.15,
        help="Desplazar /initialpose hacia delante (m) si initpose=ahead",
    )
    ap.add_argument(
        "--allow-skip",
        type=str,
        default="true",
        help="Permitir saltar waypoints tras progreso real (def=true)",
    )
    ap.add_argument(
        "--min-progress-to-skip",
        type=float,
        default=0.08,
        help="Progreso mínimo (m) para permitir saltos",
    )
    args = ap.parse_args()
    allow_skip = str(args.allow_skip).lower() in ("1", "true", "yes", "y")
    rclpy.init()
    node = Player(
        args.file,
        skip_first=args.skip_first,
        initpose=args.initpose,
        init_bump=args.init_bump,
        allow_skip=allow_skip,
        min_progress_to_skip=args.min_progress_to_skip,
    )
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
