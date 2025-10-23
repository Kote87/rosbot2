#!/usr/bin/env python3
import argparse
import math
import signal

import rclpy
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from builtin_interfaces.msg import Time as TimeMsg
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue
from rcl_interfaces.msg import ParameterType as PT


def q_from_yaw(yaw: float):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))


class NTPClient(Node):
    def __init__(
        self,
        yaml_file: str,
        frame_id: str = "map",
        premove_mode: str = "auto",
        premove_dist: float = 0.6,
        detect_xy: float = 0.30,
        detect_yaw: float = 0.35,
        force_stateful: bool = True,
        bt_file: str = "",
    ):
        super().__init__("nav_through_poses_client")
        self._shutdown_called = False
        self._ac_ntp = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self._ac_n2p = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._poses = self._load_yaml(yaml_file, frame_id)
        self._premove_mode = premove_mode
        self._premove_dist = premove_dist
        self._detect_xy = detect_xy
        self._detect_yaw = detect_yaw
        self._bt_file = bt_file
        self.get_logger().info(f"Leyendo {yaml_file} – {len(self._poses)} puntos (NTP)")
        if force_stateful:
            self._ensure_stateful_goalchecker()
        self._prepare_closed_loop_if_needed()
        self._send_goal()

    def _load_yaml(self, path, frame_id):
        data = yaml.safe_load(open(path, "r"))
        poses = []
        for w in data.get("waypoints", []):
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = TimeMsg(sec=0, nanosec=0)  # se resella al enviar
            ps.pose.position.x = float(w["x"])
            ps.pose.position.y = float(w["y"])
            q = q_from_yaw(float(w.get("yaw", 0.0)))
            ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            poses.append(ps)
        if not poses:
            raise RuntimeError("El YAML no contiene 'waypoints'.")
        return poses

    def _send_goal(self):
        self.get_logger().info("🚀  Esperando servidor /navigate_through_poses …")
        self._ac_ntp.wait_for_server()
        # Sella timestamps actuales para evitar que downstream considere mensajes viejos
        now = self.get_clock().now().to_msg()
        for ps in self._poses:
            ps.header.stamp = now

        goal = NavigateThroughPoses.Goal()
        goal.poses = self._poses
        # Si nos pasan un BT, úsalo; si no, deja el del bringup
        goal.behavior_tree = self._bt_file if self._bt_file else ""

        self.get_logger().info("🚀  Enviando acción /navigate_through_poses …")
        sfut = self._ac_ntp.send_goal_async(goal, feedback_callback=self._on_feedback)
        sfut.add_done_callback(self._on_goal_response)

    def _on_feedback(self, fb):
        i = fb.feedback.current_pose
        self.get_logger().debug(
            f"➡️  Progreso NTP: x={i.pose.position.x:.2f}, y={i.pose.position.y:.2f}"
        )

    def _on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("⛔  Acción NTP rechazada")
            self._safe_shutdown()
            return
        self.get_logger().info("✅  Acción NTP aceptada")
        rf = gh.get_result_async()
        rf.add_done_callback(
            lambda *_: (
                self.get_logger().info("🏁  NTP terminado"),
                self._safe_shutdown(),
            )
        )

    # ---------- Helpers: parámetros del controller_server ----------
    def _ensure_stateful_goalchecker(self):
        get_cli = self.create_client(GetParameters, "/controller_server/get_parameters")
        set_cli = self.create_client(SetParameters, "/controller_server/set_parameters")
        get_cli.wait_for_service()
        set_cli.wait_for_service()

        req = GetParameters.Request()
        req.names = ["current_goal_checker", "goal_checker.stateful"]
        res_future = get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result()
        if res is None:
            self.get_logger().warn("⚠️  No se pudo consultar parámetros del controller_server")
            return

        cur = res.values[0].string_value if len(res.values) > 0 else ""
        stf = len(res.values) > 1 and res.values[1].bool_value
        patch = []
        if cur != "goal_checker":
            pv = ParameterValue(type=PT.PARAMETER_STRING, string_value="goal_checker")
            patch.append(ParamMsg(name="current_goal_checker", value=pv))
        if not stf:
            pv = ParameterValue(type=PT.PARAMETER_BOOL, bool_value=True)
            patch.append(ParamMsg(name="goal_checker.stateful", value=pv))

        if not patch:
            self.get_logger().info("✅  Goal checker ya stateful")
            return

        set_req = SetParameters.Request()
        set_req.parameters = patch
        set_future = set_cli.call_async(set_req)
        rclpy.spin_until_future_complete(self, set_future)
        result = set_future.result()
        if result is None:
            self.get_logger().warn("⚠️  No se pudo ajustar el goal checker a stateful")
            return
        if all(r.successful for r in result.results):
            self.get_logger().warn(
                "♻️  Ajustado controller_server → current_goal_checker=goal_checker, goal_checker.stateful=true"
            )
        else:
            razones = ", ".join(r.reason for r in result.results if not r.successful)
            self.get_logger().warn(
                f"⚠️  No se pudo ajustar el goal checker a stateful ({razones or 'motivo desconocido'})"
            )

    # ---------- Helpers: rutas cerradas (último==primero) ----------
    @staticmethod
    def _yaw(q: Quaternion):
        s = 2.0 * (q.w * q.z + q.x * q.y)
        c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(s, c)

    @staticmethod
    def _angdiff(a, b):
        return abs((a - b + math.pi) % (2 * math.pi) - math.pi)

    def _prepare_closed_loop_if_needed(self):
        if len(self._poses) < 2:
            return
        first, second, last = self._poses[0], self._poses[1], self._poses[-1]
        dist = math.hypot(
            last.pose.position.x - first.pose.position.x,
            last.pose.position.y - first.pose.position.y,
        )
        dyaw = self._angdiff(self._yaw(last.pose.orientation), self._yaw(first.pose.orientation))
        closed = dist < self._detect_xy and dyaw < self._detect_yaw

        if self._premove_mode == "none":
            return
        if not closed and self._premove_mode == "auto":
            return

        if self._premove_mode in ("auto", "second"):
            self._goto(second)
            self._poses = self._poses[1:]
            self.get_logger().info("🔁 Ruta cerrada: pre‑move al 2º waypoint y reordenación aplicada")
        elif self._premove_mode == "offset":
            self._goto(self._offset_from(first, second, self._premove_dist))
            self._poses = self._poses[1:]

    def _goto(self, target: PoseStamped):
        self.get_logger().info("🚚  Pre‑move (NavigateToPose)…")
        self._ac_n2p.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = target.header.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = target.pose
        sf = self._ac_n2p.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sf)
        gh = sf.result()
        if not gh or not gh.accepted:
            self.get_logger().warn("⚠️  Pre‑move rechazado; continuo con NTP")
            return
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        self.get_logger().info("✅  Pre‑move terminado")

    def _offset_from(self, first: PoseStamped, second: PoseStamped, distance: float) -> PoseStamped:
        fx, fy = first.pose.position.x, first.pose.position.y
        vx = second.pose.position.x - fx
        vy = second.pose.position.y - fy
        norm = math.hypot(vx, vy) or 1.0
        target = PoseStamped()
        target.header.frame_id = first.header.frame_id
        target.pose.position.x = fx + distance * (vx / norm)
        target.pose.position.y = fy + distance * (vy / norm)
        target.pose.orientation = first.pose.orientation
        return target

    def _safe_shutdown(self):
        if self._shutdown_called:
            return
        self._shutdown_called = True
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", required=True)
    ap.add_argument("--frame", default="map")
    ap.add_argument(
        "--pre",
        choices=["auto", "second", "offset", "none"],
        default="auto",
        help="Pre‑move para rutas cerradas (auto=detecta y usa 'second')",
    )
    ap.add_argument("--pre-dist", type=float, default=0.6)
    ap.add_argument("--detect-xy", type=float, default=0.30)
    ap.add_argument("--detect-yaw", type=float, default=0.35)
    ap.add_argument(
        "--bt",
        default="",
        help="Ruta absoluta a un BT .xml (opcional) para NTP",
    )
    ap.add_argument(
        "--force-stateful",
        action="store_true",
        default=True,
        help="Asegura goal_checker.stateful=true en controller_server",
    )
    args = ap.parse_args()
    rclpy.init()
    node = NTPClient(
        args.file,
        args.frame,
        premove_mode=args.pre,
        premove_dist=args.pre_dist,
        detect_xy=args.detect_xy,
        detect_yaw=args.detect_yaw,
        force_stateful=args.force_stateful,
        bt_file=args.bt,
    )
    # Ctrl-C → cancelación limpia sin 'double shutdown'
    signal.signal(signal.SIGINT, lambda *_: node._safe_shutdown())
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        # rclpy.shutdown() ya se invoca en _safe_shutdown()


if __name__ == "__main__":
    main()
