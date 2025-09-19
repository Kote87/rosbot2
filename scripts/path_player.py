#!/usr/bin/env python3
"""
Reproduce un recorrido YAML de forma FLUIDA con “salto condicionado” + DIAGNÓSTICO.
 - Arranque: ALINEA opcionalmente la ruta al AMCL (no publica /initialpose salvo que lo pidas)
 - NO salta waypoints hasta que haya PROGRESO REAL (>= min_progress_to_skip, def 0.08 m)
 - Si ya hubo progreso y sigue bloqueado, puede saltar 1 punto y continuar
 - NTP por defecto; fallback a FollowWaypoints si NTP no existe
 - DIAG: lifecycle, plugins, coste en celda del robot (global/local), stats del láser, endpoints /cmd_vel*, distancias a WPs
Uso:
  python3 /scripts/path_player.py --file /routes/nombre.yaml     [--skip-first 0] [--align-to-current true]     [--initpose {none,first,ahead}] [--init-bump 0.15] [--nudge false]     [--allow-skip true] [--min-progress-to-skip 0.08]
"""
import argparse, sys, math, time, yaml, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, FollowWaypoints
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty
from math import atan2, sin, cos, pi
from typing import List, Optional, Tuple
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import GetCostmap
from sensor_msgs.msg import LaserScan

def quaternion_from_yaw(yaw): half=yaw*0.5; return (0.0, 0.0, math.sin(half), math.cos(half))

class Player(Node):
    def __init__(self, yaml_file: str, skip_first: int = 0,
                 align_to_current: bool = True, initpose: str = "none",
                 init_bump: float = 0.15, nudge: bool = False,
                 allow_skip: bool = True, min_progress_to_skip: float = 0.08,
                 max_retries: int = 4):
        super().__init__("path_player")
        self.get_logger().info("🧩 path_player starting (diag=ON)")
        self.declare_parameter("global_frame", "map")
        self.global_frame = self.get_parameter("global_frame").value
        self.max_retries = max_retries
        self.skip_first = max(0, int(skip_first))
        self.align_to_current = bool(align_to_current)
        self.initpose_mode = initpose  # "none" | "first" | "ahead"
        self.init_bump = max(0.0, float(init_bump))
        self.nudge = bool(nudge)
        self.allow_skip = bool(allow_skip)
        self.min_progress_to_skip = max(0.0, float(min_progress_to_skip))
        self.diag = True
        self._diag_once_dumped = False

        self.poses = self._load_poses(yaml_file)
        if self.skip_first > 0 and self.skip_first < len(self.poses):
            self.get_logger().info(f"⤴️  Saltando primeros {self.skip_first} waypoints (solicitado)")
            self.poses = self.poses[self.skip_first:]
        self.ntp = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self.fw  = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self.clear_local  = self.create_client(Empty, "/local_costmap/clear_entirely_local_costmap")
        self.clear_global = self.create_client(Empty, "/global_costmap/clear_entirely_global_costmap")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self._amcl_ready = False
        self._amcl_last = None
        self._amcl_last_yaw = None
        self._amcl_attempt_start = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)

        # DIAG Láser
        self._scan_raw_stats = None
        self._scan_filt_stats = None
        self.create_subscription(LaserScan, "/scan", self._on_scan_raw, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self._on_scan_filt, 10)

        # DIAG costmaps
        self._svc_candidates_global = ["/global_costmap/global_costmap/get_costmap", "/global_costmap/get_costmap"]
        self._svc_candidates_local  = ["/local_costmap/local_costmap/get_costmap",  "/local_costmap/get_costmap"]

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
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = 2.0 * atan2(float(q.z), float(q.w))
        self._amcl_last = (float(p.x), float(p.y))
        self._amcl_last_yaw = float(yaw)
        cov_yaw = msg.pose.covariance[35]
        if cov_yaw > 0.0 and cov_yaw < (math.radians(15) ** 2):
            self._amcl_ready = True

    # Láser stats
    def _scan_stats_from_msg(self, msg: LaserScan) -> dict:
        import math as _m
        vals = [v for v in msg.ranges if _m.isfinite(v)]
        if not vals: return {"count": 0}
        n = len(vals); mn=min(vals); mx=max(vals)
        under_0_20 = sum(1 for v in vals if v < 0.20)
        under_0_30 = sum(1 for v in vals if v < 0.30)
        return {"count": n, "min": round(mn,3), "max": round(mx,3),
                "%<0.20": round(100.0*under_0_20/n,1), "%<0.30": round(100.0*under_0_30/n,1),
                "angle_min": round(msg.angle_min,3), "angle_max": round(msg.angle_max,3)}
    def _on_scan_raw(self, msg: LaserScan):  self._scan_raw_stats  = self._scan_stats_from_msg(msg)
    def _on_scan_filt(self, msg: LaserScan): self._scan_filt_stats = self._scan_stats_from_msg(msg)

    def _progress_since_attempt(self) -> float:
        if self._amcl_last is None or self._amcl_attempt_start is None: return 0.0
        dx = self._amcl_last[0] - self._amcl_attempt_start[0]
        dy = self._amcl_last[1] - self._amcl_attempt_start[1]
        return math.hypot(dx, dy)

    def _clear_costmaps(self):
        for cli in (self.clear_local, self.clear_global):
            if cli.wait_for_service(timeout_sec=1.0):
                try: cli.call_async(Empty.Request())
                except Exception: pass
        self.get_logger().info("🧹 Costmaps limpiados")

    def _publish_initialpose(self, mode: str, bump: float):
        if not self.poses: return
        p0 = self.poses[0].pose.position
        yaw = self._yaw_first_segment()
        x, y = float(p0.x), float(p0.y)
        if mode == "ahead" and len(self.poses) >= 2 and bump > 0.0:
            x += cos(yaw) * bump; y += sin(yaw) * bump
        qz, qw = sin(yaw / 2.0), cos(yaw / 2.0)
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = self.global_frame
        ip.pose.pose.position.x = x; ip.pose.pose.position.y = y
        ip.pose.pose.orientation.z = qz; ip.pose.pose.orientation.w = qw
        ip.pose.covariance[0] = 0.05**2; ip.pose.covariance[7] = 0.05**2
        ip.pose.covariance[35] = (5.0 * pi / 180.0) ** 2
        pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        for _ in range(3): pub.publish(ip); time.sleep(0.1)
        self.get_logger().info(f"📍 /initialpose → ({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}°  mode={mode}, bump={bump:.2f}m")

    def _yaw_first_segment(self) -> float:
        if len(self.poses) >= 2:
            a, b = self.poses[0].pose.position, self.poses[1].pose.position
            return atan2(float(b.y)-float(a.y), float(b.x)-float(a.x))
        q = self.poses[0].pose.orientation
        return 2.0*atan2(float(q.z), float(q.w))

    def _nudge_forward(self, v=0.10, duration=0.6):
        end = time.time() + float(duration); tw = Twist(); tw.linear.x = float(v)
        self.get_logger().info(f"👣 Nudge forward v={v} dur={duration}s")
        while time.time() < end: self.cmd_pub.publish(tw); time.sleep(0.05)
        self.cmd_pub.publish(Twist())

    def _align_route_to_current(self):
        """Alinea ruta al AMCL (traslación + rotación)."""
        if not self.poses: return
        t0 = time.time()
        while rclpy.ok() and (self._amcl_last is None or self._amcl_last_yaw is None) and time.time()-t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._amcl_last is None or self._amcl_last_yaw is None:
            self.get_logger().warn("AMCL no disponible aún; NO se alinea ruta"); return
        r0 = self.poses[0]
        rx0, ry0 = float(r0.pose.position.x), float(r0.pose.position.y)
        rqz, rqw = float(r0.pose.orientation.z), float(r0.pose.orientation.w)
        ryaw = 2.0*atan2(rqz, rqw)
        cx0, cy0 = self._amcl_last; cyaw = self._amcl_last_yaw
        dth = cyaw - ryaw; c, s = cos(dth), sin(dth)
        for ps in self.poses:
            dx = float(ps.pose.position.x) - rx0; dy = float(ps.pose.position.y) - ry0
            x = cx0 + (dx*c - dy*s); y = cy0 + (dx*s + dy*c)
            qz, qw = float(ps.pose.orientation.z), float(ps.pose.orientation.w)
            yaw = 2.0*atan2(qz, qw) + dth; q = quaternion_from_yaw(yaw)
            ps.pose.position.x = x; ps.pose.position.y = y
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        self.get_logger().info(f"🧭 Ruta alineada a AMCL: Δx={cx0-rx0:+.2f} Δy={cy0-ry0:+.2f} Δθ={math.degrees(dth):+.1f}°")

    # ---------- DIAGNÓSTICO PROFUNDO ----------
    def _log_topic_endpoints(self, topic: str):
        pubs = self.get_publishers_info_by_topic(topic)
        subs = self.get_subscriptions_info_by_topic(topic)
        self.get_logger().info(f"🔌 {topic}: pubs={[p.node_name for p in pubs]} subs={[s.node_name for s in subs]}")

    def _get_remote_params(self, node: str, names: List[str]) -> Optional[dict]:
        node = node if node.startswith("/") else f"/{node}"
        cli = self.create_client(GetParameters, f"{node}/get_parameters")
        if not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"❔ get_parameters no disponible en {node}"); return None
        req = GetParameters.Request(names=names)
        fut = cli.call_async(req); rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.result(): self.get_logger().warn(f"❌ fallo get_parameters en {node}"); return None
        vals = []
        for v in fut.result().values:
            if   v.type == v.TYPE_STRING:        vals.append(v.string_value)
            elif v.type == v.TYPE_DOUBLE:        vals.append(v.double_value)
            elif v.type == v.TYPE_INTEGER:       vals.append(v.integer_value)
            elif v.type == v.TYPE_BOOL:          vals.append(v.bool_value)
            elif v.type == v.TYPE_BYTE_ARRAY:    vals.append(list(v.byte_array_value))
            elif v.type == v.TYPE_STRING_ARRAY:  vals.append(list(v.string_array_value))
            else:                                vals.append(None)
        return dict(zip(names, vals))

    def _get_lifecycle(self, node: str) -> str:
        node = node if node.startswith("/") else f"/{node}"
        cli = self.create_client(GetState, f"{node}/get_state")
        if not cli.wait_for_service(timeout_sec=1.0): return "no-service"
        req = GetState.Request(); fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.result(): return "error"
        return fut.result().current_state.label

    def _try_get_costmap(self, candidates: List[str]):
        for svc in candidates:
            cli = self.create_client(GetCostmap, svc)
            if cli.wait_for_service(timeout_sec=0.5):
                fut = cli.call_async(GetCostmap.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
                if fut.result(): return fut.result().map
        return None

    def _cost_at_robot(self, which: str):
        if self._amcl_last is None:
            self.get_logger().warn("AMCL aún sin pose; no puedo muestrear costmap"); return
        cm = self._try_get_costmap(self._svc_candidates_global if which=="global" else self._svc_candidates_local)
        if not cm:
            self.get_logger().warn(f"⚠️  {which} costmap no expone get_costmap"); return
        ox, oy, res, sx, sy, grid = cm.metadata.origin.position.x, cm.metadata.origin.position.y, cm.metadata.resolution, cm.metadata.size_x, cm.metadata.size_y, list(cm.data)
        x, y = self._amcl_last; cx = int((x-ox)/res); cy = int((y-oy)/res)
        if cx<0 or cy<0 or cx>=sx or cy>=sy:
            self.get_logger().warn(f"⚠️  {which} costmap: robot fuera del grid (idx=({cx},{cy}), size=({sx},{sy}))"); return
        W=7; half=W//2; vals=[]
        for j in range(cy-half, cy+half+1):
            if 0<=j<sy:
                for i in range(cx-half, cx+half+1):
                    if 0<=i<sx: vals.append(grid[j*sx+i])
        if not vals: self.get_logger().info(f"📊 {which} costmap: sin datos en ventana"); return
        lethal=sum(1 for v in vals if v>=253); unknown=sum(1 for v in vals if v==255); avg=sum(vals)/len(vals); cen=grid[cy*sx+cx]
        self.get_logger().info(f"📊 {which} costmap @robot: center={cen} avg={avg:.1f} lethal%={100.0*lethal/len(vals):.1f} unknown%={100.0*unknown/len(vals):.1f}")

    def _diag_snapshot(self, when: str):
        if not self.diag: return
        self.get_logger().info(f"===== 🔎 DIAG SNAPSHOT ({when}) =====")
        for n in ["bt_navigator","planner_server","controller_server","behavior_server","global_costmap/global_costmap","local_costmap/local_costmap","amcl"]:
            st = self._get_lifecycle(n)
            self.get_logger().info(f"⚙️  {n}: {st}")
        gp = self._get_remote_params("global_costmap/global_costmap", ["plugins","inflation_layer.inflation_radius"])
        lp = self._get_remote_params("local_costmap/local_costmap", ["plugins","inflation_layer.inflation_radius"])
        pp = self._get_remote_params("planner_server", ["GridBased.allow_unknown","GridBased.tolerance"])
        if gp: self.get_logger().info(f"🧩 global plugins = {gp}")
        if lp: self.get_logger().info(f"🧩 local  plugins = {lp}")
        if pp: self.get_logger().info(f"🧩 planner flags = {pp}")
        self._cost_at_robot("global")
        self._cost_at_robot("local")
        for t in ["/cmd_vel","/cmd_vel_unstamped","/scan","/scan_filtered","/amcl_pose","/map"]:
            self._log_topic_endpoints(t)
        if self._scan_raw_stats:  self.get_logger().info(f"📡 /scan stats: {self._scan_raw_stats}")
        if self._scan_filt_stats: self.get_logger().info(f"📡 /scan_filtered stats: {self._scan_filt_stats}")
        if self._amcl_last and self.poses:
            ax, ay = self._amcl_last
            for k in range(min(3, len(self.poses))):
                p = self.poses[k].pose.position; d = math.hypot(float(p.x)-ax, float(p.y)-ay)
                self.get_logger().info(f"🎯 WP[{k}] dist={d:.2f} m")
        self.get_logger().info("===== 🔎 DIAG END =====")

    def _prepare_and_run(self):
        if self.align_to_current: self._align_route_to_current()
        if not self._diag_once_dumped:
            self._diag_snapshot("pre-start"); self._diag_once_dumped=True
        if self.initpose_mode in ("first", "ahead"):
            self._publish_initialpose(self.initpose_mode, self.init_bump if self.initpose_mode=="ahead" else 0.0)
        t0 = time.time()
        while not self._amcl_ready and time.time()-t0 < 8.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self._amcl_ready: self.get_logger().warn("AMCL no listo tras 8s; continúo igualmente")
        self._clear_costmaps(); time.sleep(0.2)
        self._amcl_attempt_start = self._amcl_last
        if self.ntp.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("🚀  /navigate_through_poses"); self._send_ntp(self.poses, retry=0, phase="start")
        elif self.fw.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("ℹ️  NTP no disponible, usando /follow_waypoints"); self._send_fw(self.poses, retry=0, phase="start")
        else:
            self.get_logger().error("⛔  No hay servidores NTP/FW"); rclpy.shutdown()

    # ---------- NTP ----------
    def _send_ntp(self, poses, retry, phase="run"):
        fut = self.ntp.send_goal_async(NavigateThroughPoses.Goal(poses=poses))
        fut.add_done_callback(lambda f: self._on_ntp_accepted(f, poses, retry, phase))
    def _on_ntp_accepted(self, future, poses, retry, phase):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("⛔  NTP goal rechazado"); return self._maybe_retry_or_fallback(poses, retry, phase)
        self.get_logger().info("✅  NTP goal aceptado")
        handle.get_result_async().add_done_callback(lambda fr: self._on_ntp_result(fr, poses, retry, phase))
    def _on_ntp_result(self, future, poses, retry, phase):
        res = future.result(); status = getattr(res, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁  Recorrido terminado (NTP SUCCESS)"); rclpy.shutdown(); return
        self.get_logger().warn(f"⚠️  NTP terminó con estado {status}."); self._diag_snapshot(f"ntp-failed@retry{retry}")
        self._handle_failure(poses, retry, prefer_ntp=True, phase=phase)

    # ---------- FW ----------
    def _send_fw(self, poses, retry, phase="run"):
        fut = self.fw.send_goal_async(FollowWaypoints.Goal(poses=poses))
        fut.add_done_callback(lambda f: self._on_fw_accepted(f, poses, retry, phase))
    def _on_fw_accepted(self, future, poses, retry, phase):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("⛔  FW goal rechazado"); return self._maybe_retry_or_fallback(poses, retry, phase, already_fw=True)
        self.get_logger().info("✅  FW goal aceptado")
        handle.get_result_async().add_done_callback(lambda fr: self._on_fw_result(fr, poses, retry, phase))
    def _on_fw_result(self, future, poses, retry, phase):
        res = future.result(); status = getattr(res, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁  Recorrido terminado (FW SUCCESS)"); rclpy.shutdown(); return
        self.get_logger().warn(f"⚠️  FW terminó con estado {status}."); self._diag_snapshot(f"fw-failed@retry{retry}")
        self._handle_failure(poses, retry, prefer_ntp=False, phase=phase)

    # ---------- Helpers ----------
    def _maybe_retry_or_fallback(self, poses, retry, phase, already_fw=False):
        if not already_fw and self.fw.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("↩️  Fallback a /follow_waypoints"); return self._send_fw(poses, retry, phase)
        if self.ntp.wait_for_server(timeout_sec=0.5): return self._send_ntp(poses, retry, phase)
        self.get_logger().error("⛔  No hay servidores NTP/FW"); rclpy.shutdown()

    def _handle_failure(self, poses, retry, prefer_ntp: bool, phase: str):
        moved = self._progress_since_attempt()
        self.get_logger().info(f"📏 Progreso desde inicio del intento: {moved:.3f} m"); self._clear_costmaps()
        if moved < self.min_progress_to_skip and retry < self.max_retries:
            if self.align_to_current: self._align_route_to_current()
            if self.initpose_mode != "none": self._publish_initialpose("ahead", max(self.init_bump, 0.15))
            time.sleep(0.2)
            if self.nudge: self._nudge_forward(v=0.10, duration=0.6)
            time.sleep(0.2); self._amcl_attempt_start = self._amcl_last
            if prefer_ntp and self.ntp.wait_for_server(timeout_sec=0.5): return self._send_ntp(poses, retry+1, phase)
            return self._send_fw(poses, retry+1, phase)
        if not self.allow_skip:
            self.get_logger().info("🔁 allow-skip=false → reintento MISMOS puntos (sin saltar)")
            if retry < self.max_retries:
                if prefer_ntp and self.ntp.wait_for_server(timeout_sec=0.5): return self._send_ntp(poses, retry+1, phase)
                return self._send_fw(poses, retry+1, phase)
            self.get_logger().error("⛔  Reintentos agotados. Terminando."); rclpy.shutdown(); return
        if retry >= self.max_retries or len(poses) <= 1:
            self.get_logger().error("⛔  Reintentos agotados o sin más puntos. Terminando."); rclpy.shutdown(); return
        new_poses = poses[1:]; self.get_logger().info(f"⤴️  (allow-skip) Saltando 1 waypoint (quedan {len(new_poses)}) y reintentando… ({retry+1}/{self.max_retries})")
        time.sleep(0.2); self._amcl_attempt_start = self._amcl_last
        if prefer_ntp and self.ntp.wait_for_server(timeout_sec=0.5): self._send_ntp(new_poses, retry+1, phase)
        else: self._send_fw(new_poses, retry+1, phase)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", required=True, help="Ruta al YAML de la ruta")
    ap.add_argument("--skip-first", type=int, default=0, help="Saltarse N waypoints iniciales (def=0)")
    ap.add_argument("--align-to-current", type=str, default="true", help="Alinear la ruta al AMCL actual (def=true)")
    ap.add_argument("--initpose", choices=["none","first","ahead"], default="none", help="Publicar /initialpose (def=none)")
    ap.add_argument("--init-bump", type=float, default=0.15, help="Desplazar /initialpose hacia delante (m) si initpose=ahead")
    ap.add_argument("--nudge", type=str, default="false", help="Hacer micro-avance /cmd_vel si no hay progreso (def=false)")
    ap.add_argument("--allow-skip", type=str, default="true", help="Permitir saltar waypoints tras progreso real (def=true)")
    ap.add_argument("--min-progress-to-skip", type=float, default=0.08, help="Progreso mínimo (m) para permitir saltos")
    args = ap.parse_args()
    allow_skip = str(args.allow_skip).lower() in ("1","true","yes","y")
    align_to_current = str(args.align_to_current).lower() in ("1","true","yes","y")
    nudge = str(args.nudge).lower() in ("1","true","yes","y")
    rclpy.init()
    node = Player(args.file, skip_first=args.skip_first, align_to_current=align_to_current,
                  initpose=args.initpose, init_bump=args.init_bump, nudge=nudge,
                  allow_skip=allow_skip, min_progress_to_skip=args.min_progress_to_skip)
    try: rclpy.spin(node)
    finally:
        try: node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__": main()
