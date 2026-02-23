#!/usr/bin/env python3
import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

try:
    from nav2_msgs.msg import Costmap as Nav2Costmap
except Exception:
    Nav2Costmap = None  # type: ignore

from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters

import tf2_ros


LIFECYCLE_STATE = {
    0: "unknown",
    1: "unconfigured",
    2: "inactive",
    3: "active",
    4: "finalized",
}


@dataclass
class CheckResult:
    ok: bool
    name: str
    details: str


def _now() -> str:
    return time.strftime("%H:%M:%S")


class Nav2Preflight(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("nav2_preflight")
        self.args = args

        # TF buffer + listener (hilo propio para recibir /tf sin depender de spin principal)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

    # ---------------- Graph helpers ----------------

    def topic_types(self) -> Dict[str, List[str]]:
        return {name: types for name, types in self.get_topic_names_and_types()}

    def publishers_of(self, topic: str) -> List[str]:
        infos = self.get_publishers_info_by_topic(topic)
        return [f"{i.node_namespace}/{i.node_name}".replace("//", "/") for i in infos]

    def subscribers_of(self, topic: str) -> List[str]:
        infos = self.get_subscriptions_info_by_topic(topic)
        return [f"{i.node_namespace}/{i.node_name}".replace("//", "/") for i in infos]

    def wait_for_publishers(self, topic: str, min_count: int, timeout_s: float) -> Tuple[bool, List[str]]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            pubs = self.publishers_of(topic)
            if len(pubs) >= min_count:
                return True, pubs
            time.sleep(0.2)
        return False, self.publishers_of(topic)

    # ---------------- TF helpers ----------------

    def wait_for_tf(self, target: str, source: str, timeout_s: float) -> bool:
        """Equivalente a: tf2_echo <target> <source> (existencia de transform)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                if self.tf_buffer.can_transform(
                    target, source, Time(), timeout=Duration(seconds=0.05)
                ):
                    return True
            except Exception:
                pass
            time.sleep(0.1)
        return False

    # ---------------- Lifecycle helpers ----------------

    def lifecycle_state(self, node_name: str, timeout_s: float = 1.0) -> Optional[str]:
        srv = f"/{node_name}/get_state"
        client = self.create_client(GetState, srv)
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        req = GetState.Request()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        if not fut.done() or fut.result() is None:
            return None
        state_id = fut.result().current_state.id
        return LIFECYCLE_STATE.get(state_id, f"state_{state_id}")

    def get_param(self, node_name: str, param_name: str, timeout_s: float = 1.0) -> Optional[str]:
        srv = f"/{node_name}/get_parameters"
        client = self.create_client(GetParameters, srv)
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        req = GetParameters.Request()
        req.names = [param_name]
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
        if not fut.done() or fut.result() is None or not fut.result().values:
            return None
        v = fut.result().values[0]
        # stringify “best effort”
        if v.type == 1:   # BOOL
            return str(v.bool_value)
        if v.type == 2:   # INTEGER
            return str(v.integer_value)
        if v.type == 3:   # DOUBLE
            return str(v.double_value)
        if v.type == 4:   # STRING
            return v.string_value
        return "<non-scalar>"

    # ---------------- Rate probe ----------------

    def measure_rate(self, topic: str, msg_type, window_s: float) -> Tuple[float, int]:
        count = 0

        def cb(_msg):
            nonlocal count
            count += 1

        sub = self.create_subscription(msg_type, topic, cb, 10)
        t0 = time.monotonic()
        while time.monotonic() - t0 < window_s:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)

        rate = count / max(window_s, 1e-9)
        return rate, count

    def resolve_costmap_msg_type(self, topic: str):
        t = self.topic_types().get(topic, [])
        if "nav2_msgs/msg/Costmap" in t and Nav2Costmap is not None:
            return Nav2Costmap
        if "nav_msgs/msg/OccupancyGrid" in t:
            return OccupancyGrid
        return None

    # ---------------- Main run ----------------

    def run(self) -> int:
        checks: List[CheckResult] = []

        # 0) Lectura (si se puede) de odom_topic configurado en bt_navigator
        odom_topic = self.get_param("bt_navigator", "odom_topic") or self.get_param("bt_navigator", "odom_topic")
        if odom_topic:
            checks.append(CheckResult(True, "param bt_navigator.odom_topic", odom_topic))
        else:
            checks.append(CheckResult(True, "param bt_navigator.odom_topic", "no disponible (no bloqueante)"))

        # 1) Publishers mínimos
        for topic, min_count in [
            (self.args.scan_topic, 1),
            (self.args.tf_topic, 1),
        ]:
            ok, pubs = self.wait_for_publishers(topic, min_count=min_count, timeout_s=self.args.timeout)
            checks.append(CheckResult(
                ok, f"publishers {topic}",
                f"{len(pubs)} -> {pubs}" if pubs else "0 (NINGUNO)"
            ))

        # Odom/IMU (no siempre se llaman igual en todos los setups; aquí probamos varias opciones)
        odom_candidates = ["/odometry/filtered", "/localization/odometry/filtered", "/rosbot_xl_base_controller/odom"]
        imu_candidates = ["/imu", "/imu_broadcaster/imu"]

        odom_ok = False
        for t in odom_candidates:
            ok, pubs = self.wait_for_publishers(t, 1, timeout_s=2.0)
            if ok:
                checks.append(CheckResult(True, f"publishers {t}", f"{len(pubs)} -> {pubs}"))
                odom_ok = True
                break
        if not odom_ok:
            checks.append(CheckResult(False, "publishers odom (candidatos)", f"probados: {odom_candidates}"))

        imu_ok = False
        for t in imu_candidates:
            ok, pubs = self.wait_for_publishers(t, 1, timeout_s=2.0)
            if ok:
                checks.append(CheckResult(True, f"publishers {t}", f"{len(pubs)} -> {pubs}"))
                imu_ok = True
                break
        if not imu_ok:
            checks.append(CheckResult(False, "publishers imu (candidatos)", f"probados: {imu_candidates}"))

        # 2) TF crítico: odom<->base_link y map<->odom
        tf_odom_bl = self.wait_for_tf("base_link", "odom", timeout_s=self.args.timeout)
        checks.append(CheckResult(tf_odom_bl, "TF base_link <- odom", "OK" if tf_odom_bl else "NO existe / timeout"))

        tf_map_odom = self.wait_for_tf("map", "odom", timeout_s=self.args.timeout)
        checks.append(CheckResult(tf_map_odom, "TF map <- odom", "OK" if tf_map_odom else "NO existe / timeout"))

        # 3) Lifecycle (si hay servicio; si no hay, lo anotamos)
        required_lifecycle = ["controller_server", "local_costmap", "global_costmap", "planner_server", "bt_navigator"]
        optional_lifecycle = ["amcl", "map_server", "slam_toolbox"]

        for n in required_lifecycle:
            st = self.lifecycle_state(n, timeout_s=1.0)
            if st is None:
                checks.append(CheckResult(False, f"lifecycle {n}", "sin /get_state (¿no arrancó?)"))
            else:
                checks.append(CheckResult(st == "active", f"lifecycle {n}", st))

        for n in optional_lifecycle:
            st = self.lifecycle_state(n, timeout_s=0.5)
            if st is None:
                checks.append(CheckResult(True, f"lifecycle {n}", "no presente (OK)"))
            else:
                checks.append(CheckResult(True, f"lifecycle {n}", st))

        # 4) Scan rate + subs
        ttypes = self.topic_types()
        scan_types = ttypes.get(self.args.scan_topic, [])
        if "sensor_msgs/msg/LaserScan" not in scan_types:
            checks.append(CheckResult(False, "scan type", f"{self.args.scan_topic} types={scan_types}"))
        else:
            rate, count = self.measure_rate(self.args.scan_topic, LaserScan, window_s=self.args.rate_window)
            subs = self.subscribers_of(self.args.scan_topic)
            ok = rate >= self.args.min_scan_hz
            checks.append(CheckResult(
                ok, f"scan rate {self.args.scan_topic}",
                f"{rate:.2f} Hz ({count} msgs/{self.args.rate_window:.1f}s), subs={subs}"
            ))

        # 5) Costmaps publishing rate
        for cm_topic, min_hz in [
            ("/local_costmap/costmap", self.args.min_local_cm_hz),
            ("/global_costmap/costmap", self.args.min_global_cm_hz),
        ]:
            msg_type = self.resolve_costmap_msg_type(cm_topic)
            if msg_type is None:
                checks.append(CheckResult(False, f"costmap type {cm_topic}", f"topic types={ttypes.get(cm_topic)}"))
                continue
            rate, count = self.measure_rate(cm_topic, msg_type, window_s=self.args.rate_window)
            ok = rate >= min_hz
            checks.append(CheckResult(ok, f"costmap rate {cm_topic}", f"{rate:.2f} Hz ({count} msgs/{self.args.rate_window:.1f}s)"))

        # ---------------- Print summary ----------------
        print(f"\n[{_now()}] ===== Nav2 PRE-FLIGHT =====")
        any_fail = False
        for c in checks:
            tag = "✅ PASS" if c.ok else "⛔ FAIL"
            print(f"{tag:8} | {c.name:28} | {c.details}")
            if not c.ok:
                any_fail = True

        # Sugerencias rápidas si falla TF
        if any_fail:
            print("\n--- HINTS rápidos (si hay FAIL) ---")
            print("1) Si falla TF base_link<-odom: el robot está 'ciego' para el local_costmap.")
            print("   Normalmente es EKF/odom TF caído o no arrancado.")
            print("2) Si falla TF map<-odom: AMCL (SLAM=False) no está publicando map->odom o no está localizado.")
            print("3) Si no hay /scan_filtered publisher: el costmap no tiene obstáculos aunque todo lo demás esté bien.")

        return 2 if any_fail else 0


def main():
    ap = argparse.ArgumentParser(description="Preflight duro para Nav2: TF + costmaps + scan + lifecycle.")
    ap.add_argument("--timeout", type=float, default=20.0, help="timeout global para esperar TF/publishers críticos")
    ap.add_argument("--scan-topic", default="/scan_filtered")
    ap.add_argument("--tf-topic", default="/tf")
    ap.add_argument("--rate-window", type=float, default=2.0)
    ap.add_argument("--min-scan-hz", type=float, default=5.0)
    ap.add_argument("--min-local-cm-hz", type=float, default=1.0)
    ap.add_argument("--min-global-cm-hz", type=float, default=0.5)
    args = ap.parse_args()

    rclpy.init()
    node = Nav2Preflight(args)
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
