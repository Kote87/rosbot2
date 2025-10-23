#!/usr/bin/env python3
"""Active wait for a transform before starting Nav2 tooling."""

from __future__ import annotations

import argparse
import sys
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener


class TFWaiter(Node):
    """Spin until a transform between two frames becomes available."""

    def __init__(self, target_frame: str, source_frame: str) -> None:
        super().__init__("wait_for_tf")
        self._target_frame = target_frame
        self._source_frame = source_frame
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

    def wait(self, timeout: float) -> bool:
        deadline: Optional[Duration] = None if timeout <= 0 else Duration(seconds=float(timeout))
        start_time = self.get_clock().now()
        self.get_logger().info(
            f"Esperando TF {self._source_frame} → {self._target_frame} "
            f"(timeout {timeout:.1f} s)..."
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                self._buffer.lookup_transform(
                    self._target_frame,
                    self._source_frame,
                    rclpy.time.Time(),
                )
                self.get_logger().info(
                    f"TF {self._source_frame} → {self._target_frame} disponible."
                )
                return True
            except (LookupException, ConnectivityException, ExtrapolationException):
                if deadline is not None and self.get_clock().now() - start_time > deadline:
                    break
        self.get_logger().error(
            f"TF {self._source_frame} → {self._target_frame} "
            f"no disponible tras {timeout:.1f} s."
        )
        return False


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target_frame", help="Target frame (e.g. odom)")
    parser.add_argument("source_frame", help="Source frame (e.g. base_link)")
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Timeout in seconds (<=0 to wait forever)",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = TFWaiter(args.target_frame, args.source_frame)
    try:
        success = node.wait(args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
