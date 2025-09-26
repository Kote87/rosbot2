#!/usr/bin/env python3
import subprocess
import sys
import time


DEF_CHECK = "source /opt/ros/humble/setup.bash && ros2 topic info {topic} -v || true"

def costmaps_subscribed(topic: str) -> bool:
    try:
        output = subprocess.check_output(
            ["bash", "-lc", DEF_CHECK.format(topic=topic)],
            text=True,
        )
    except subprocess.CalledProcessError:
        return False
    return "/local_costmap" in output and "/global_costmap" in output


def wait_for_costmaps(topic: str, attempts: int = 10, delay: float = 1.0) -> bool:
    for _ in range(attempts):
        if costmaps_subscribed(topic):
            return True
        time.sleep(delay)
    return False


def main() -> int:
    topic = "/scan_filtered"
    if not wait_for_costmaps(topic):
        print(
            f"\u26d4  {topic} NO está suscrito por los costmaps. Revisa 'topic: /scan_filtered' en los YAML.",
            file=sys.stderr,
        )
        return 1
    print(f"\u2705  {topic} OK: costmaps suscritos.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
