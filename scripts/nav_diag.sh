#!/usr/bin/env bash
set -euo pipefail
echo "=== NODES ==="
docker compose exec -T navigation bash -lc 'source /opt/ros/humble/setup.bash; ros2 node list || true'
echo
echo "=== ACTIONS ==="
docker compose exec -T navigation bash -lc 'source /opt/ros/humble/setup.bash; ros2 action list || true'
echo
echo "=== TOPICS (map, amcl, scan) ==="
docker compose exec -T navigation bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic list | egrep "^/map$|^/amcl_pose$|^/scan_filtered$" || true'
echo
echo "=== LIFECYCLE STATES (nav) ==="
for n in /bt_navigator /planner_server /controller_server /behavior_server; do
  docker compose exec -T navigation bash -lc "source /opt/ros/humble/setup.bash; ros2 lifecycle get ${n} || true"
done
echo
echo "=== LIFECYCLE STATES (slam) ==="
for n in /slam_toolbox /map_saver; do
  docker compose exec -T navigation bash -lc "source /opt/ros/humble/setup.bash; ros2 lifecycle get ${n} || true"
done
echo
echo "=== LAST LOGS ==="
docker compose logs --no-log-prefix -n 120 navigation || true
docker compose logs --no-log-prefix -n 120 map_saver || true
