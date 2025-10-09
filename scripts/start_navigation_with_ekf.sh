#!/bin/bash
set -euo pipefail

source /opt/ros/humble/setup.bash
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source /ros2_ws/install/setup.bash
fi

echo "[EKF] Lanzando robot_localization::ekf_node con /config/ekf.yaml ..."
ros2 run robot_localization ekf_node --ros-args --params-file /config/ekf.yaml >/tmp/ekf.log 2>&1 &
EKF_PID=$!
trap 'kill "${EKF_PID}" 2>/dev/null || true' EXIT

sleep 1
echo "[EKF] Comprobando odom->base_link (hasta 5s)..."
for i in {1..5}; do
  if timeout 1 ros2 run tf2_ros tf2_echo odom base_link >/dev/null 2>&1; then
    echo "[EKF] OK: odom->base_link disponible"
    break
  fi
  sleep 1
done

ros2 launch /husarion_utils/bringup_launch.py \
  slam:=${SLAM:-True} \
  params_file:=/params.yaml \
  map:=/maps/${MAP:-r1}.yaml \
  use_sim_time:=False
