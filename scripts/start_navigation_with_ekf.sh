#!/bin/bash
set -euo pipefail

source /opt/ros/humble/setup.bash
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source /ros2_ws/install/setup.bash
fi

ros2 run robot_localization ekf_node --ros-args --params-file /config/ekf.yaml &
EKF_PID=$!
trap 'kill "${EKF_PID}" 2>/dev/null || true' EXIT

ros2 launch /husarion_utils/bringup_launch.py \
  slam:=${SLAM:-True} \
  params_file:=/params.yaml \
  map:=/maps/${MAP:-r1}.yaml \
  use_sim_time:=False
