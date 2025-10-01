#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash || true

# Espera a que existan servicios
for i in $(seq 1 20); do
  ros2 service list >/tmp/svcs.txt || true
  grep -q local_costmap /tmp/svcs.txt && grep -q global_costmap /tmp/svcs.txt && break
  sleep 0.5
done

# Llama a todos los servicios conocidos de limpieza (según distro)
call_clear() {
  local svc="$1"
  if ros2 service list | grep -q "$svc"; then
    echo "→ limpiando $svc"
    ros2 service call "$svc" nav2_msgs/srv/ClearEntireCostmap "{}" || true
  fi
}

call_clear /local_costmap/clear_entirely_local_costmap
call_clear /local_costmap/clear_entirely
call_clear /global_costmap/clear_entirely_global_costmap
call_clear /global_costmap/clear_entirely
echo "✅ costmaps limpiados"
