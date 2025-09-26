#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash

clear_if_exists() {
  local svc="$1"
  if ros2 service list | grep -qx "$svc"; then
    local typ
    typ="$(ros2 service type "$svc" 2>/dev/null || true)"
    [ -n "$typ" ] && timeout 6 ros2 service call "$svc" "$typ" "{}" || true
  fi
}

# Local (ambos nombres posibles según versión Nav2)
clear_if_exists /local_costmap/clear_entire_costmap
clear_if_exists /local_costmap/clear_entirely_local_costmap

# Global (ambos nombres posibles)
clear_if_exists /global_costmap/clear_entire_costmap
clear_if_exists /global_costmap/clear_entirely_global_costmap
