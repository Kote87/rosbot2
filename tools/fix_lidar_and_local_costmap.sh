#!/usr/bin/env bash
set -euo pipefail
shopt -s nullglob
YAMLS=(config/nav2_*_params.yaml)
command -v yq >/dev/null || { echo "Instala yq primero"; exit 1; }

for F in "${YAMLS[@]}"; do
  echo "→ parcheando $F"
  yq -i '
    .amcl.ros__parameters.scan_topic = "/scan_filtered" |

    .local_costmap.ros__parameters.plugins = ["obstacle_layer","inflation_layer"] |
    .local_costmap.ros__parameters.obstacle_layer.plugin = "nav2_costmap_2d::ObstacleLayer" |
    .local_costmap.ros__parameters.obstacle_layer.enabled = true |
    .local_costmap.ros__parameters.obstacle_layer.footprint_clearing_enabled = true |
    .local_costmap.ros__parameters.obstacle_layer.observation_sources = "scan" |
    .local_costmap.ros__parameters.obstacle_layer.scan.topic = "/scan_filtered" |
    .local_costmap.ros__parameters.obstacle_layer.scan.data_type = "LaserScan" |
    .local_costmap.ros__parameters.obstacle_layer.scan.marking = true |
    .local_costmap.ros__parameters.obstacle_layer.scan.clearing = true |
    .local_costmap.ros__parameters.obstacle_layer.scan.inf_is_valid = true |
    .local_costmap.ros__parameters.obstacle_layer.scan.obstacle_min_range = 0.05 |
    .local_costmap.ros__parameters.obstacle_layer.scan.obstacle_max_range = 6.0 |
    .local_costmap.ros__parameters.obstacle_layer.scan.raytrace_min_range = 0.0 |
    .local_costmap.ros__parameters.obstacle_layer.scan.raytrace_max_range = 7.0 |
    .local_costmap.ros__parameters.obstacle_layer.scan.observation_persistence = 0.0 |
    .local_costmap.ros__parameters.obstacle_layer.scan.expected_update_rate = 0.0 |
    .local_costmap.ros__parameters.inflation_layer.plugin = "nav2_costmap_2d::InflationLayer" |
    .local_costmap.ros__parameters.inflation_layer.enabled = true |
    .local_costmap.ros__parameters.inflation_layer.inflation_radius = 0.10 |
    .local_costmap.ros__parameters.inflation_layer.cost_scaling_factor = 6.0 |

    .waypoint_follower.ros__parameters.stop_on_failure = false |
    .waypoint_follower.ros__parameters.waypoint_task_executor.plugin = "nav2_waypoint_follower::WaitAtWaypoint" |
    .waypoint_follower.ros__parameters.waypoint_task_executor.enabled = true |
    .waypoint_follower.ros__parameters.waypoint_task_executor.wait_duration = "0.0s"
  ' "$F"
done

echo "✓ Listo. Reinicia sólo navegación:  docker compose restart navigation"
