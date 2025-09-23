#!/usr/bin/env bash
set -euo pipefail

echo "[ros_diag] Iniciando snapshot periódico del grafo ROS…"
while true; do
  echo "──────── $(date '+%H:%M:%S') ────────"
  for t in /scan /scan_filtered /map; do
    if ros2 topic list | grep -q "^${t}$"; then
      echo "[ros_diag] ${t} → OK (existe)"
    else
      echo "[ros_diag] ${t} → NO VISTO"
    fi
  done
  echo "[ros_diag] Publishers /scan:"
  ros2 topic info -v /scan 2>/dev/null | sed -n '/Publisher count/,$p' || true
  echo "────────────────────────────────────"
  sleep 5
done
