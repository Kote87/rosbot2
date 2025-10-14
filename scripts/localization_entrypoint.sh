#!/usr/bin/env bash
set -Eeo pipefail

# Evita el "unbound variable" de setup.bash con -u (no usamos -u) y define esta var
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}

source /opt/ros/humble/setup.bash

echo "[localization] host=$(hostname) — $(date)"
echo "[localization] comprobando paquete robot_localization…"
if ! ros2 pkg executables robot_localization | grep -q ekf_node; then
  echo "[FATAL] robot_localization/ekf_node NO está instalado en esta imagen."
  sleep 600; exit 1
fi

echo "[localization] YAML /config/ekf_odom.yaml (primeras 200 líneas):"
sed -n '1,200p' /config/ekf_odom.yaml || true

# Espera (no bloqueante) a que aparezcan los tópicos base
for topic in /rosbot_xl_base_controller/odom /imu_broadcaster/imu; do
  for i in $(seq 1 60); do
    if ros2 topic list | grep -q "^${topic}$"; then
      echo "[localization] encontrado ${topic}"
      break
    fi
    sleep 0.5
  done
done

echo "[localization] lanzando EKF en ns=/localization (nodo por defecto: ekf_filter_node)"
# (opcional) traza rápida de TF para verificar que el EKF publica algo al arrancar
( timeout 8s ros2 topic echo /tf | egrep -i "frame_id|child_frame_id" || true ) &

# Desactiva la TF de odometría del base_controller para evitar duplicados
ros2 param set /rosbot_xl_base_controller enable_odom_tf false || true

exec ros2 run robot_localization ekf_node \
  --ros-args -r __ns:=/localization \
  --params-file /config/ekf_odom.yaml \
  --log-level robot_localization:=info
