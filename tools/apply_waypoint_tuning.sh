#!/usr/bin/env bash
set -euo pipefail

# ===== Parámetros de tuning =====
XY_TOL="0.50"     # "puntos gordos": tolerancia XY (metros)
YAW_TOL="0.60"    # tolerancia de yaw (radianes)
DIST="0.50"       # distancia entre puntos grabados (metros)

echo "===> Ajuste de puntos gordos y separación de ruta"
echo "     xy_goal_tolerance=${XY_TOL}  yaw_goal_tolerance=${YAW_TOL}  DIST_THRESHOLD=${DIST}"

# --- 0) yq disponible (para editar YAML de forma segura)
if ! command -v yq >/dev/null 2>&1; then
  echo "yq no está instalado. Instalando versión estática en /usr/bin/yq (necesita sudo)..."
  sudo bash -c '
    set -e
    YQ_VERSION=v4.35.1
    ARCH=$(arch)
    case "$ARCH" in
      x86_64)  YQ_ARCH="amd64" ;;
      aarch64) YQ_ARCH="arm64" ;;
      arm64)   YQ_ARCH="arm64" ;;
      *)       YQ_ARCH="$ARCH" ;;
    esac
    curl -L "https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH}" -o /usr/bin/yq
    chmod +x /usr/bin/yq
  '
fi

# --- 1) Poner DIST_THRESHOLD=0.50 m en el grabador de rutas
REC_FILE="scripts/path_recorder.py"
if [ -f "$REC_FILE" ]; then
  sed -i -E "s/^(DIST_THRESHOLD\s*=\s*)([0-9.]+)/\1${DIST}/" "$REC_FILE"
  echo "Actualizado DIST_THRESHOLD en $REC_FILE → ${DIST} m"
else
  echo "AVISO: No encuentro $REC_FILE"
fi

# --- 2) Aumentar tolerancias del Goal Checker (puntos gordos) en Nav2
#     Soporta tus tres perfiles: mppi, rpp y dwb
for F in config/nav2_*_params.yaml; do
  [ -f "$F" ] || continue

  # ¿Usa plugins múltiples (DWB) o uno simple (MPPI/RPP)?
  if yq e '.controller_server.ros__parameters.goal_checker_plugins' "$F" >/dev/null 2>&1; then
    # Caso DWB: general_goal_checker
    yq -i ".controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance = ${XY_TOL}" "$F"
    yq -i ".controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance = ${YAW_TOL}" "$F"
    echo "Parches en $F (DWB): xy=${XY_TOL}, yaw=${YAW_TOL}"
  else
    # Caso MPPI/RPP: goal_checker
    yq -i ".controller_server.ros__parameters.goal_checker.xy_goal_tolerance = ${XY_TOL}" "$F"
    yq -i ".controller_server.ros__parameters.goal_checker.yaw_goal_tolerance = ${YAW_TOL}" "$F"
    echo "Parches en $F (MPPI/RPP): xy=${XY_TOL}, yaw=${YAW_TOL}"
  fi
done

echo "===> Listo. Revisa los cambios:"
grep -n 'DIST_THRESHOLD' scripts/path_recorder.py || true
echo "---"
for F in config/nav2_*_params.yaml; do
  echo "[ $F ]"
  yq e '.controller_server.ros__parameters.goal_checker.xy_goal_tolerance // .controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance' "$F" 2>/dev/null || true
  yq e '.controller_server.ros__parameters.goal_checker.yaw_goal_tolerance // .controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance' "$F" 2>/dev/null || true
  echo "---"
done

echo "Sugerencia: guarda cambios con 'git add -A && git commit -m \"tuning: puntos gordos + 0.50m entre puntos\"'"
