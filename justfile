set dotenv-load

[private]
default:
    @just --list --unsorted

[private]
alias husarnet := connect-husarnet
[private]
alias flash := flash-firmware
[private]
alias rosbot := start-rosbot
[private]
alias start := start-rosbot

[private]
gazebo: (start-simulation "gazebo")

[private]
webots: (start-simulation "webots")

[private]
pre-commit:
    #!/bin/bash
    if ! command -v pre-commit &> /dev/null; then
        pip install pre-commit
        pre-commit install
    fi
    pre-commit run -a

# connect to Husarnet VPN network
connect-husarnet joincode hostname: _run-as-root
    #!/bin/bash
    if ! command -v husarnet > /dev/null; then
        echo "Husarnet is not installed. Installing now..."
        curl https://install.husarnet.com/install.sh | bash
    fi
    husarnet join {{joincode}} {{hostname}}

# Copy repo content to remote host with 'rsync' and watch for changes
sync hostname="${ROBOT_NAMESPACE}" password="husarion": _install-rsync _run-as-user
    #!/bin/bash
    mkdir -m 775 -p maps
    sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --exclude='.git/' --exclude='maps/' ; do
        sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done

# flash the proper firmware for STM32 microcontroller in ROSbot XL
flash-firmware: _install-yq _run-as-user
    #!/bin/bash
    echo "Stopping all running containers"
    docker ps -q | xargs -r docker stop

    echo "Flashing the firmware for STM32 microcontroller in ROSbot"
    docker run \
        --rm -it \
        --device /dev/ttyUSBDB \
        --device /dev/bus/usb/ \
        $(yq .services.rosbot.image compose.yaml) \
        flash-firmware.py -p /dev/ttyUSBDB # todo
        # ros2 run rosbot_utils flash_firmware

# start containers on a physical ROSbot XL
start-rosbot: _run-as-user
    #!/bin/bash
    mkdir -m 775 -p maps
    docker compose -f compose.yaml down
    docker compose -f compose.yaml pull
    docker compose -f compose.yaml up

# start the simulation (available options: gazebo, webots)
start-simulation engine="gazebo": _run-as-user
    #!/bin/bash
    xhost +local:docker
    if [[ "{{engine}}" == "gazebo" ]]; then
        export SIMULATION_DOCKER_IMAGE="husarion/rosbot-xl-gazebo:humble-0.9.1-20240131"
        export SIMULATION_COMMAND="ros2 launch rosbot_xl_gazebo simulation.launch.py mecanum:=${MECANUM:-True}"
    elif [[ "{{engine}}" == "webots" ]]; then
        export SIMULATION_DOCKER_IMAGE="husarion/webots:humble-2023.0.4-20230809-stable"
        export SIMULATION_COMMAND="ros2 launch webots_ros2_husarion rosbot_xl_launch.py"
    else
        echo -e "\e[1;33mUnknown ROS 2 simulation engine: {{engine}}\e[0m"
        exit 1
    fi
    docker compose -f compose.simulation.yaml down
    docker compose -f compose.simulation.yaml pull
    docker compose -f compose.simulation.yaml up

# Restart the Nav2 container
restart-navigation: _run-as-user
    #!/bin/bash
    docker compose down navigation
    docker compose up -d navigation

_run-as-root:
    #!/bin/bash
    if [ "$EUID" -ne 0 ]; then
        echo -e "\e[1;33mPlease re-run as root user to install dependencies\e[0m"
        exit 1
    fi

_run-as-user:
    #!/bin/bash
    if [ "$EUID" -eq 0 ]; then
        echo -e "\e[1;33mPlease re-run as non-root user\e[0m"
        exit 1
    fi

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null || ! command -v sshpass &> /dev/null || ! command -v inotifywait &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 1
        fi
        apt install -y rsync sshpass inotify-tools
    fi

_install-yq:
    #!/bin/bash
    if ! command -v /usr/bin/yq &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 1
        fi

        YQ_VERSION=v4.35.1
        ARCH=$(arch)

        if [ "$ARCH" = "x86_64" ]; then
            YQ_ARCH="amd64"
        elif [ "$ARCH" = "aarch64" ]; then
            YQ_ARCH="arm64"
        else
            YQ_ARCH="$ARCH"
        fi

        curl -L https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH} -o /usr/bin/yq
        chmod +x /usr/bin/yq
        echo "yq installed successfully!"
    fi

teleop:
    @echo "Starting sensors + SLAM + teleop…"
    docker compose -f compose.yaml -f docker-compose.override.yml up -d
    @echo ""
    @echo "╭───────────────────────────────────────────"
    @echo "│  TELEOP  (W/S = adelante/atrás)"
    @echo "│          A/D = girar;  Q/E = giro suave"
    @echo "│  Salir sin matar:  Ctrl-P  Ctrl-Q"
    @echo "╰───────────────────────────────────────────"
    # Abrimos la cámara en paralelo (comprimida)
    @bash -lc '( source /opt/ros/humble/setup.bash; just camera-oak ) >/dev/null 2>&1 &' || true
    docker attach $(docker compose -f compose.yaml -f docker-compose.override.yml ps -q teleop)

# ────────────────────────────────────────────────────────────────
#  Cámara OAK-D Pro: visor en host (pantalla HDMI) comprimido
#    • Espera a que exista el tópico y abre rqt_image_view en 2º plano
#    • Controles: Ctrl+1 = zoom 1:1, F11 = pantalla completa
# ────────────────────────────────────────────────────────────────
camera-oak topic="/oak/rgb/image" transport="compressed": _run-as-user
    #!/bin/bash
    set -e
    # Nos aseguramos de tener el entorno ROS 2 en el HOST
    source /opt/ros/humble/setup.bash || true
    # Espera hasta que rosbot esté arriba y publique imagen
    for i in $(seq 1 30); do
        if ros2 topic list | grep -q "{{topic}}"; then break; fi
        sleep 1
    done
    # Lanza el visor y lo desancla de la terminal
    (
        rqt_image_view \
            --ros-args \
            -p image:={{topic}} \
            -p image_transport:={{transport}} \
        >/tmp/rqt_image_view.log 2>&1 & disown
    )
    echo "OAK-D Pro → {{topic}} ({{transport}}).  Controles: Ctrl+1 (1:1), F11 (fullscreen)"

# ────────────────────────────────────────────────────────────────
#  Helper: arranca NTP con un MAP concreto (usa tu start-ntp)
# ────────────────────────────────────────────────────────────────
_run-route-with-map ruta map: _run-as-user
    #!/bin/bash
    set -e
    MAP={{map}} just start-ntp {{ruta}}

# ────────────────────────────────────────────────────────────────
#  Atajos directos pedidos (mapa + ruta + cámara)
#   socis1 → ruta socis1  con mapa msocis1
#   socis2 → ruta socis2  con mapa msocis2
#   socis3 → ruta socis2* con mapa msocis3   (*tal como pediste)
# ────────────────────────────────────────────────────────────────
socis1: _run-as-user
    #!/bin/bash
    echo "SOCIS1: mapa=msocis, ruta=socis1"
    # Abre visor en paralelo
    ( just camera-oak >/dev/null 2>&1 & )
    # Lanza navegación + NTP con el mapa correcto
    just _run-route-with-map socis1 msocis

socis2: _run-as-user
    #!/bin/bash
    echo "SOCIS2: mapa=msocis2, ruta=socis2"
    ( just camera-oak >/dev/null 2>&1 & )
    just _run-route-with-map socis2 msocis2

socis3: _run-as-user
    #!/bin/bash
    echo "SOCIS3: mapa=msocis3, ruta=socis3 (según tu indicación)"
    ( just camera-oak >/dev/null 2>&1 & )
    just _run-route-with-map socis3 msocis3

# ------------------ Rutas grabadas ---------------------------------

# Grabar un recorrido con teleoperación activa
record-path name:
    @echo "Grabando recorrido {{name}} — pulsa Ctrl-C para terminar"
    docker compose exec -it path_tools bash -lc \
      "source /opt/ros/humble/setup.bash && \
       python3 /scripts/path_recorder.py --output /routes/{{name}}.yaml"

# Reproducir un recorrido con Nav2 (esquiva de obstáculos)
play-path name:
    @echo "Ejecutando recorrido {{name}}"
    docker exec -it $(docker compose ps -q path_tools) \
        bash -c "source /opt/ros/humble/setup.bash && \
                 python3 /scripts/path_player.py \
                 --file /routes/{{name}}.yaml"

## Reproducir un recorrido con NTP (NavigateThroughPoses) – movimiento fluido
play-ntp name:
    @echo "Ejecutando recorrido (NTP) {{name}}"
    docker exec -it $(docker compose ps -q path_tools) \
        bash -c "source /opt/ros/humble/setup.bash && \
                 python3 /scripts/nav_through_poses.py \
                 --file /routes/{{name}}.yaml \
                 --pre auto --force-stateful \
                 --bt /ros2_ws/bt/ntp_no_early_goal.xml"
# ────────────────────────────────────────────────────────────────
#  start-route  →  Arranca ROSbot con mapa fijo y reproduce una ruta
#     Uso:  just start-route mi_ruta        # (omite la extensión .yaml)
# ────────────────────────────────────────────────────────────────
start-route ruta="mi_ruta":
    # 1) Levanta sólo compose.yaml (sin el override ⇒ no arranca teleop)
    @SLAM=False docker compose -f compose.yaml up -d

    # 2) Espera a que el servicio navigation aparezca sano (máx 60 s)
    @echo "⌛  Esperando a Nav2..."
    @bash -lc 'for i in {1..30}; do \
        docker compose ps navigation | grep -q "(healthy)" && exit 0; \
        sleep 2; done; echo "⛔  navigation no healthy"; exit 1'

    # 2a) Espera explícita a que las acciones de Nav2 estén disponibles
    @docker compose exec navigation bash -lc 'source /opt/ros/humble/setup.bash; \
      for i in $(seq 1 30); do \
        ros2 action list | grep -Eq "/follow_waypoints|/navigate_through_poses" && exit 0; \
        sleep 1; done; echo "⛔  acciones de Nav2 no disponibles"; exit 1'

    # 2b) Limpieza robusta de costmaps (intenta ambos servicios conocidos)
    @docker compose exec navigation bash -lc 'export AMENT_TRACE_SETUP_FILES=""; /ros2_ws/scripts/clear_costmaps.sh' || true

    # 2c) Verifica que los costmaps realmente están suscritos a /scan_filtered
    @docker compose exec navigation bash -lc 'python3 /ros2_ws/scripts/nav2_guard.py'

    # 3) Lanza el reproductor de waypoints dentro de path_tools
    @just play-path {{ruta}}

## ───────────────────────────────────────────────────────────────
##  start-ntp  →  Igual que start-route pero usando NTP (fluido)
##      Uso:  just start-ntp mi_ruta
## ───────────────────────────────────────────────────────────────
start-ntp ruta="mi_ruta":
    # 1) Levanta solo compose.yaml (sin override ⇒ no teleop)
    @SLAM=False docker compose -f compose.yaml up -d

    # 2) Espera a Nav2 'healthy'
    @echo "⌛  Esperando a Nav2..."
    @bash -lc 'for i in {1..30}; do \
        docker compose ps navigation | grep -q "(healthy)" && exit 0; \
        sleep 2; done; echo "⛔  navigation no healthy"; exit 1'

    # 2a) Espera a que /navigate_through_poses esté disponible
    @docker compose exec navigation bash -lc 'source /opt/ros/humble/setup.bash; \
      for i in $(seq 1 30); do \
        ros2 action list | grep -q "/navigate_through_poses" && exit 0; \
        sleep 1; done; echo "⛔  /navigate_through_poses no disponible"; exit 1'

    # 2b) Limpieza de costmaps (evita aviso de AMENT_TRACE_SETUP_FILES)
    @docker compose exec navigation bash -lc 'export AMENT_TRACE_SETUP_FILES=""; /ros2_ws/scripts/clear_costmaps.sh' || true

    # 2c) Verifica que los costmaps están suscritos a /scan_filtered
    @docker compose exec navigation bash -lc 'python3 /ros2_ws/scripts/nav2_guard.py'
    # 2d) Instala BT mínimo para NTP (evita éxito inmediato al inicio)
    @docker compose exec navigation python3 -c 'from pathlib import Path; bt_dir = Path("/ros2_ws/bt"); bt_dir.mkdir(parents=True, exist_ok=True); bt_dir.joinpath("ntp_no_early_goal.xml").write_text("""<root main_tree_to_execute="MainTree">\n  <BehaviorTree ID="MainTree">\n    <Sequence name="ntp_no_early_goal">\n      <ComputePathThroughPoses goals="{goals}" path="{path}"/>\n      <FollowPath path="{path}"/>\n      <GoalReached/>\n    </Sequence>\n  </BehaviorTree>\n</root>\n""")'

    # 3) Lanza NTP dentro de path_tools
    @just play-ntp {{ruta}}

    @printf $'5) Afinados recomendados (opcionales)\n\nTolerancias: si quieres hacerlo "más difícil" de cantar meta (incluso con el BT mínimo), baja tolerancias en tus YAML (xy_goal_tolerance: 0.08, yaw_goal_tolerance: 0.15) en ambos plugins (goal_checker y general_goal_checker). Esto no es imprescindible con el BT, pero endurece el cierre.\n\nTF al arranque: los avisos de Timed out waiting for transform base_link→odom sólo al inicio son normales, pero si quieres evitar sustos, puedes añadir una espera activa a TF en start-ntp antes de lanzar el script (igual que ya esperas a las acciones). Tu layout y herramientas ya pintan /plan, /unsmoothed_plan, /goal_pose, etc., así que es fácil ver que está funcionando.\n'

# ────────────────────────────────────────────────────────────────
#  ruta1  →  atajo sin parámetros. Equivale a:
#           just start-route mi_ruta
# ────────────────────────────────────────────────────────────────
ruta1:
    just start-route mi_ruta
