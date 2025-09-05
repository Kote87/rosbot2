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
    docker attach $(docker compose -f compose.yaml -f docker-compose.override.yml ps -q teleop)

# ------------------ Rutas grabadas ---------------------------------

# Grabar un recorrido con teleoperación activa
record-path name:
    @echo "Grabando recorrido {{name}} — pulsa Ctrl-C para terminar"
    docker compose exec -it path_tools bash -lc \
      "source /opt/ros/humble/setup.bash && \
       python3 /scripts/path_recorder.py --output /routes/{{name}}.yaml"

# Reproducir un recorrido en modo FLUIDO (NavigateThroughPoses)
play-path name:
    @echo "Ejecutando recorrido {{name}} (fluido, re-muestreo 5cm, salida robusta)"
    docker exec -it $(docker compose ps -q path_tools) \
        bash -c "source /opt/ros/humble/setup.bash && \
                 python3 /scripts/path_player.py --file /routes/{{name}}.yaml"
# ────────────────────────────────────────────────────────────────
#  start-route  →  Arranca ROSbot con mapa fijo y reproduce una ruta
#     Uso:  just start-route mi_ruta        # (omite la extensión .yaml)
# ────────────────────────────────────────────────────────────────
start-route ruta="r1":
    # 1) Levanta sólo compose.yaml (sin el override ⇒ no arranca teleop)
    @SLAM=False docker compose -f compose.yaml up -d

    # 2) Espera a que el servicio navigation aparezca sano (máx 60 s)
    @echo "⌛  Esperando a Nav2..."
    @bash -c 'for i in {1..30}; do \
        docker compose ps navigation | grep -q "(healthy)" && exit 0; \
        sleep 2; done; echo "⛔  navigation no healthy"; exit 1'

    # 3) Lanza el reproductor de waypoints dentro de path_tools
    @just play-path {{ruta}}

# ────────────────────────────────────────────────────────────────
#  ruta1  →  atajo sin parámetros. Equivale a:
#           just start-route mi_ruta
# ────────────────────────────────────────────────────────────────
ruta1:
    just start-route r1

# Genera PointCloud de la OAK (publica /oak/points) dentro de path_tools
oak-points:
    CID=$(docker compose ps -q path_tools)
    docker exec -it $$CID bash -lc \
      "source /opt/ros/humble/setup.bash && \
       ros2 run depth_image_proc point_cloud_xyz \
         --ros-args \
         -r depth:=/oak/stereo/depth \
         -r camera_info:=/oak/stereo/camera_info \
         -r points:=/oak/points"

# Detiene el conversor si quedó corriendo en otra terminal
oak-points-stop:
    CID=$(docker compose ps -q path_tools)
    docker exec -it $$CID bash -lc "pkill -f depth_image_proc || true"

# ============================ OAK / PointCloud =============================

# Reinicia sólo la cámara OAK (contenedor rosbot)
oak-restart:
    @docker compose restart rosbot

# Diagnóstico rápido de OAK (nodos, topics y publisher de /oak/points)
check-oak:
    @docker compose exec rosbot bash -lc '\
      source /opt/ros/humble/setup.bash && \
      echo "— nodes —" && ros2 node list | egrep -i "oak|depthai|point|camera" || true && \
      echo "— topics —" && ros2 topic list | egrep "^/oak/" || true && \
      echo "— /oak/points —" && ros2 topic info /oak/points || true && \
      echo "— stereo —" && (ros2 topic list | egrep "^/oak/stereo/" || echo "NO /oak/stereo/*") \
    '

# Fallback: si tu build publica /oak/stereo/image_rect, lo relay a /oak/stereo/image_raw
# (para que el nodo /oak_point_cloud_xyzrgb_node empiece a publicar /oak/points)
oak-relay-stereo:
    @docker compose exec -d rosbot bash -lc '\
      source /opt/ros/humble/setup.bash && \
      if ros2 topic list | grep -q "^/oak/stereo/image_rect$"; then \
        echo "Relaying /oak/stereo/image_rect -> /oak/stereo/image_raw"; \
        ros2 run topic_tools relay /oak/stereo/image_rect /oak/stereo/image_raw ; \
      else \
        echo "No existe /oak/stereo/image_rect (no se lanza relay)"; \
      fi \
    '

# Comprobación de TF para nube en 3D (repite si da extrapolation justo al arrancar)
oak-tf:
    @docker compose exec rosbot bash -lc '\
      source /opt/ros/humble/setup.bash && \
      timeout 3 ros2 run tf2_ros tf2_echo base_link oak_rgb_camera_optical_frame || true && \
      timeout 3 ros2 run tf2_ros tf2_echo map base_link || true \
    '
