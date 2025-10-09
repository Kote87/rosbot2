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
    docker compose -f compose.yaml -f docker-compose.override.yml up -d \
      rosbot rplidar navigation microros scan_filter ekf foxglove foxglove-ds teleop
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

# Reproducir un recorrido con Nav2 (esquiva de obstáculos)
play-path name:
    @echo "Ejecutando recorrido {{name}}"
    docker exec -it $(docker compose ps -q path_tools) \
        bash -c "source /opt/ros/humble/setup.bash && \
                 python3 /scripts/path_player.py \
                 --file /routes/{{name}}.yaml"

## Reproducir un recorrido con NTP (NavigateThroughPoses) – movimiento fluido
play-ntp ruta:
    @echo "Ejecutando recorrido (NTP) {{ruta}}"
    docker exec -it $(docker compose ps -q path_tools) \
        bash -c "source /opt/ros/humble/setup.bash && \
                 python3 /scripts/nav_through_poses.py \
                 --file /routes/{{ruta}}.yaml"
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

    # 3) Lanza NTP dentro de path_tools
    @just play-ntp {{ruta}}

# ────────────────────────────────────────────────────────────────
#  ruta1  →  atajo sin parámetros. Equivale a:
#           just start-route mi_ruta
# ────────────────────────────────────────────────────────────────
ruta1:
    just start-route mi_ruta

# --- Diagnóstico ODOM/TF sin y con Nav2 ------------------------
diag-odom-tf:
    #!/usr/bin/env bash
    set -euo pipefail

    echo "▶ Drivers + Lidar + Filtro + micro-ROS"
    docker compose up -d microros rosbot rplidar scan_filter

    echo
    echo "▶ Chequeos dentro de 'rosbot'"
    docker compose exec rosbot bash -lc '
      source /opt/ros/humble/setup.bash
      echo "== nodos =="; ros2 node list || true
      echo "== Odometry topics =="; ros2 topic list | grep -E "/rosbot_xl_base_controller/odom$|^/odometry/filtered$" || true
      echo "== info /rosbot_xl_base_controller/odom =="; ros2 topic info -v /rosbot_xl_base_controller/odom || true
      echo "== info /odometry/filtered ==";             ros2 topic info -v /odometry/filtered || true
      echo "== hz /rosbot_xl_base_controller/odom (8s) =="; timeout 8 ros2 topic hz /rosbot_xl_base_controller/odom || true
      echo "== hz /odometry/filtered (8s) ==";             timeout 8 ros2 topic hz /odometry/filtered || true
      echo "== TF odom->base_link (8s) ==";                 timeout 8 ros2 run tf2_ros tf2_echo odom base_link || true
    '

    echo
    echo "▶ Lanzando Nav2 (para map->odom)"
    docker compose up -d navigation

    echo
    echo "▶ Chequeos dentro de 'navigation'"
    docker compose exec navigation bash -lc '
      source /opt/ros/humble/setup.bash
      echo "== TF map->odom (8s) ==";       timeout 8 ros2 run tf2_ros tf2_echo map odom || true
      echo "== TF odom->base_link (8s) =="; timeout 8 ros2 run tf2_ros tf2_echo odom base_link || true
    '

# --- Bridge temporal ODOM→TF (si el base no publica odom->base_link) ----
bridge-odom-tf:
    #!/usr/bin/env bash
    set -euo pipefail

    echo "▶ Creando /tmp/odom_tf_bridge.py dentro de 'rosbot' (sin heredoc en just)"
    docker compose exec -T rosbot bash -lc '
      set -e
      tmp=/tmp/odom_tf_bridge.py
      : > "$tmp"
      printf "%s\n" "import rclpy" >> "$tmp"
      printf "%s\n" "from rclpy.node import Node" >> "$tmp"
      printf "%s\n" "from nav_msgs.msg import Odometry" >> "$tmp"
      printf "%s\n" "from tf2_ros import TransformBroadcaster" >> "$tmp"
      printf "%s\n" "from geometry_msgs.msg import TransformStamped" >> "$tmp"
      printf "%s\n" "" >> "$tmp"
      printf "%s\n" "SRC_TOPICS = [\x22/rosbot_xl_base_controller/odom\x22, \x22/odometry/filtered\x22]" >> "$tmp"
      printf "%s\n" "" >> "$tmp"
      printf "%s\n" "class OdomTFBridge(Node):" >> "$tmp"
      printf "%s\n" "    def __init__(self):" >> "$tmp"
      printf "%s\n" "        super().__init__(\x22odom_tf_bridge\x22)" >> "$tmp"
      printf "%s\n" "        self.br = TransformBroadcaster(self)" >> "$tmp"
      printf "%s\n" "        self.pub = self.create_publisher(Odometry, \x22/odom\x22, 10)" >> "$tmp"
      printf "%s\n" "        self.subs = [self.create_subscription(Odometry, t, self.cb, 10) for t in SRC_TOPICS]" >> "$tmp"
      printf "%s\n" "        self.get_logger().info(f\x22Esperando odometría en: {SRC_TOPICS}\x22)" >> "$tmp"
      printf "%s\n" "" >> "$tmp"
      printf "%s\n" "    def cb(self, msg: Odometry):" >> "$tmp"
      printf "%s\n" "        msg.header.frame_id = \x22odom\x22" >> "$tmp"
      printf "%s\n" "        if not msg.child_frame_id:" >> "$tmp"
      printf "%s\n" "            msg.child_frame_id = \x22base_link\x22" >> "$tmp"
      printf "%s\n" "        self.pub.publish(msg)" >> "$tmp"
      printf "%s\n" "        t = TransformStamped()" >> "$tmp"
      printf "%s\n" "        t.header.stamp = self.get_clock().now().to_msg()" >> "$tmp"
      printf "%s\n" "        t.header.frame_id = \x22odom\x22" >> "$tmp"
      printf "%s\n" "        t.child_frame_id  = msg.child_frame_id" >> "$tmp"
      printf "%s\n" "        t.transform.translation.x = msg.pose.pose.position.x" >> "$tmp"
      printf "%s\n" "        t.transform.translation.y = msg.pose.pose.position.y" >> "$tmp"
      printf "%s\n" "        t.transform.translation.z = msg.pose.pose.position.z" >> "$tmp"
      printf "%s\n" "        t.transform.rotation      = msg.pose.pose.orientation" >> "$tmp"
      printf "%s\n" "        self.br.sendTransform(t)" >> "$tmp"
      printf "%s\n" "" >> "$tmp"
      printf "%s\n" "rclpy.init()" >> "$tmp"
      printf "%s\n" "try:" >> "$tmp"
      printf "%s\n" "    rclpy.spin(OdomTFBridge())" >> "$tmp"
      printf "%s\n" "finally:" >> "$tmp"
      printf "%s\n" "    rclpy.shutdown()" >> "$tmp"
      nohup python3 "$tmp" >/tmp/odom_tf_bridge.log 2>&1 &
    '

    echo
    echo "▶ Verificando TF odom->base_link"
    docker compose exec -T rosbot bash -lc '
      source /opt/ros/humble/setup.bash
      timeout 8 ros2 run tf2_ros tf2_echo odom base_link || true
    '

# --- Bridge ODOM→TF con QoS explícitos (FIX: retener suscripciones) -------------
bridge-odom-tf-qos:
    #!/usr/bin/env bash
    set -euo pipefail
    docker compose exec -T rosbot bash -lc '
      source /opt/ros/humble/setup.bash
      tmp=/tmp/odom_tf_bridge_qos.py
      : > "$tmp"
      printf "%s\n" "import rclpy" >> "$tmp"
      printf "%s\n" "from rclpy.node import Node" >> "$tmp"
      printf "%s\n" "from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy" >> "$tmp"
      printf "%s\n" "from nav_msgs.msg import Odometry" >> "$tmp"
      printf "%s\n" "from tf2_ros import TransformBroadcaster" >> "$tmp"
      printf "%s\n" "from geometry_msgs.msg import TransformStamped" >> "$tmp"
      printf "%s\n" "" >> "$tmp"
      printf "%s\n" "qos_best = QoSProfile(depth=10)" >> "$tmp"
      printf "%s\n" "qos_best.reliability = QoSReliabilityPolicy.BEST_EFFORT" >> "$tmp"
      printf "%s\n" "qos_best.history = QoSHistoryPolicy.KEEP_LAST" >> "$tmp"
      printf "%s\n" "qos_best.durability = QoSDurabilityPolicy.VOLATILE" >> "$tmp"
      printf "%s\n" "qos_rel = QoSProfile(depth=10)" >> "$tmp"
      printf "%s\n" "" >> "$tmp"
      printf "%s\n" "class OdomTFBridge(Node):" >> "$tmp"
      printf "%s\n" "    def __init__(self):" >> "$tmp"
      printf "%s\n" "        super().__init__(\"odom_tf_bridge_qos\")" >> "$tmp"
      printf "%s\n" "        self.br  = TransformBroadcaster(self)" >> "$tmp"
      printf "%s\n" "        self.pub = self.create_publisher(Odometry, \"/odom\", 10)" >> "$tmp"
      printf "%s\n" "        # RETENER suscripciones para evitar GC:" >> "$tmp"
      printf "%s\n" "        self.sub_odom     = self.create_subscription(Odometry, \"/rosbot_xl_base_controller/odom\", self.cb, qos_best)" >> "$tmp"
      printf "%s\n" "        self.sub_filtered = self.create_subscription(Odometry, \"/odometry/filtered\",              self.cb, qos_rel)" >> "$tmp"
      printf "%s\n" "        self.first = True" >> "$tmp"
      printf "%s\n" "        self.get_logger().info(\"Bridge activo: BEST_EFFORT + RELIABLE\")" >> "$tmp"
      printf "%s\n" "" >> "$tmp"
      printf "%s\n" "    def cb(self, msg: Odometry):" >> "$tmp"
      printf "%s\n" "        if self.first:" >> "$tmp"
      printf "%s\n" "            self.get_logger().info(f\"Primer odom recibido: {msg.header.frame_id} -> {msg.child_frame_id}\")" >> "$tmp"
      printf "%s\n" "            self.first = False" >> "$tmp"
      printf "%s\n" "        # Normaliza frames" >> "$tmp"
      printf "%s\n" "        msg.header.frame_id = \"odom\"" >> "$tmp"
      printf "%s\n" "        if not msg.child_frame_id:" >> "$tmp"
      printf "%s\n" "            msg.child_frame_id = \"base_link\"" >> "$tmp"
      printf "%s\n" "        self.pub.publish(msg)" >> "$tmp"
      printf "%s\n" "        # TF dinámica con sello del mensaje" >> "$tmp"
      printf "%s\n" "        t = TransformStamped()" >> "$tmp"
      printf "%s\n" "        t.header.stamp = msg.header.stamp" >> "$tmp"
      printf "%s\n" "        t.header.frame_id = \"odom\"" >> "$tmp"
      printf "%s\n" "        t.child_frame_id  = msg.child_frame_id" >> "$tmp"
      printf "%s\n" "        t.transform.translation.x = msg.pose.pose.position.x" >> "$tmp"
      printf "%s\n" "        t.transform.translation.y = msg.pose.pose.position.y" >> "$tmp"
      printf "%s\n" "        t.transform.translation.z = msg.pose.pose.position.z" >> "$tmp"
      printf "%s\n" "        t.transform.rotation      = msg.pose.pose.orientation" >> "$tmp"
      printf "%s\n" "        self.br.sendTransform(t)" >> "$tmp"
      printf "%s\n" "" >> "$tmp"
      printf "%s\n" "rclpy.init()" >> "$tmp"
      printf "%s\n" "try:" >> "$tmp"
      printf "%s\n" "    rclpy.spin(OdomTFBridge())" >> "$tmp"
      printf "%s\n" "finally:" >> "$tmp"
      printf "%s\n" "    rclpy.shutdown()" >> "$tmp"
      nohup python3 -u "$tmp" >/tmp/odom_tf_bridge.log 2>&1 &
      sleep 1
      echo "== proceso bridge =="; pgrep -fal odom_tf_bridge_qos || true
      echo "== TF odom->base_link (8s) =="; timeout 8 ros2 run tf2_ros tf2_echo odom base_link || true
      echo "== log bridge (cola) =="; tail -n 50 /tmp/odom_tf_bridge.log || true
    '
