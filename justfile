# --- justfile ---

# 0. arranque base
start-rosbot:
    docker compose up --build

# 1. grabar recorrido
record-path name:
    CID=$(docker compose ps -q path_tools)
    docker exec -i $$CID bash -c \
      "source /opt/ros/humble/setup.bash && \
       python3 /scripts/path_recorder.py --output /routes/{{name}}.yaml"

# 2. reproducir recorrido
play-path name:
    CID=$(docker compose ps -q path_tools)
    docker exec -i $$CID bash -c \
      "source /opt/ros/humble/setup.bash && \
       python3 /scripts/path_player.py --file /routes/{{name}}.yaml"

# 3. AUTOMÁTICO: mapa + play
start1:
    just start-rosbot &
    WAIT=0; \
    until docker compose ps navigation | grep -q "(healthy)"; do \
        sleep 2; WAIT=$$((WAIT+2)); [ $$WAIT -gt 60 ] && echo "⛔ nav2 no sano" && exit 1; \
    done
    just play-path circuito1
