A continuación tienes un paso-a-paso exhaustivo para que un ingeniero externo
añada la opción B (localización global automática + bucle rectangular) sin
tocar nada de lo que ya funciona (just start-rosbot, explore-lite, etc.).
Todos los cambios están confinados en un nuevo paquete y un nuevo "just recipe".

0 · Resumen de la lógica

```vbnet
just auto-loop
└─➤ inicia drivers (start-rosbot)           ← SIN slam_toolbox
   └─➤ lanza Nav2 con mapa fijo
      ├─ servicio /reinitialize_global_localization
      ├─ comportamiento Spin 360° (AMCL converge)
      ├─ navigate_to_pose  (x0,y0,yaw0)
      └─ loop_rect  (bucle 9.25 × 1.25 m indefinido)
```
Mapa usado: maps/mapa_nave.yaml
Pose de referencia: (x0 = 0.10 m, y0 = 0.05 m, yaw0 = 0.0 rad)
(Ajusta valores si tu “estación de salida” es otra.)

1 · Crear paquete aislado rosbotxl_auto

```bash
cd ~/rosbotxl
ros2 pkg create --build-type ament_python rosbotxl_auto
mkdir -p rosbotxl_auto/rosbotxl_auto
touch rosbotxl_auto/rosbotxl_auto/__init__.py
```

### 1.1 loop_rect.py – bucle rectangular
Guarda en `rosbotxl_auto/rosbotxl_auto/loop_rect.py`:

```python
#!/usr/bin/env python3
import math, time, rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

L = 9.25   # largo (m)
W = 1.25   # ancho (m)
PAUSE = 0.5  # s entre vueltas

def mk_pose(x, y, th):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z = math.sin(th/2); p.pose.orientation.w = math.cos(th/2)
    return p

def build_rect(start):
    x0, y0 = start.pose.position.x, start.pose.position.y
    th0 = 2*math.atan2(start.pose.orientation.z, start.pose.orientation.w)
    pts = []
    pts.append((x0+L*math.cos(th0), y0+L*math.sin(th0), th0+math.pi/2))
    pts.append((pts[-1][0]+W*math.cos(th0+math.pi/2),
                pts[-1][1]+W*math.sin(th0+math.pi/2), th0+math.pi))
    pts.append((pts[-1][0]+L*math.cos(th0+math.pi),
                pts[-1][1]+L*math.sin(th0+math.pi), th0+1.5*math.pi))
    pts.append((pts[-1][0]+W*math.cos(th0+1.5*math.pi),
                pts[-1][1]+W*math.sin(th0+1.5*math.pi), th0))
    return [mk_pose(*p) for p in pts]

def main():
    rclpy.init(); nav = BasicNavigator(); nav.waitUntilNav2Active()
    waypoints = build_rect(nav.getCurrentPose())
    while rclpy.ok():
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete(): time.sleep(0.2)
        time.sleep(PAUSE)

if __name__ == '__main__':
    main()
```

### 1.2 auto_loop.launch.py
Guarda en `rosbotxl_auto/rosbotxl_auto/auto_loop.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

MAP = '/home/husarion/rosbotxl/maps/mapa_nave.yaml'
X0, Y0, YAW0 = 0.10, 0.05, 0.0   #  ⇦ Ajusta si tu “punto cero” cambia

def generate_launch_description():
    # 1. Drivers (start-rosbot) – sin SLAM
    teleop = ExecuteProcess(cmd=['just','start-rosbot'], shell=True)

    # 2. Mata slam_toolbox y map_saver (garantía de /map único)
    kill = ExecuteProcess(
        cmd=['bash','-c','sleep 3 && pkill -f slam_toolbox || true && pkill -f map_saver || true'],
        shell=True
    )

    # 3. Nav2 con mapa fijo
    nav2 = ExecuteProcess(
        cmd=[
            'ros2','launch','nav2_bringup','bringup_launch.py',
            f'slam:=False',
            f'map:={MAP}',
            'autostart:=true'
        ],
        shell=True
    )

    # 4. Localización global
    global_loc = Node(
        package='nav2_util',
        executable='simple_service_call',
        arguments=['/reinitialize_global_localization', 'std_srvs/srv/Empty'],
        output='screen'
    )

    spin360 = Node(
        package='nav2_behavior_tree',
        executable='bt_navigator_cli',
        arguments=['spin','--angle','6.283'],
        output='screen'
    )

    goto_origin = Node(
        package='nav2_behavior_tree',
        executable='bt_navigator_cli',
        arguments=[
            'navigate_to_pose',
            '--x',str(X0),'--y',str(Y0),'--yaw',str(YAW0)
        ],
        output='screen'
    )

    loop = Node(
        package='rosbotxl_auto',
        executable='loop_rect',
        output='screen'
    )

    return LaunchDescription([teleop, kill, nav2, global_loc, spin360, goto_origin, loop])
```

### 1.3 setup.py

```python
from setuptools import setup

package_name = 'rosbotxl_auto'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'loop_rect = rosbotxl_auto.loop_rect:main',
        ],
    },
)
```

### 1.4 package.xml (añade dependencias)

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav2_simple_commander</exec_depend>
<exec_depend>nav2_util</exec_depend>
<exec_depend>nav2_behavior_tree</exec_depend>
```

### 1.5 Compilar

```bash
cd ~/rosbotxl
colcon build --packages-select rosbotxl_auto
source install/setup.bash
```

2 · Añadir la “just recipe” sin romper las existentes

Abre `justfile` y al final añade:

```makefile
# ejecuta el bucle rectangular autónomo
auto-loop: _run-as-user
    #!/bin/bash
    docker compose -f compose.yaml -f docker-compose.override.yml up -d auto_loop
```

3 · No toques explore‑lite ni tus YAML actuales

`explore_lite` queda “dormido” en el docker‑compose.override.yml; tu implementación no lo toca.

`config/nav2_*_params.yaml` siguen ahí para otras misiones.

El único YAML que se usa aquí es `maps/mapa_nave.yaml` (resolución 0.05, origin [-4.4,-10.3,0]).

4 · Uso por parte del operador

```bash
ssh rosbot            # o abre terminal local
cd ~/rosbotxl
just auto-loop
```
1. Los contenedores se levantan; el robot gira 360° (10–15 s).
2. AMCL converge y navega al punto (0.10, 0.05).
3. Comienza el rectángulo de 9.25 × 1.25 m en bucle, con evitación integrada.

Para detenerlo: Ctrl +C en la terminal o `docker compose down` si usas systemd/service.

5 · Cosas importantes a vigilar

| Ítem | Qué revisar | Ajuste si falla |
|------|-------------|----------------|
| /scan 10 Hz | `ros2 topic hz /scan` | Revisar servicio rplidar en compose |
| /map único | `ros2 topic info -v /map \| grep Publisher:` | |
| TF map→odom | `ros2 run tf2_ros tf2_echo map odom --once` | Esperar a que AMCL converja tras el giro |
| Acción disponible | `ros2 action list \| grep follow_waypoints` | |

6 · Pruebas sin tocar el robot real (opcional)

Usa tu simulador Webots/Gazebo:

```bash
just webots          # o just gazebo
source install/setup.bash
just auto-loop
```
Verás el modelo girar, localizarse y empezar el rectángulo. Una vez validado, ejecuta la misma receta en el robot físico.

Con estos cambios no se modifica tu flujo de trabajo actual (just start‑rosbot, explore‑lite, tele‑op, etc.). Solo se añade capacidad de arranque autónomo, localización global y recorrido rectangular infinito mediante el nuevo comando `just auto-loop`.

