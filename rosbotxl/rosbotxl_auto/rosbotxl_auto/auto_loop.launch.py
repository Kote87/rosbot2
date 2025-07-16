from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

MAP = "/home/husarion/rosbotxl/maps/mapa_nave.yaml"
X0, Y0, YAW0 = 0.10, 0.05, 0.0  #  ⇦ Ajusta si tu “punto cero” cambia


def generate_launch_description():
    # 1. Drivers (start-rosbot) – sin SLAM
    teleop = ExecuteProcess(cmd=["just", "start-rosbot"], shell=True)

    # 2. Mata slam_toolbox y map_saver (garantía de /map único)
    kill = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "sleep 3 && pkill -f slam_toolbox || true && pkill -f map_saver || true",
        ],
        shell=True,
    )

    # 3. Nav2 con mapa fijo
    nav2 = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "nav2_bringup",
            "bringup_launch.py",
            "slam:=false",
            f"map:={MAP}",
            "autostart:=true",
        ],
        shell=True,
    )

    # 4. Localización global
    global_loc = Node(
        package="nav2_util",
        executable="simple_service_call",
        arguments=["/reinitialize_global_localization", "std_srvs/srv/Empty"],
        output="screen",
    )

    spin360 = Node(
        package="nav2_behavior_tree",
        executable="bt_navigator_cli",
        arguments=["spin", "--angle", "6.283"],
        output="screen",
    )

    goto_origin = Node(
        package="nav2_behavior_tree",
        executable="bt_navigator_cli",
        arguments=[
            "navigate_to_pose",
            "--x",
            str(X0),
            "--y",
            str(Y0),
            "--yaw",
            str(YAW0),
        ],
        output="screen",
    )

    loop = Node(
        package="rosbotxl_auto",
        executable="loop_rect",
        output="screen",
    )

    return LaunchDescription(
        [teleop, kill, nav2, global_loc, spin360, goto_origin, loop]
    )
