"""Simulation Gazebo multi-robot d'ARMADA.

    ros2 launch armada_simulation simulation.launch.py
    ros2 launch armada_simulation simulation.launch.py robots:=armada1,armada2,armada3
    ros2 launch armada_simulation simulation.launch.py robots:=armada1:-1.5:0:0,armada2:-1.5:0.5:1.57

`robots` = liste séparée par des virgules ; chaque élément est `nom` (placé automatiquement) ou `nom:x:y:yaw`.
Piloter un robot (même commande que pour le vrai ; topic /armada1/cmd_vel dans la fenêtre) :

    ros2 run rqt_robot_steering rqt_robot_steering
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_robots(spec, spacing):
    items = [s.strip() for s in spec.split(",") if s.strip()]
    robots = []
    for i, item in enumerate(items):
        parts = item.split(":")
        name = parts[0]
        if len(parts) == 4:
            x, y, yaw = parts[1:]
        elif len(parts) == 1:
            # alignés sur l'axe y, au bord ouest de l'arène, tournés vers le centre
            x, y, yaw = "-1.5", str(round((i - (len(items) - 1) / 2.0) * spacing, 3)), "0.0"
        else:
            raise RuntimeError(f"robot '{item}' : format attendu `nom` ou `nom:x:y:yaw`")
        robots.append((name, x, y, yaw))
    if len({r[0] for r in robots}) != len(robots):
        raise RuntimeError("deux robots ont le même nom : chaque namespace doit être unique")
    return robots


def _robots(context):
    robots = _parse_robots(LaunchConfiguration("robots").perform(context),
                           float(LaunchConfiguration("spacing").perform(context)))
    spawn = PathJoinSubstitution([FindPackageShare("armada_simulation"), "launch", "spawn_robot.launch.py"])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn),
            launch_arguments={"robot_name": name, "x": x, "y": y, "yaw": yaw}.items(),
        )
        for name, x, y, yaw in robots
    ]


def generate_launch_description():
    world = os.path.join(get_package_share_directory("armada_simulation"), "worlds", "armada_arena.sdf")
    gz_sim = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]))

    return LaunchDescription(
        [
            DeclareLaunchArgument("robots", default_value="armada1,armada2", description="voir l'en-tête du fichier"),
            DeclareLaunchArgument("spacing", default_value="0.4", description="écart (m) du placement automatique"),
            DeclareLaunchArgument("world", default_value=world),
            DeclareLaunchArgument("gui", default_value="true", description="false = serveur seul (pas de fenêtre)"),
            IncludeLaunchDescription(
                gz_sim,
                launch_arguments={"gz_args": ["-r ", LaunchConfiguration("world")]}.items(),
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            IncludeLaunchDescription(
                gz_sim,
                launch_arguments={"gz_args": ["-r -s --headless-rendering ", LaunchConfiguration("world")]}.items(),
                condition=UnlessCondition(LaunchConfiguration("gui")),
            ),
            # Horloge de simulation : tous les noeuds du côté ROS tournent en use_sim_time
            Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
                 arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen"),
            OpaqueFunction(function=_robots),
        ]
    )
