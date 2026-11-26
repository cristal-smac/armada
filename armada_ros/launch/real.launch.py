"""PC de pilotage des robots ARMADA réels : un micro-ROS agent + le bringup de chaque robot.

    ros2 launch armada_ros real.launch.py robots:=armada1,armada2

Chaque ESP32 se connecte à l'agent (UDP, port 8888 par défaut) et publie dans son propre namespace, défini dans
son firmware (ROBOT_NAME de armada_esp_micro_ros.ino) : ce nom doit être dans `robots`. Un seul agent suffit pour
tous les robots.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _robots(context):
    names = [n.strip() for n in LaunchConfiguration("robots").perform(context).split(",") if n.strip()]
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("armada_ros"), "launch", "robot.launch.py"])),
            launch_arguments={"robot_name": name}.items(),
        )
        for name in names
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robots", default_value="armada1", description="noms des robots, séparés par des virgules"),
            DeclareLaunchArgument("agent", default_value="true", description="lancer le micro-ROS agent"),
            DeclareLaunchArgument("agent_port", default_value="8888", description="port UDP de l'agent (= AGENT_PORT du firmware)"),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                arguments=["udp4", "--port", LaunchConfiguration("agent_port")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("agent")),
            ),
            OpaqueFunction(function=_robots),
        ]
    )
