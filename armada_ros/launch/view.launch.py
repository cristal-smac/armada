"""RViz sur UN robot (réel ou simulé) : modèle, TF, anneau ToF, capteur de sol.

    ros2 launch armada_ros view.launch.py robot_name:=armada1
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz(context):
    name = LaunchConfiguration("robot_name").perform(context)
    template = os.path.join(get_package_share_directory("armada_ros"), "rviz", "armada.rviz")
    with open(template, encoding="utf-8") as f:
        config = f.read().replace("{ns}", name)
    path = os.path.join(tempfile.gettempdir(), f"armada_view_{name}.rviz")
    with open(path, "w", encoding="utf-8") as f:
        f.write(config)
    return [Node(package="rviz2", executable="rviz2", name=f"rviz_{name}", arguments=["-d", path],
                 parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}], output="log")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="armada1"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_rviz),
        ]
    )
