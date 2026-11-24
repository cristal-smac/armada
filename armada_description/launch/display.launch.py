"""Affiche le robot dans RViz avec des curseurs pour les roues.

    ros2 launch armada_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("armada_description")
    with open(os.path.join(share, "urdf", "armada.urdf"), encoding="utf-8") as f:
        urdf = f.read()
    return LaunchDescription(
        [
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[{"robot_description": urdf}]),
            Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
            Node(package="rviz2", executable="rviz2"),
        ]
    )
