"""Côté PC d'UN robot ARMADA (réel ou simulé) : description + TF + joint_states, dans le namespace du robot.

    ros2 launch armada_ros robot.launch.py robot_name:=armada1

Lancé à l'identique pour le robot réel (via real.launch.py) et simulé (via armada_simulation), c'est ce qui garantit
que les deux se pilotent pareil. Il ne démarre PAS la source des topics (micro-ROS agent ou Gazebo).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    with open(os.path.join(get_package_share_directory("armada_description"), "urdf", "armada.urdf"),
              encoding="utf-8") as f:
        urdf = f.read()

    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="armada1", description="namespace du robot"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=robot_name,
                parameters=[{
                    "robot_description": urdf,
                    # frames <robot_name>/base_link... : /tf est global, il faut des noms uniques par robot
                    "frame_prefix": ParameterValue([robot_name, "/"], value_type=str),
                    "use_sim_time": use_sim_time,
                }],
            ),
            Node(
                package="armada_ros",
                executable="odom_adapter",
                namespace=robot_name,
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
