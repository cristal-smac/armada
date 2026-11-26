"""Ajoute UN robot ARMADA dans une simulation Gazebo déjà lancée (simulation.launch.py l'appelle pour chaque robot).

    ros2 launch armada_simulation spawn_robot.launch.py robot_name:=armada3 x:=1.0 y:=0.5 yaw:=1.57

Côté ROS, le robot expose la même interface que le robot réel (voir armada_ros/README.md), dans /<robot_name>/ :
    cmd_vel, odom, scan, floor/range, imu/data, joint_states, robot_description (+ /tf global, frames <robot_name>/...)
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _spawn(context):
    name = LaunchConfiguration("robot_name").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    # URDF + plugins Gazebo pour CE robot. Les meshes package:// sont remplacés par des chemins absolus,
    # pour ne dépendre d'aucune variable d'environnement de Gazebo.
    xacro_file = os.path.join(get_package_share_directory("armada_simulation"), "urdf", "armada_gazebo.urdf.xacro")
    description_share = get_package_share_directory("armada_description")
    urdf = xacro.process_file(xacro_file, mappings={"robot_name": name}).toxml()
    urdf = urdf.replace("package://armada_description/", "file://" + description_share + "/")

    # Les topics Gazebo portent les mêmes noms que les topics ROS : /<name>/...
    bridge_args = [
        f"/{name}/gz/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        f"/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        f"/{name}/gz/floor_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        f"/{name}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
    ]

    return [
        # Même bringup que pour le robot réel : robot_state_publisher + odom_adapter, dans /<name>
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("armada_ros"), "launch", "robot.launch.py"])),
            launch_arguments={"robot_name": name, "use_sim_time": "true"}.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{name}",
            arguments=["-name", name, "-string", urdf, "-x", x, "-y", y, "-z", z, "-Y", yaw],
            output="screen",
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name=f"bridge_{name}",
             arguments=bridge_args, output="screen"),
        # Comportement du firmware : limite des roues, arrêt sans commande
        Node(package="armada_simulation", executable="cmd_vel_filter", namespace=name,
             parameters=[{"use_sim_time": True}]),
        # HC-SR04 : LaserScan 1 rayon -> Range
        Node(package="armada_simulation", executable="scan_to_range", namespace=name,
             parameters=[{"use_sim_time": True}]),
        # L'odométrie part de 0 à l'endroit du spawn : on la place dans le repère commun "world"
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"world_to_{name}_odom",
            arguments=["--x", x, "--y", y, "--z", "0", "--yaw", yaw, "--pitch", "0", "--roll", "0",
                       "--frame-id", "world", "--child-frame-id", f"{name}/odom"],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="armada1", description="namespace, unique par robot"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.02", description="léger jeu : le robot se pose au sol"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            OpaqueFunction(function=_spawn),
        ]
    )
