# armada_simulation

Gazebo (multi-robot) simulation of ARMADA. Each robot exposes **the same ROS interface as the real one** (see
[armada_ros](../armada_ros/README.md#the-interface)): what you write here works on the real robot once
`armada_esp_micro_ros.ino` is flashed.

Meant for ROS 2 **Jazzy** + Gazebo **Harmonic** (Ubuntu 24.04), not tested yet. Not native Windows: use WSL2,
Docker or Linux.

## Install

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-rqt-robot-steering
cd ~/ros2_ws && colcon build --symlink-install --packages-up-to armada_simulation && source install/setup.bash
```

## Launch

```bash
ros2 launch armada_simulation simulation.launch.py                         # 2 robots: armada1, armada2
ros2 launch armada_simulation simulation.launch.py robots:=armada1,armada2,armada3
ros2 launch armada_simulation simulation.launch.py robots:=armada1:-1.5:0:0,armada2:1:1:3.14   # name:x:y:yaw
ros2 launch armada_simulation simulation.launch.py gui:=false              # no window
```

Add a robot to a running simulation:

```bash
ros2 launch armada_simulation spawn_robot.launch.py robot_name:=armada3 x:=0.5 y:=0.5
```

## Drive

Exactly like the real robot, one robot at a time thanks to the namespace. Commands must arrive **continuously**
(the robot stops after 0.4 s without a message, like the firmware):

```bash
ros2 run rqt_robot_steering rqt_robot_steering     # in the window, topic: /armada1/cmd_vel
ros2 topic pub -r 10 /armada2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.5}}"
```

`teleop_twist_keyboard` only publishes on each key press: the robot stops between the first press and the keyboard's
auto-repeat, so driving is jerky.

See the sensors: `ros2 launch armada_ros view.launch.py robot_name:=armada1 use_sim_time:=true`

## What is simulated

| Real                                                                       | Simulation                                                     |
| -------------------------------------------------------------------------- | -------------------------------------------------------------- |
| 2 STS3215 servos                                                           | `DiffDrive` plugin, URDF geometry                              |
| firmware limits (60 rpm per wheel, stop after 0.4 s without a command)     | `cmd_vel_filter` node                                          |
| 8 VL53L0X                                                                  | 1 `gpu_lidar` with 8 beams at the ring centre, 5 mm Gaussian noise |
| HC-SR04 floor sensor                                                       | 1-beam `gpu_lidar` + `scan_to_range` node                      |
| BNO055 (planned)                                                           | `imu` sensor                                                   |

Per robot, in `/<robot>/`: `cmd_vel`, `odom`, `scan`, `floor/range`, `imu/data`, `joint_states`,
`robot_description`; plus the global `/tf` and `/clock`. The `gz/*` topics are internal (Gazebo bridge).
Robots are placed in the `world` frame by a static `world → <robot>/odom` TF.
