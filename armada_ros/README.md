# armada_ros

ROS 2 control of ARMADA from a PC, **the same for the real robot and for the simulation**.

```
 PC (ROS 2 Jazzy)                                   Topic source
 ┌──────────────────────────────┐        ┌─ real robot: ESP32 + armada_esp_micro_ros.ino ── Wi-Fi ── micro-ROS agent
 │ teleop / navigation / your   │ ◄────► │
 │ programs  +  armada_ros      │        └─ simulation: armada_simulation (Gazebo)
 └──────────────────────────────┘
```

Your programs only talk to the interface below. Switching from the simulation to the robot means changing where the
topics come from, nothing else.

## The interface

Each robot lives in its own **namespace** (`armada1`, `armada2`...). Everything is relative to `/<robot>/`.

| Topic               | Type                     | Direction | Details                                                                                                                       |
| ------------------- | ------------------------ | --------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `cmd_vel`           | `geometry_msgs/Twist`    | → robot   | `linear.x` m/s, `angular.z` rad/s. Stops by itself after 0.4 s without a command. Wheels limited to 60 rpm                    |
| `odom`              | `nav_msgs/Odometry`      | robot →   | pose + velocity, frame `<robot>/odom` → `<robot>/base_footprint`                                                              |
| `scan`              | `sensor_msgs/LaserScan`  | robot →   | ring of 8 ToF: 8 beams at 0°, 45°… 315° (F FL L BL B BR R FR), measured from the ring centre, 0.04–1.2 m, `inf` = nothing     |
| `floor/range`       | `sensor_msgs/Range`      | robot →   | HC-SR04 tilted towards the floor, 0.02–0.4 m                                                                                  |
| `imu/data`          | `sensor_msgs/Imu`        | robot →   | simulation only for now (the BNO055 is not in the firmware yet)                                                               |
| `joint_states`      | `sensor_msgs/JointState` | PC →      | made by `odom_adapter` (wheels turning in RViz)                                                                               |
| `robot_description` | `std_msgs/String`        | PC →      | URDF, from `robot_state_publisher`                                                                                            |
| `/tf` (global)      |                          | PC →      | `<robot>/odom → <robot>/base_footprint` (from `odom_adapter`), then the URDF tree with the `<robot>/` prefix                  |

`scan` and `floor/range` are published as _best effort_: subscribe with `qos_profile_sensor_data`
(or `--qos-reliability best_effort` on the command line). `odom` is _reliable_ (too big for best effort in
micro-ROS). `cmd_vel` accepts both.

`cmd_vel` must arrive **continuously** (at least every 0.4 s), otherwise the robot stops.

The PC builds `joint_states` and the TF itself from `odom`: the ESP32 does not have to publish them.

## Install

In a ROS 2 **Jazzy** workspace (Ubuntu 24.04, possibly inside WSL2 or Docker):

```bash
cd ~/ros2_ws/src
git clone <this repository> armada     # or copy/link the repository here
sudo apt install ros-jazzy-rqt-robot-steering ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

The micro-ROS agent (real robot only) is not in apt; build it in the workspace:

```bash
cd ~/ros2_ws/src && git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
cd ~/ros2_ws && rosdep install --from-paths src -y --ignore-src && colcon build --packages-select micro_ros_setup
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh && ros2 run micro_ros_setup build_agent.sh && source install/setup.bash
```

## Real robot

1. Flash `Armada_robot/armada_esp_micro_ros/armada_esp_micro_ros.ino` (see [its header](../Armada_robot/armada_esp_micro_ros/armada_esp_micro_ros.ino)):
   a **different `ROBOT_NAME` for each robot**, your Wi-Fi, the PC's IP address.
2. On the PC (same network, UDP 8888 open):
   ```bash
   ros2 launch armada_ros real.launch.py robots:=armada1,armada2
   ```
   One agent serves all the robots.
3. Drive (in the window, topic `/armada1/cmd_vel`):
   ```bash
   ros2 run rqt_robot_steering rqt_robot_steering
   ```
   Avoid `teleop_twist_keyboard`: it only publishes on each key press, so the robot stops between the first press
   and the keyboard's auto-repeat.
4. Visualise: `ros2 launch armada_ros view.launch.py robot_name:=armada1`

## Simulation

Same commands 3 and 4, but with `use_sim_time:=true` for RViz; the topic source is
[armada_simulation](../armada_simulation/README.md).

## Contents

|                              |                                                                                                       |
| ---------------------------- | ----------------------------------------------------------------------------------------------------- |
| `launch/robot.launch.py`     | PC side of one robot: `robot_state_publisher` (frame prefix) + `odom_adapter`, in its namespace       |
| `launch/real.launch.py`      | micro-ROS agent + `robot.launch.py` for each robot in `robots:=`                                      |
| `launch/view.launch.py`      | RViz set up for one robot                                                                             |
| `armada_ros/odom_adapter.py` | `odom` → TF + `joint_states`                                                                          |

## Multi-robot rules

- one unique name per robot, the same in the firmware (`ROBOT_NAME`) and in `robots:=`;
- `/tf` is global, so frames are prefixed (`armada1/base_link`);
- each robot has its own independent `odom`. To place them in a common frame, publish a `world → <robot>/odom` TF
  (the simulation does it with the start pose; for real robots it is up to you, or to a localisation system).
