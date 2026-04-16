# TidyBot

A home-tidying mobile robot simulation built for the
[Drift](https://godrift.ai/) take-home assignment. ROS 2 Humble +
Gazebo Classic 11.

The robot is a 4-wheeled skid-steer base with a torso, a 3-DOF arm, and
a sensor suite (2D LiDAR, RGB-D camera, IMU, front-facing ground
camera). It lives in a two-room home and navigates autonomously using
Nav2 with a pre-built map and AMCL localization.

---

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (desktop install)

---

## One-time setup

```bash
# ROS 2 + Gazebo + tooling
sudo apt install -y \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-localization \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard

# Python support for colcon builds
pip install catkin_pkg empy==3.3.4 lark
```

Clone and build:

```bash
git clone https://github.com/JueeChandrachud/Tidybot.git ~/DriftAI
cd ~/DriftAI
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## Run the full navigation stack

```bash
cd ~/DriftAI
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tidybot_description navigation.launch.py
```

This brings up, in sequence:
- Gazebo server + client with the two-room home world
- The robot (spawned at the collection box in Room A)
- `robot_state_publisher` and `joint_state_publisher` for TF
- `robot_localization` EKF fusing wheel odometry + IMU
- Nav2 with the saved map (`maps/home.yaml`) + AMCL particle filter
- RViz pre-configured to show map, scan, plan, and costmaps
- Auto-seeds AMCL's initial pose after ~22 seconds

Wait until the RViz window shows the map and the robot's laser scan
(usually around 25 seconds after launch).

---

## Send the robot to the pink box

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
cd ~/DriftAI

python3 scripts/send_goal.py pink
```

The robot plans a path from Room A through the doorway to Room B and
stops ~40 cm west of the pink (purple) cube, facing it. Watch the
path in RViz and the physical motion in Gazebo.

### Other goals

```bash
python3 scripts/send_goal.py home        # back to spawn
```

Full preset list is in `scripts/send_goal.py`.

---

## Stopping

Ctrl-C in the launch terminal. If Gazebo is stubborn:

```bash
pkill -9 -f gzserver
pkill -9 -f gzclient
```
