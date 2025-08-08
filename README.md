# Joint Mover Package

ROS 2 package for controlling joint movements on Fanuc robots.

## Dependencies
- rclpy
- trajectory_msgs
- fanuc_msgs

## Installation
```bash
cd ~/your_ws/src
git clone https://github.com/jatinmayekar/fanuc_joint_mover.git
cd ..
colcon build --packages-select joint_mover
source install/setup.bash
```

## Usage
```bash
ros2 launch joint_mover j2_mover.launch.py
```

## Description
Moves J2 joint in a sine wave pattern with 15-degree amplitude.