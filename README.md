# Joint Mover Package

ROS 2 package for controlling joint movements on Fanuc robots.

## Description
Moves J2 joint in a sine wave pattern with 15-degree amplitude and sets a DO 1 to True.

## Dependencies
- rclpy
- trajectory_msgs
- fanuc_msgs

## Installation
```bash
cd ~/ws_fanuc/src
git clone https://github.com/jatinmayekar/fanuc_joint_mover.git
cd ..
colcon build --packages-select joint_mover
source install/setup.bash
```

## Usage - single J2 joint motion
```bash
ros2 launch joint_mover j2_mover.launch.py
```

## Usage - cyclic J2 joint motion
```bash
ros2 launch joint_mover j2_mover_cont.launch.py
```

