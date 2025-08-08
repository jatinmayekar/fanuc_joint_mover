from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_mover',
            executable='joint_j2_mover.py', 
            name='joint_j2_mover',
            output='screen'
        ),
    ])