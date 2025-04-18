from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package = "joy",
             executable = "joy_node"
            ),
        Node(package='robot_control',
             executable='ramped_joypad.py',
             output='screen'
             ),
        Node(package='robot_control',
             executable='robot_controller_gazebo.py',
             output='screen'
             ),
    ])