from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo with an empty world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '--world', 'empty.world'],
            output='screen'
        ),

        # Spawn your robot model in the empty world
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'acs_robot',  # Name of the robot
                '-file', '/home/ashy/quadruped_robot_ROS2/src/robot_simulation/robot_description/robot/robot.sdf',  # Path to your URDF file
                '-x', '0', '-y', '0', '-z', '0.5',  # Initial spawn position
                '--ros-args', '--log-level', 'debug'
            ],
            output='screen'
        ),

        # Start joystick input node
        Node(
            package="joy",
            executable="joy_node",
            output='screen'
        ),

        # Start the custom joystick handler node
        Node(
            package='robot_control',
            executable='ramped_joypad.py',
            output='screen'
        ),

        # Start the robot controller node for Gazebo
        Node(
            package='robot_control',
            executable='robot_controller_gazebo.py',
            output='screen'
        ),
    ])

