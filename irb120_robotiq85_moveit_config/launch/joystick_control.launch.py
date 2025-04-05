#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    # Declare a launch argument for the joystick device
    dev_arg = DeclareLaunchArgument(
        'dev',
        default_value='/dev/input/js0',
        description='Path to the joystick device'
    )

    # Launch the joy_node with the appropriate parameters
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('dev'),
            'deadzone': 0.2,
            'autorepeat_rate': 40,
            'coalesce_interval': 0.025
        }]
    )

    # Launch the MoveIt joystick interface node (moveit_joy)
    moveit_joy_node = launch_ros.actions.Node(
        package='moveit_ros_visualization',
        executable='moveit_joy.py',
        name='moveit_joy',
        output='screen'
    )

    return LaunchDescription([
        dev_arg,
        joy_node,
        moveit_joy_node
    ])

if __name__ == '__main__':
    generate_launch_description()