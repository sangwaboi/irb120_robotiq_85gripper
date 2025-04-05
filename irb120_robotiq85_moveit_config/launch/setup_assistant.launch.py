#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros.actions

def generate_launch_description():
    # Declare the 'debug' launch argument.
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    # LaunchConfiguration for debug flag.
    debug = LaunchConfiguration('debug')
    
    # Determine the launch prefix based on the debug flag.
    # If debug is "true", prefix the node launch with "gdb --ex run --args"; otherwise, no prefix.
    launch_prefix = PythonExpression(
         ["'gdb --ex run --args' if '", debug, "' == 'true' else ''"]
    )
    
    # Launch the MoveIt! Setup Assistant node with the given configuration package.
    moveit_setup_assistant_node = launch_ros.actions.Node(
        package='moveit_setup_assistant',
        executable='moveit_setup_assistant',
        name='moveit_setup_assistant',
        arguments=["--config_pkg=irb120_robotiq85_moveit_config"],
        prefix=launch_prefix,
        output='screen'
    )
    
    return LaunchDescription([
        debug_arg,
        moveit_setup_assistant_node
    ])

if __name__ == '__main__':
    generate_launch_description()