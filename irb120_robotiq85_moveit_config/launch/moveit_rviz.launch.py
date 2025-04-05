#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory for the MoveIt configuration package
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # Declare launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='false',
        description='Enable config mode for RViz'
    )
    launch_prefix_arg = DeclareLaunchArgument(
        'launch_prefix',
        default_value='',
        description='Launch prefix (e.g. "gdb --ex run --args" if debug is true)'
    )
    command_args_arg = DeclareLaunchArgument(
        'command_args',
        default_value='',
        description='Command line arguments for RViz (e.g. "-d <path>" if config is true)'
    )

    debug = LaunchConfiguration('debug')
    config = LaunchConfiguration('config')
    launch_prefix = LaunchConfiguration('launch_prefix')
    command_args = LaunchConfiguration('command_args')

    # Get the path to the kinematics YAML file (to be loaded as parameters)
    kinematics_yaml = os.path.join(moveit_config_share, 'config', 'kinematics.yaml')

    # Define the RViz2 node.
    # - Package: "rviz2" (the ROS2 version)
    # - Executable: "rviz2"
    # - The node's prefix is set by the launch_prefix argument.
    # - The arguments (if any) are provided by command_args.
    # - The parameters are loaded from the kinematics YAML file.
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        prefix=launch_prefix,
        arguments=[command_args],
        output='screen',
        parameters=[kinematics_yaml]
    )

    return LaunchDescription([
        debug_arg,
        config_arg,
        launch_prefix_arg,
        command_args_arg,
        rviz_node
    ])