#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments with default values
    planning_plugin_arg = DeclareLaunchArgument(
        'planning_plugin',
        default_value='chomp_interface/CHOMPPlanner',
        description='The planning plugin to use'
    )
    start_state_error_arg = DeclareLaunchArgument(
        'start_state_max_bounds_error',
        default_value='0.1',
        description='The start state maximum bounds error'
    )

    planning_plugin = LaunchConfiguration('planning_plugin')
    start_state_max_bounds_error = LaunchConfiguration('start_state_max_bounds_error')

    # Get the path to the chomp planning YAML configuration file
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')
    chomp_yaml_file = os.path.join(moveit_config_share, 'config', 'chomp_planning.yaml')

    # In ROS1, <param> tags were used to set parameters on the parameter server.
    # In ROS2, parameters are associated with nodes. Here, we assume these parameters
    # need to be loaded into the move_group node.
    # Use ExecuteProcess to mimic the ROS1 rosparam load command:
    load_params = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/move_group', chomp_yaml_file],
        output='screen'
    )

    return LaunchDescription([
        planning_plugin_arg,
        start_state_error_arg,
        load_params
    ])