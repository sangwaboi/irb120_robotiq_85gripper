#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    planning_plugin_arg = DeclareLaunchArgument(
        'planning_plugin',
        default_value='ompl_interface/OMPLPlanner',
        description='OMPL planner plugin'
    )
    planning_adapters_arg = DeclareLaunchArgument(
        'planning_adapters',
        default_value='default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
        description='Request adapters for planning'
    )
    start_state_max_bounds_error_arg = DeclareLaunchArgument(
        'start_state_max_bounds_error',
        default_value='0.1',
        description='Maximum bounds error for start state'
    )
    
    # Launch configurations
    planning_plugin = LaunchConfiguration('planning_plugin')
    planning_adapters = LaunchConfiguration('planning_adapters')
    start_state_max_bounds_error = LaunchConfiguration('start_state_max_bounds_error')
    
    # Get the path to the OMPL planning YAML file
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')
    ompl_yaml = os.path.join(moveit_config_share, 'config', 'ompl_planning.yaml')
    
    # Use ExecuteProcess actions to set parameters on the /move_group node.
    # Delay these actions to ensure /move_group is up.
    set_planning_plugin = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/move_group', 'planning_plugin', planning_plugin],
        output='screen'
    )
    set_planning_adapters = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/move_group', 'request_adapters', planning_adapters],
        output='screen'
    )
    set_start_state_error = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/move_group', 'start_state_max_bounds_error', start_state_max_bounds_error],
        output='screen'
    )
    
    load_ompl_yaml = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/move_group', ompl_yaml],
        output='screen'
    )
    
    delayed_set_planning_plugin = TimerAction(
        period=2.0,
        actions=[set_planning_plugin]
    )
    delayed_set_planning_adapters = TimerAction(
        period=2.2,
        actions=[set_planning_adapters]
    )
    delayed_set_start_state_error = TimerAction(
        period=2.4,
        actions=[set_start_state_error]
    )
    delayed_load_yaml = TimerAction(
        period=2.6,
        actions=[load_ompl_yaml]
    )
    
    return LaunchDescription([
        planning_plugin_arg,
        planning_adapters_arg,
        start_state_max_bounds_error_arg,
        delayed_set_planning_plugin,
        delayed_set_planning_adapters,
        delayed_set_start_state_error,
        delayed_load_yaml
    ])

if __name__ == '__main__':
    generate_launch_description()