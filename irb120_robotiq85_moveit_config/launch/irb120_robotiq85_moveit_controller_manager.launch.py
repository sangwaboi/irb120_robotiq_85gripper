#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the moveit_controller_manager argument with a default value
    controller_manager_arg = DeclareLaunchArgument(
        'moveit_controller_manager',
        default_value='moveit_simple_controller_manager/MoveItSimpleControllerManager',
        description='Controller manager plugin to use'
    )
    moveit_controller_manager = LaunchConfiguration('moveit_controller_manager')

    # Get the path to the ros_controllers.yaml file
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')
    ros_controllers_yaml = os.path.join(moveit_config_share, 'config', 'ros_controllers.yaml')

    # Action to set the 'moveit_controller_manager' parameter on the /move_group node
    set_controller_manager = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/move_group', 'moveit_controller_manager', moveit_controller_manager],
        output='screen'
    )

    # Action to load additional parameters from ros_controllers.yaml into the /move_group node
    load_ros_controllers = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/move_group', ros_controllers_yaml],
        output='screen'
    )

    # Use TimerActions to delay these commands so that /move_group has time to start
    delayed_set_controller_manager = TimerAction(
        period=2.0,
        actions=[set_controller_manager]
    )
    delayed_load_ros_controllers = TimerAction(
        period=2.5,
        actions=[load_ros_controllers]
    )

    return LaunchDescription([
        controller_manager_arg,
        delayed_set_controller_manager,
        delayed_load_ros_controllers
    ])