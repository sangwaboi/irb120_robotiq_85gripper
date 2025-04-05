#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    moveit_manage_controllers_arg = DeclareLaunchArgument(
        'moveit_manage_controllers',
        default_value='true',
        description='Flag indicating whether MoveIt! is allowed to manage controllers'
    )
    moveit_controller_manager_arg = DeclareLaunchArgument(
        'moveit_controller_manager',
        default_value='irb120_robotiq85',
        description='Robot-specific controller manager name'
    )

    # Launch configurations
    moveit_manage_controllers = LaunchConfiguration('moveit_manage_controllers')
    moveit_controller_manager = LaunchConfiguration('moveit_controller_manager')

    # Trajectory execution parameters to be used by the move_group node (or merged later)
    trajectory_execution_params = {
        'trajectory_execution/allowed_execution_duration_scaling': 1.2,
        'trajectory_execution/allowed_goal_duration_margin': 0.5,
        'trajectory_execution/allowed_start_tolerance': 0.01,
        'moveit_manage_controllers': moveit_manage_controllers
    }
    # (Note: In ROS2, these parameters need to be merged into the node that uses them.)

    # Get the share directory for your moveit configuration package
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # Construct the path to the controller manager launch file.
    # This file is assumed to be converted to ROS2 and named:
    # <moveit_controller_manager>_moveit_controller_manager.launch.py
    controller_manager_launch_file = PathJoinSubstitution(
        [moveit_config_share, 'launch',
         LaunchConfiguration('moveit_controller_manager'),
         TextSubstitution(text='_moveit_controller_manager.launch.py')]
    )

    controller_manager_launch = GroupAction(
        actions=[
            # Include the controller manager launch file.
            # It is assumed that the included launch file will take care of loading the appropriate
            # controllers and merging any necessary parameters (e.g., trajectory execution settings)
            # into the node that uses them.
            # (You might modify the included launch file to accept additional launch arguments
            # if needed.)
            # Note: We're using PythonLaunchDescriptionSource to include the ROS2 launch file.
            # The substitution constructs the file name based on the moveit_controller_manager argument.
            PythonLaunchDescriptionSource(controller_manager_launch_file)
        ]
    )

    return LaunchDescription([
        moveit_manage_controllers_arg,
        moveit_controller_manager_arg,
        controller_manager_launch
    ])

if __name__ == '__main__':
    generate_launch_description()