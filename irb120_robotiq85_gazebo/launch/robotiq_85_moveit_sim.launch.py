#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # After 1 second, launch robotiq_85.launch from the robotiq_85_gazebo package.
    robotiq_85_gazebo_bringup = TimerAction(
        period=1.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robotiq_85_gazebo'),
                             'launch', 'robotiq_85.launch.py')
            )
        )]
    )

    # After 8 seconds, launch robotiq_85_moveit_planning_execution.launch with argument sim:=true
    robotiq_85_moveit_bringup = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robotiq_85_moveit_config'),
                             'launch', 'robotiq_85_moveit_planning_execution.launch.py')
            ),
            launch_arguments={'sim': 'true'}.items()
        )]
    )

    # After 12 seconds, launch test_kinematics.launch from the robotiq_85_gazebo package.
    robotiq_85_kintest_bringup = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robotiq_85_gazebo'),
                             'launch', 'test_kinematics.launch.py')
            )
        )]
    )

    return LaunchDescription([
        robotiq_85_gazebo_bringup,
        robotiq_85_moveit_bringup,
        robotiq_85_kintest_bringup
    ])