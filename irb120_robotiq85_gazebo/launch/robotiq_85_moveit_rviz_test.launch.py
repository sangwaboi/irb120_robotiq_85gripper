#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Timer for the first launch file: after 1 second, launch robotiq_85.launch from robotiq_85_gazebo package
    robotiq_85_gazebo_bringup = TimerAction(
        period=1.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robotiq_85_gazebo'),
                             'launch', 'robotiq_85.launch.py')
            )
        )]
    )
    
    # Timer for the second launch file: after 8 seconds, launch robotiq_85_moveit_planning_execution.launch with argument sim:=true
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
    
    # Timer for the third launch file: after 15 seconds, launch moveit_rviz.launch with argument config:=true
    moveit_rviz_bringup = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robotiq_85_moveit_config'),
                             'launch', 'moveit_rviz.launch.py')
            ),
            launch_arguments={'config': 'true'}.items()
        )]
    )
    
    return LaunchDescription([
        robotiq_85_gazebo_bringup,
        robotiq_85_moveit_bringup,
        moveit_rviz_bringup
    ])