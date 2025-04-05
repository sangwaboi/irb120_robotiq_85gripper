#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the 'pipeline' launch argument with a default value of "ompl"
    pipeline_arg = DeclareLaunchArgument(
        'pipeline',
        default_value='ompl',
        description='Planning pipeline to use (e.g., "ompl", "chomp", etc.)'
    )
    
    pipeline = LaunchConfiguration('pipeline')
    
    # Get the package share directory for the MoveIt configuration package
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')
    
    # Construct the path to the planning pipeline launch file.
    # Here we assume the planning pipeline launch files have been converted to ROS2 and have the extension ".launch.py".
    planning_pipeline_launch_file = PathJoinSubstitution(
        [moveit_config_share, 'launch', pipeline, TextSubstitution(text='_planning_pipeline.launch.py')]
    )
    
    # Include the planning pipeline launch file
    include_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planning_pipeline_launch_file)
    )
    
    return LaunchDescription([
        pipeline_arg,
        include_pipeline
    ])

if __name__ == '__main__':
    generate_launch_description()