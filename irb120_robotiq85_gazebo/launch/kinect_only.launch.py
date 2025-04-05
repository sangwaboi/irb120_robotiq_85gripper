#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    paused_arg = DeclareLaunchArgument(
        'paused', default_value='false', description='Start Gazebo in paused mode')
    camera_name_arg = DeclareLaunchArgument(
        'camera_name', default_value='kinect_camera', description='Name of the Kinect camera')
    
    paused = LaunchConfiguration('paused')
    camera_name = LaunchConfiguration('camera_name')
    
    # Get package directories
    pkg_share = get_package_share_directory('irb120_robotiq85_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    # Generate robot description from xacro (processing kinect.urdf.xacro)
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_share, 'urdf', 'kinect.urdf.xacro')
    ])
    robot_description = {'robot_description': robot_description_content}
    
    # Include the Gazebo empty world launch from gazebo_ros
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'world_name': os.path.join(pkg_share, 'worlds', 'empty.world'),
            'gui': 'true',
            'paused': paused,
            'use_sim_time': 'true'
        }.items()
    )
    
    # Spawn the Kinect camera model in Gazebo using spawn_model
    spawn_model = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_model',
        name=[camera_name, '_spawn_urdf'],  # Node name concatenates camera_name and suffix
        output='screen',
        arguments=[
            '-urdf', '-param', 'robot_description', '-model', camera_name
        ]
    )
    
    # Launch robot_state_publisher to broadcast TF transforms
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Launch RViz2 using the provided RViz configuration
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'kinect_only.rviz')
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        paused_arg,
        camera_name_arg,
        gazebo_launch,
        spawn_model,
        robot_state_publisher,
        rviz_node
    ])