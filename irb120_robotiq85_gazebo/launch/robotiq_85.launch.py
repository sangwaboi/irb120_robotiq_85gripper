#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    limited_arg = DeclareLaunchArgument(
        'limited', default_value='false', description='Limited mode')
    paused_arg = DeclareLaunchArgument(
        'paused', default_value='false', description='Start Gazebo in paused mode')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', description='Enable Gazebo GUI')

    limited = LaunchConfiguration('limited')
    paused = LaunchConfiguration('paused')
    gui = LaunchConfiguration('gui')

    # Get package directories
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('robotiq_85_gazebo')
    gripper_desc_share = get_package_share_directory('robotiq_85_description')

    # Include the Gazebo empty world launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'world_name': os.path.join(pkg_share, 'worlds', 'empty.world'),
            'paused': paused,
            'gui': gui
        }.items()
    )

    # Generate robot description from the xacro file in robotiq_85_description
    robot_description_content = Command([
        'xacro ',
        os.path.join(gripper_desc_share, 'urdf', 'robotiq_85_gripper.urdf.xacro')
    ])
    robot_description = {'robot_description': robot_description_content}

    # Launch robot_state_publisher to broadcast TF transforms
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn the robot model in Gazebo (using the robot_description parameter)
    spawn_model = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_model',
        output='screen',
        arguments=[
            '-urdf', '-param', 'robot_description', '-model', 'robot'
        ]
    )

    # Load gripper controller parameters from YAML and spawn the gripper controller
    controller_params = os.path.join(pkg_share, 'controller', 'gripper_controller_robotiq.yaml')
    gripper_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper'],
        output='screen',
        parameters=[controller_params]
    )

    return LaunchDescription([
        limited_arg,
        paused_arg,
        gui_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_model,
        gripper_controller_spawner
    ])