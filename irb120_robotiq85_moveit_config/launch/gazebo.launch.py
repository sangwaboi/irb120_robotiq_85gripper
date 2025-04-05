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
    paused_arg = DeclareLaunchArgument(
        'paused', default_value='false',
        description='Start Gazebo in paused mode')
    gazebo_gui_arg = DeclareLaunchArgument(
        'gazebo_gui', default_value='true',
        description='Enable Gazebo GUI')
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=os.path.join(
            get_package_share_directory('irb120_camara_diego'),
            'urdf', 'irb120_robotiq85_macro.xacro'
        ),
        description='Path to the robot xacro file'
    )

    paused = LaunchConfiguration('paused')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    urdf_path = LaunchConfiguration('urdf_path')

    # Get package directories
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # ------------------- Gazebo Simulator -------------------
    # Include the ROS2 version of the empty world launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'world_name': os.path.join(moveit_config_share, 'worlds', 'empty.world'),
            'paused': paused,
            'gui': gazebo_gui
        }.items()
    )

    # ------------------- Robot Description -------------------
    # Generate the robot description by processing the xacro file.
    # (This replaces the ROS1 <param name="robot_description" textfile="..."/> tag.)
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = {'robot_description': robot_description_content}

    # ------------------- Spawn the Robot Model -------------------
    # Spawn the robot in Gazebo using the robot_description parameter.
    spawn_model = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_model',
        name='spawn_gazebo_model',
        output='screen',
        parameters=[robot_description],
        arguments=['-urdf', '-param', 'robot_description', '-model', 'robot', '-x', '0', '-y', '0', '-z', '0']
    )

    # ------------------- Controllers -------------------
    # Include the ros_controllers launch file from the MoveIt configuration package.
    # (Assumes that ros_controllers.launch has been converted to ROS2 as ros_controllers.launch.py.)
    ros_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'ros_controllers.launch.py')
        )
    )

    return LaunchDescription([
        paused_arg,
        gazebo_gui_arg,
        urdf_path_arg,
        gazebo_launch,
        spawn_model,
        ros_controllers_launch
    ])