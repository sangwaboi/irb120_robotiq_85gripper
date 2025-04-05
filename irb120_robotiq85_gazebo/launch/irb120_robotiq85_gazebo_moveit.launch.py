#!/usr/bin/env python3
import os

from launch import LaunchDescription
import launch.actions
import launch.substitutions
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    paused_arg = launch.actions.DeclareLaunchArgument(
        'paused', default_value='false', description='Start Gazebo in paused mode')
    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')
    gui_arg = launch.actions.DeclareLaunchArgument(
        'gui', default_value='true', description='Enable Gazebo GUI')
    headless_arg = launch.actions.DeclareLaunchArgument(
        'headless', default_value='false', description='Run Gazebo headless')
    debug_arg = launch.actions.DeclareLaunchArgument(
        'debug', default_value='false', description='Enable debug mode')

    # Get package directories
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('irb120_robotiq85_gazebo')
    moveit_pkg_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # Robot description parameter: run xacro to generate the URDF
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_share, 'urdf', 'irb120_robotiq85_macro.xacro')
    ])
    robot_description = {'robot_description': robot_description_content}

    # Include Gazebo empty world launch (assumes ROS2 version is available as a Python launch file)
    gazebo_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'world_name': os.path.join(pkg_share, 'worlds', 'empty.world'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    # Spawn the robot model in Gazebo
    spawn_model = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_model',
        arguments=['-urdf', '-param', 'robot_description', '-model', 'irb120_robotiq85'],
        output='screen'
    )

    # Robot state publisher node to publish TF transforms
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Controller spawner nodes (using ROS2 controller_manager's spawner)
    joint_state_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_controller'],
        output='screen'
    )
    arm_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )
    gripper_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    # Fake calibration publisher (simulate calibration message)
    fake_calibration = launch.actions.ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/msg/Bool', 'true'],
        output='screen'
    )

    # Include the MoveIt! group launch file (assumes MoveIt2 launch file is available in the moveit_config package)
    moveit_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'info': 'true'}.items()
    )

    return LaunchDescription([
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        gazebo_launch,
        # Publish robot description and spawn robot
        spawn_model,
        robot_state_publisher,
        # Spawn controllers
        joint_state_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        # Fake calibration publisher
        fake_calibration,
        # Launch MoveIt!
        moveit_launch
    ])