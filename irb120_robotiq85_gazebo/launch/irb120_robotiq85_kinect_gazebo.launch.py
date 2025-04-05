#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    paused_arg = DeclareLaunchArgument(
        'paused', default_value='false', description='Start Gazebo in paused mode')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', description='Enable Gazebo GUI')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false', description='Run Gazebo headless')
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false', description='Enable debug mode')

    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')

    # Global remappings for controller topics
    global_remappings = [
        ("/arm_controller/follow_joint_trajectory", "/joint_trajectory_action"),
        ("/arm_controller/state", "/feedback_states"),
        ("/arm_controller/command", "/joint_path_command")
    ]

    # Get package directories
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('irb120_robotiq85_gazebo')

    # Generate robot description by processing the xacro file
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_share, 'urdf', 'irb120_robotiq85_kinect.macro.xacro')
    ])
    robot_description = {'robot_description': robot_description_content}

    # Include Gazebo empty world launch (using empty.world)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'world_name': os.path.join(pkg_share, 'worlds', 'empty.world'),
            'gui': gui,
            'paused': paused,
            'use_sim_time': use_sim_time,
            'headless': headless
        }.items()
    )

    # Spawn the robot model in Gazebo
    spawn_model = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_model',
        arguments=['-urdf', '-param', 'robot_description', '-model', 'irb120_robotiq85_kinect'],
        output='screen'
    )

    # Launch robot state publisher to broadcast TF transforms
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn controllers: joint state, arm, and gripper controllers
    joint_state_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_controller'],
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'joint_state_controller.yaml')]
    )
    arm_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'irb120_arm_controller.yaml')]
    )
    gripper_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'robotiq_gripper_controller.yaml')]
    )

    # Publish a fake calibration message on /calibrated topic
    fake_calibration = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/msg/Bool', 'true', '--once'],
        output='screen'
    )

    # Launch RViz using the provided configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'irb120.rviz')
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Group all nodes so that the global remappings apply to all nodes
    group = GroupAction(
        actions=[
            gazebo_launch,
            spawn_model,
            robot_state_publisher,
            joint_state_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            fake_calibration,
            rviz_node
        ],
        remappings=global_remappings
    )

    return LaunchDescription([
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        group
    ])