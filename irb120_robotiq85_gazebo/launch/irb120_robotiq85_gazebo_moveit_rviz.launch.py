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
    db_arg = DeclareLaunchArgument(
        'db', default_value='false', description='Enable warehouse database launch')
    db_path_arg = DeclareLaunchArgument(
        'db_path', default_value=os.path.join(
            get_package_share_directory('abb_irb120_moveit_config'),
            'default_warehouse_mongo_db'),
        description='Path for the MoveIt warehouse database')

    paused = LaunchConfiguration('paused')
    db = LaunchConfiguration('db')
    db_path = LaunchConfiguration('db_path')

    # Global remappings for controller topics (to mimic ROS-I conventions)
    global_remappings = [
        ("/arm_controller/follow_joint_trajectory", "/joint_trajectory_action"),
        ("/arm_controller/state", "/feedback_states"),
        ("/arm_controller/command", "/joint_path_command")
    ]

    # Get package directories
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    abb_irb120_gazebo_share = get_package_share_directory('abb_irb120_gazebo')
    abb_irb120_support_share = get_package_share_directory('abb_irb120_support')
    abb_irb120_moveit_config_share = get_package_share_directory('abb_irb120_moveit_config')

    # --------------------- PART 1: SIMULATION SETUP ---------------------

    # Include Gazebo empty world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'world_name': os.path.join(abb_irb120_gazebo_share, 'worlds', 'empty.world'),
            'gui': 'true',
            'paused': paused
        }.items()
    )

    # Include the robot description loader from abb_irb120_gazebo
    load_robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(abb_irb120_gazebo_share, 'launch', 'load_irb120_3_58.launch.py')
        )
    )

    # Spawn the robot model in Gazebo
    spawn_robot = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_model',
        output='screen',
        arguments=['-urdf', '-param', 'robot_description', '-model', 'abb_irb120_3_58']
    )

    # Launch robot_state_publisher to broadcast TF transforms
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen'
    )

    # Include the control interface launch file (ros_control interface)
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(abb_irb120_gazebo_share, 'launch', 'irb120_3_58_control.launch.py')
        )
    )

    # --------------------- PART 2: MOVEIT! INTEGRATION ---------------------

    # Load joint names parameters for MoveIt (simulate ROS1 <rosparam load>)
    # Here we use ExecuteProcess to mimic parameter loading for a target node (e.g. /move_group)
    joint_names_params = os.path.join(abb_irb120_support_share, 'config', 'joint_names_irb120_3_58.yaml')
    load_joint_names = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/move_group', joint_names_params],
        output='screen'
    )

    # Include planning_context.launch with load_robot_description set to false
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(abb_irb120_moveit_config_share, 'launch', 'planning_context.launch.py')
        ),
        launch_arguments={'load_robot_description': 'false'}.items()
    )

    # Include move_group.launch with publish_monitored_planning_scene true
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(abb_irb120_moveit_config_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'publish_monitored_planning_scene': 'true'}.items()
    )

    # Include moveit_rviz.launch with config true
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(abb_irb120_moveit_config_share, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={'config': 'true'}.items()
    )

    # Conditionally include default_warehouse_db.launch if db is true
    warehouse_db = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(abb_irb120_moveit_config_share, 'launch', 'default_warehouse_db.launch.py')
        ),
        launch_arguments={'moveit_warehouse_database_path': db_path}.items()
    )

    # Group all nodes and included launches so that the global remappings apply
    group = GroupAction(
        actions=[
            gazebo_launch,
            load_robot_description,
            spawn_robot,
            robot_state_publisher,
            control_launch,
            load_joint_names,
            planning_context,
            move_group,
            moveit_rviz,
            warehouse_db
        ],
        remappings=global_remappings
    )

    return LaunchDescription([
        paused_arg,
        db_arg,
        db_path_arg,
        group
    ])