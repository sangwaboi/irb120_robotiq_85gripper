#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    db_arg = DeclareLaunchArgument(
        'db', default_value='false',
        description='Whether to start the warehouse database (default: false)'
    )
    db_path_arg = DeclareLaunchArgument(
        'db_path', default_value=os.path.join(
            get_package_share_directory('irb120_robotiq85_moveit_config'),
            'default_warehouse_mongo_db'
        ),
        description='Path for the MoveIt warehouse database'
    )
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Enable debug mode'
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='false',
        description='Enable joint_state_publisher GUI (default: false)'
    )

    # Launch configurations
    db = LaunchConfiguration('db')
    db_path = LaunchConfiguration('db_path')
    debug = LaunchConfiguration('debug')
    use_gui = LaunchConfiguration('use_gui')

    # Get package share directory for MoveIt configuration
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # ------------------- LOAD PLANNING CONTEXT -------------------
    # This launch file loads the URDF, SRDF, and YAML configuration parameters
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'planning_context.launch.py')
        ),
        launch_arguments={'load_robot_description': 'true'}.items()
    )

    # ------------------- JOINT STATE PUBLISHER -------------------
    # Launch the joint_state_publisher; it publishes fake joint states, optionally with its GUI
    joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': use_gui,
                     'source_list': ['/joint_states']}]
    )

    # ------------------- ROBOT STATE PUBLISHER -------------------
    # Launch robot_state_publisher to broadcast TF transforms (the robot_description is loaded by planning_context)
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen'
    )

    # ------------------- MOVEIT! NODES -------------------
    # Include the move_group launch file to run the MoveIt! planning node
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'allow_trajectory_execution': 'true',
            'fake_execution': 'false',
            'info': 'true',
            'debug': debug
        }.items()
    )

    # Include the MoveIt! RViz launch file to visualize the planning scene
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={
            'config': 'true',
            'debug': debug
        }.items()
    )

    # ------------------- WAREHOUSE DATABASE (OPTIONAL) -------------------
    # Conditionally launch the warehouse database if 'db' is true
    warehouse_db = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'default_warehouse_db.launch.py')
        ),
        launch_arguments={'moveit_warehouse_database_path': db_path}.items(),
        condition=IfCondition(db)
    )

    # ------------------- GROUPING -------------------
    # Group all the actions together for simplicity
    group = GroupAction(
        actions=[
            planning_context,
            joint_state_publisher,
            robot_state_publisher,
            move_group,
            moveit_rviz,
            warehouse_db
        ]
    )

    return LaunchDescription([
        db_arg,
        db_path_arg,
        debug_arg,
        use_gui_arg,
        group
    ])