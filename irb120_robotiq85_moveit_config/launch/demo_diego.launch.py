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
        'db', default_value='false', description='Do not start the database by default'
    )
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=os.path.join(get_package_share_directory('irb120_robotiq85_moveit_config'),
                                     'default_warehouse_mongo_db'),
        description='Path for the MoveIt warehouse database'
    )
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false', description='Enable debug mode'
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='false', description='Show joint_state_publisher GUI'
    )

    # Launch configurations
    db = LaunchConfiguration('db')
    db_path = LaunchConfiguration('db_path')
    debug = LaunchConfiguration('debug')
    use_gui = LaunchConfiguration('use_gui')

    # Get package directories
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # ------------------ Load Robot Description and Planning Context ------------------
    # Include the planning_context launch file to load URDF, SRDF, and YAML configurations on the parameter server.
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'planning_context.launch.py')
        ),
        launch_arguments={'load_robot_description': 'true'}.items()
    )

    # ------------------ Joint State Publisher ------------------
    # Launch joint_state_publisher with the argument for using GUI.
    joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': use_gui,
                     'source_list': ['move_group/fake_controller_joint_states']}]
    )

    # ------------------ Robot State Publisher ------------------
    # Launch robot_state_publisher to broadcast TF transforms
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # The robot description parameter is assumed to have been loaded by planning_context.
        parameters=[]
    )

    # ------------------ Move Group (MoveIt!) ------------------
    # Include the move_group launch file with appropriate arguments.
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'allow_trajectory_execution': 'true',
            'fake_execution': 'true',
            'info': 'true',
            'debug': debug
        }.items()
    )

    # ------------------ MoveIt RViz ------------------
    # Include the moveit_rviz launch file with configuration enabled.
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={
            'config': 'true',
            'debug': debug
        }.items()
    )

    # ------------------ Warehouse Database ------------------
    # Conditionally include the warehouse database launch if 'db' is true.
    warehouse_db = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'default_warehouse_db.launch.py')
        ),
        launch_arguments={'moveit_warehouse_database_path': db_path}.items(),
        condition=IfCondition(db)
    )

    # Group all actions together for easier management (if needed, remappings can be added here)
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