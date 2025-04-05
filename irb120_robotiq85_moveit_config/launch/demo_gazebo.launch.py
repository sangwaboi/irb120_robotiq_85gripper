#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    db_arg = DeclareLaunchArgument(
        'db', default_value='false',
        description='Do not start a database by default'
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
        description='Show joint_state_publisher GUI'
    )
    gazebo_gui_arg = DeclareLaunchArgument(
        'gazebo_gui', default_value='true',
        description='Enable Gazebo GUI'
    )
    paused_arg = DeclareLaunchArgument(
        'paused', default_value='false',
        description='Start Gazebo in paused mode'
    )
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path', default_value=os.path.join(
            get_package_share_directory('irb120_camara_diego'),
            'urdf', 'irb120_robotiq85_macro.xacro'
        ),
        description='Path to the robot xacro file'
    )

    # Launch configurations
    db = LaunchConfiguration('db')
    db_path = LaunchConfiguration('db_path')
    debug = LaunchConfiguration('debug')
    use_gui = LaunchConfiguration('use_gui')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    paused = LaunchConfiguration('paused')
    urdf_path = LaunchConfiguration('urdf_path')

    # Get package directories
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # ----------------- LAUNCH GAZEBO SIMULATOR -----------------
    # Include the Gazebo launch file (converted to ROS2) to launch the simulator and spawn the robot.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'paused': paused,
            'gazebo_gui': gazebo_gui,
            'urdf_path': urdf_path
        }.items()
    )

    # ----------------- LOAD PLANNING CONTEXT -----------------
    # Include the planning_context launch file to load URDF, SRDF, and other configuration files on the parameter server.
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'planning_context.launch.py')
        ),
        launch_arguments={'load_robot_description': 'false'}.items()
    )

    # ----------------- JOINT STATE PUBLISHER -----------------
    # Launch the joint_state_publisher node (with GUI optional) to publish fake joint states.
    joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': use_gui,
                     'source_list': ['/joint_states']}]
    )

    # ----------------- ROBOT STATE PUBLISHER -----------------
    # Launch the robot_state_publisher node to broadcast TF transforms
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen'
    )

    # ----------------- MOVEIT! NODES -----------------
    # Include the main MoveIt! executable (move_group) with required arguments
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

    # Include the MoveIt! RViz launch to visualize the move_group state
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={
            'config': 'true',
            'debug': debug
        }.items()
    )

    # ----------------- WAREHOUSE DATABASE (OPTIONAL) -----------------
    # Conditionally include the default warehouse database launch if the db argument is true.
    warehouse_db = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'default_warehouse_db.launch.py')
        ),
        launch_arguments={'moveit_warehouse_database_path': db_path}.items(),
        condition=IfCondition(db)
    )

    # ----------------- GROUPING -----------------
    # Group all the actions together. Global remappings could be added here if needed.
    group = GroupAction(
        actions=[
            gazebo_launch,
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
        gazebo_gui_arg,
        paused_arg,
        urdf_path_arg,
        group
    ])