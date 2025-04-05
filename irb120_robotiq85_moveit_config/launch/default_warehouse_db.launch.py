#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    reset_arg = DeclareLaunchArgument(
        'reset',
        default_value='false',
        description='Reset the warehouse database'
    )
    db_path_arg = DeclareLaunchArgument(
        'moveit_warehouse_database_path',
        default_value=os.path.join(get_package_share_directory('irb120_robotiq85_moveit_config'),
                                     'default_warehouse_mongo_db'),
        description='Path for the MoveIt warehouse database'
    )

    # Launch configurations
    reset = LaunchConfiguration('reset')
    db_path = LaunchConfiguration('moveit_warehouse_database_path')

    # Get the moveit configuration package share directory
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # Include the warehouse launch file (assumes a ROS2-converted version exists as warehouse.launch.py)
    warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'warehouse.launch.py')
        ),
        launch_arguments={'moveit_warehouse_database_path': db_path}.items()
    )

    # Conditionally launch the reset node if reset is true
    reset_node = launch_ros.actions.Node(
        package='moveit_ros_warehouse',
        executable='moveit_init_demo_warehouse',
        name='moveit_default_db_reset',
        output='screen',
        condition=IfCondition(reset)
    )

    return LaunchDescription([
        reset_arg,
        db_path_arg,
        warehouse_launch,
        reset_node
    ])