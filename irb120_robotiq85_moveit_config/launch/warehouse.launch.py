#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch argument for the warehouse database path
    moveit_warehouse_database_path_arg = DeclareLaunchArgument(
        'moveit_warehouse_database_path',
        default_value='',
        description='Path to the MoveIt warehouse database'
    )
    moveit_warehouse_database_path = LaunchConfiguration('moveit_warehouse_database_path')
    
    # Include the warehouse settings launch file (assumed to be converted to ROS2)
    warehouse_settings_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('irb120_robotiq85_moveit_config'),
                'launch', 'warehouse_settings.launch.py'
            )
        )
    )
    
    # Launch the DB server node from the warehouse_ros_mongo package
    db_server_node = launch_ros.actions.Node(
        package='warehouse_ros_mongo',
        executable='mongo_wrapper_ros.py',
        name='mongo_wrapper_ros',
        cwd=os.environ.get('ROS_HOME', ''),  # Set working directory if ROS_HOME is defined
        output='screen',
        parameters=[{
            'overwrite': False,
            'database_path': moveit_warehouse_database_path
        }]
    )
    
    return LaunchDescription([
        moveit_warehouse_database_path_arg,
        warehouse_settings_launch,
        db_server_node
    ])

if __name__ == '__main__':
    generate_launch_description()