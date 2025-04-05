#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare a launch argument for the sensor manager name (default "irb120_robotiq85")
    moveit_sensor_manager_arg = DeclareLaunchArgument(
        'moveit_sensor_manager',
        default_value='irb120_robotiq85',
        description='Robot-specific sensor manager name'
    )
    moveit_sensor_manager = LaunchConfiguration('moveit_sensor_manager')

    # Get the share directory for the MoveIt configuration package
    config_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # Path to the sensors_3d YAML file
    sensors_3d_yaml = os.path.join(config_share, 'config', 'sensors_3d.yaml')

    # Load the 3D sensors parameters into the target node (here we assume /move_group uses them)
    load_sensors_3d = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'load', '/move_group', sensors_3d_yaml],
                output='screen'
            )
        ]
    )

    # Set additional parameters for the octomap monitor:
    set_octomap_resolution = TimerAction(
        period=2.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/move_group', 'octomap_resolution', '0.025'],
                output='screen'
            )
        ]
    )

    set_max_range = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/move_group', 'max_range', '5.0'],
                output='screen'
            )
        ]
    )

    # Construct the path to the sensor manager launch file using substitutions.
    # This assumes that you have converted the sensor manager launch file to ROS2 and it is named:
    # "<moveit_sensor_manager>_moveit_sensor_manager.launch.py"
    sensor_manager_launch_file = PathJoinSubstitution(
        [config_share, 'launch', moveit_sensor_manager, TextSubstitution(text='_moveit_sensor_manager.launch.py')]
    )
    
    sensor_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_manager_launch_file)
    )

    return LaunchDescription([
        moveit_sensor_manager_arg,
        load_sensors_3d,
        set_octomap_resolution,
        set_max_range,
        sensor_manager_launch
    ])

if __name__ == '__main__':
    generate_launch_description()