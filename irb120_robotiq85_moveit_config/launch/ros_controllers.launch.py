#!/usr/bin/env python3
import os
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for the MoveIt configuration package.
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')
    # Construct the full path to the YAML configuration file.
    ros_controllers_yaml = os.path.join(moveit_config_share, 'config', 'ros_controllers.yaml')

    # Launch the controller spawner node.
    # It loads parameters from the YAML file and spawns the controller "arm_position_controller".
    controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        output='screen',
        arguments=['arm_position_controller'],
        parameters=[ros_controllers_yaml]
    )

    return LaunchDescription([
        controller_spawner
    ])

if __name__ == '__main__':
    generate_launch_description()