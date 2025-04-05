#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the fake_controllers.yaml file
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')
    fake_controllers_yaml = os.path.join(moveit_config_share, 'config', 'fake_controllers.yaml')
    
    # Action to set the 'moveit_controller_manager' parameter on the /move_group node
    set_param = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'set', '/move_group',
            'moveit_controller_manager',
            'moveit_fake_controller_manager/MoveItFakeControllerManager'
        ],
        output='screen'
    )
    
    # Action to load additional parameters from the YAML file into the /move_group node
    load_yaml = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'load', '/move_group', fake_controllers_yaml
        ],
        output='screen'
    )
    
    # Delay the execution slightly to allow /move_group to start up
    delayed_set_param = TimerAction(
        period=2.0,  # Delay 2 seconds
        actions=[set_param]
    )
    delayed_load_yaml = TimerAction(
        period=2.5,  # Delay 2.5 seconds
        actions=[load_yaml]
    )
    
    return LaunchDescription([
        delayed_set_param,
        delayed_load_yaml
    ])