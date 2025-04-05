#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for warehouse settings
    warehouse_port_arg = DeclareLaunchArgument(
        'moveit_warehouse_port',
        default_value='33829',
        description='Default DB port for MoveIt (non-standard to avoid conflicts)'
    )
    warehouse_host_arg = DeclareLaunchArgument(
        'moveit_warehouse_host',
        default_value='localhost',
        description='Default DB host for MoveIt'
    )

    # Create LaunchConfiguration substitutions for the arguments
    warehouse_port = LaunchConfiguration('moveit_warehouse_port')
    warehouse_host = LaunchConfiguration('moveit_warehouse_host')

    # Log the parameter values (for debugging/verification purposes)
    log_port = LogInfo(msg=["Warehouse Port: ", warehouse_port])
    log_host = LogInfo(msg=["Warehouse Host: ", warehouse_host])

    # In ROS2, parameters for nodes are typically passed directly to the node's parameter list.
    # So this file serves as a stub to declare and log the desired parameters.
    # The including launch file should pass these values to the warehouse node.
    # Alternatively, you can convert these parameters to a YAML file and load that YAML file directly.

    return LaunchDescription([
        warehouse_port_arg,
        warehouse_host_arg,
        log_port,
        log_host
    ])

if __name__ == '__main__':
    generate_launch_description()