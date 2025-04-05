#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments (as in ROS1)
    load_robot_description_arg = DeclareLaunchArgument(
        'load_robot_description', default_value='false',
        description='Whether to load the robot description (URDF) via xacro'
    )
    robot_description_arg = DeclareLaunchArgument(
        'robot_description', default_value='robot_description',
        description='Parameter name for the robot description'
    )

    load_robot_description = LaunchConfiguration('load_robot_description')
    robot_description = LaunchConfiguration('robot_description')

    # Get package share directories
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')
    irb120_camara_diego_share = get_package_share_directory('irb120_camara_diego')

    # Define file paths
    srdf_file = os.path.join(moveit_config_share, 'config', 'irb120_robotiq85.srdf')
    joint_limits_yaml = os.path.join(moveit_config_share, 'config', 'joint_limits.yaml')
    kinematics_yaml = os.path.join(moveit_config_share, 'config', 'kinematics.yaml')
    # URDF xacro file (if needed) is at:
    # urdf_xacro = os.path.join(irb120_camara_diego_share, 'urdf', 'irb120_robotiq85_macro.xacro')
    
    # Read SRDF content from file
    srdf_content = ''
    try:
        with open(srdf_file, 'r') as f:
            srdf_content = f.read()
    except Exception as e:
        print("Error reading SRDF file:", e)
    
    # Load the SRDF into the /move_group node as parameter "robot_description_semantic"
    # We use TimerAction to delay these ExecuteProcess commands until /move_group is running.
    load_srdf = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/move_group', 'robot_description_semantic', srdf_content],
                output='screen'
            )
        ]
    )
    
    # Load the joint limits YAML into /move_group
    load_joint_limits = TimerAction(
        period=2.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'load', '/move_group', joint_limits_yaml],
                output='screen'
            )
        ]
    )
    
    # Load the kinematics YAML into /move_group
    load_kinematics = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'load', '/move_group', kinematics_yaml],
                output='screen'
            )
        ]
    )
    
    # Note: In ROS2, the actual loading of the URDF via xacro is usually done
    # in the main launch file that starts the robot_state_publisher.
    # Here, we focus on loading the semantic description and additional parameters.
    
    return LaunchDescription([
        load_robot_description_arg,
        robot_description_arg,
        load_srdf,
        load_joint_limits,
        load_kinematics,
    ])

if __name__ == '__main__':
    generate_launch_description()