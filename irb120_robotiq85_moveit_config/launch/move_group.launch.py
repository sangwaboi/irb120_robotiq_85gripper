#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Enable debug mode (GDB prefix, verbose output)'
    )
    launch_prefix_arg = DeclareLaunchArgument(
        'launch_prefix',
        default_value='',
        description='Prefix for launching move_group (e.g. gdb -x ... if debug enabled)'
    )
    info_arg = DeclareLaunchArgument(
        'info', default_value=LaunchConfiguration('debug'),
        description='Verbose mode; if true, add --debug to command arguments'
    )
    command_args_arg = DeclareLaunchArgument(
        'command_args', default_value='',
        description='Additional command line arguments for move_group'
    )
    allow_trajectory_execution_arg = DeclareLaunchArgument(
        'allow_trajectory_execution', default_value='true',
        description='Enable trajectory execution'
    )
    fake_execution_arg = DeclareLaunchArgument(
        'fake_execution', default_value='false',
        description='Use fake execution'
    )
    max_safe_path_cost_arg = DeclareLaunchArgument(
        'max_safe_path_cost', default_value='1',
        description='Maximum safe path cost'
    )
    jiggle_fraction_arg = DeclareLaunchArgument(
        'jiggle_fraction', default_value='0.05',
        description='Jiggle fraction'
    )
    publish_monitored_planning_scene_arg = DeclareLaunchArgument(
        'publish_monitored_planning_scene', default_value='true',
        description='Publish monitored planning scene updates'
    )
    capabilities_arg = DeclareLaunchArgument(
        'capabilities', default_value='',
        description='Additional MoveGroup capabilities to load (space-separated)'
    )
    disable_capabilities_arg = DeclareLaunchArgument(
        'disable_capabilities', default_value='',
        description='Default MoveGroup capabilities to disable (space-separated)'
    )

    # Launch configurations
    debug = LaunchConfiguration('debug')
    launch_prefix = LaunchConfiguration('launch_prefix')
    info = LaunchConfiguration('info')
    command_args = LaunchConfiguration('command_args')
    allow_trajectory_execution = LaunchConfiguration('allow_trajectory_execution')
    fake_execution = LaunchConfiguration('fake_execution')
    max_safe_path_cost = LaunchConfiguration('max_safe_path_cost')
    jiggle_fraction = LaunchConfiguration('jiggle_fraction')
    publish_monitored_planning_scene = LaunchConfiguration('publish_monitored_planning_scene')
    capabilities = LaunchConfiguration('capabilities')
    disable_capabilities = LaunchConfiguration('disable_capabilities')

    # Get package share directory for moveit configuration
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')

    # --------------------- INCLUDE LAUNCH FILES ---------------------
    # Include the planning context launch (loads URDF, SRDF, and parameters)
    planning_context = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(moveit_config_share, 'launch', 'planning_context.launch.py')),
                launch_arguments={'load_robot_description': 'true'}.items()
            )
        ],
        namespace='move_group'
    )

    # Include the planning pipeline launch (e.g., for OMPL)
    planning_pipeline = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(moveit_config_share, 'launch', 'planning_pipeline.launch.py')),
                launch_arguments={'pipeline': 'ompl'}.items()
            )
        ],
        namespace='move_group'
    )

    # Include the trajectory execution launch, only if trajectory execution is allowed.
    trajectory_execution = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(moveit_config_share, 'launch', 'trajectory_execution.launch.py')),
                # Here we assume the trajectory_execution.launch.py file handles the fake_execution condition internally.
                launch_arguments={
                    'moveit_manage_controllers': 'true',
                    'moveit_controller_manager': 'irb120_robotiq85'  # if fake_execution is false; otherwise, it should be "fake"
                }.items()
            )
        ],
        namespace='move_group',
        condition=IfCondition(allow_trajectory_execution)
    )

    # Include the sensor manager launch, if trajectory execution is allowed.
    sensor_manager = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(moveit_config_share, 'launch', 'sensor_manager.launch.py')),
                launch_arguments={'moveit_sensor_manager': 'irb120_robotiq85'}.items()
            )
        ],
        namespace='move_group',
        condition=IfCondition(allow_trajectory_execution)
    )

    # --------------------- MOVE_GROUP NODE ---------------------
    # Set the DISPLAY environment variable (for OpenGL, if needed)
    set_display = SetEnvironmentVariable('DISPLAY', LaunchConfiguration('DISPLAY', default=':0'))

    # Launch the move_group node
    move_group_node = launch_ros.actions.Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        prefix=launch_prefix,
        arguments=[command_args],
        parameters=[{
            'allow_trajectory_execution': allow_trajectory_execution,
            'max_safe_path_cost': max_safe_path_cost,
            'jiggle_fraction': jiggle_fraction,
            'capabilities': capabilities,
            'disable_capabilities': disable_capabilities,
            'planning_scene_monitor/publish_planning_scene': publish_monitored_planning_scene,
            'planning_scene_monitor/publish_geometry_updates': publish_monitored_planning_scene,
            'planning_scene_monitor/publish_state_updates': publish_monitored_planning_scene,
            'planning_scene_monitor/publish_transforms_updates': publish_monitored_planning_scene
        }]
    )

    # --------------------- RETURN THE LAUNCH DESCRIPTION ---------------------
    return LaunchDescription([
        debug_arg,
        launch_prefix_arg,
        info_arg,
        command_args_arg,
        allow_trajectory_execution_arg,
        fake_execution_arg,
        max_safe_path_cost_arg,
        jiggle_fraction_arg,
        publish_monitored_planning_scene_arg,
        capabilities_arg,
        disable_capabilities_arg,
        planning_context,
        planning_pipeline,
        trajectory_execution,
        sensor_manager,
        set_display,
        move_group_node
    ])

if __name__ == '__main__':
    generate_launch_description()