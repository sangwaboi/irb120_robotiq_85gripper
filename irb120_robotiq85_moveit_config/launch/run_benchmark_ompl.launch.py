#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the 'cfg' launch argument that specifies the list of benchmark config files.
    cfg_arg = DeclareLaunchArgument(
        'cfg',
        default_value='',
        description='List of .cfg files for benchmarking'
    )
    cfg = LaunchConfiguration('cfg')
    
    # Get the share directory for the MoveIt configuration package.
    moveit_config_share = get_package_share_directory('irb120_robotiq85_moveit_config')
    
    # ------------------ PLANNING CONTEXT ------------------
    # Include the planning_context launch file with load_robot_description set to true.
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'planning_context.launch.py')
        ),
        launch_arguments={'load_robot_description': 'true'}.items()
    )
    
    # ------------------ WAREHOUSE DATABASE ------------------
    # Include the warehouse launch file with the database path set to "moveit_ompl_benchmark_warehouse".
    warehouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, 'launch', 'warehouse.launch.py')
        ),
        launch_arguments={'moveit_warehouse_database_path': 'moveit_ompl_benchmark_warehouse'}.items()
    )
    
    # ------------------ BENCHMARK EXECUTABLE NODE ------------------
    # Launch the benchmark node from the moveit_ros_benchmarks package.
    # Pass the 'cfg' argument and the flag '--benchmark-planners' as command line arguments.
    # Also, load additional parameters from the kinematics.yaml and ompl_planning.yaml files.
    benchmark_node = launch_ros.actions.Node(
        package='moveit_ros_benchmarks',
        executable='moveit_run_benchmark',
        name='moveit_benchmark',
        output='screen',
        arguments=[cfg, '--benchmark-planners'],
        parameters=[
            os.path.join(moveit_config_share, 'config', 'kinematics.yaml'),
            os.path.join(moveit_config_share, 'config', 'ompl_planning.yaml')
        ]
    )
    
    return LaunchDescription([
        cfg_arg,
        planning_context,
        warehouse,
        benchmark_node
    ])

if __name__ == '__main__':
    generate_launch_description()