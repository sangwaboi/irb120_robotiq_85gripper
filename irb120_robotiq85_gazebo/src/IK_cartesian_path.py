#!/usr/bin/env python3
import sys
import copy
import time
import rclpy
from rclpy.node import Node
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf_transformations import euler_from_quaternion, quaternion_from_euler

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)
    node = rclpy.create_node('moving_irb120_robot')
    
    # Initialize moveit_commander (assumes ROS2-compatible moveit_commander API)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("irb_120")
    hand_group = moveit_commander.MoveGroupCommander("robotiq_85")
    
    # Create publisher for displaying trajectory in RViz
    display_trajectory_publisher = node.create_publisher(
        moveit_msgs.msg.DisplayTrajectory,
        '/move_group/display_planned_path',
        20
    )
    
    # Function to move the arm to a given pose target (IK)
    def move_pose_arm(roll, pitch, yaw, x, y, z):
        pose_goal = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        arm_group.set_pose_target(pose_goal)
        plan = arm_group.go(wait=True)
        arm_group.stop()  # Ensure no residual movement
        arm_group.clear_pose_targets()
    
    # Build waypoints for a Cartesian path
    waypoints = []
    wpose = arm_group.get_current_pose().pose
    wpose.position.z -= 0.3  # Move up (z) relative units
    wpose.position.y += 0.2  # Move sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    
    # Position 2, lateral (absolute)
    wpose.position.x = 0
    wpose.position.y = 0.5
    wpose.position.z = 0.5  
    waypoints.append(copy.deepcopy(wpose))
    
    # Position 3, over the robot (absolute)
    wpose.position.x = 0.2
    wpose.position.y = 0
    wpose.position.z = 0.7
    waypoints.append(copy.deepcopy(wpose))
    
    # Position 4, lateral (absolute)
    wpose.position.x = 0
    wpose.position.y = -0.5
    wpose.position.z = 0.5  
    waypoints.append(copy.deepcopy(wpose))
    
    # Position 5, frontal (absolute)
    wpose.position.x = 0.5
    wpose.position.y = 0
    wpose.position.z = 0.3  
    waypoints.append(copy.deepcopy(wpose))
    
    # Compute the Cartesian path (returns a plan and fraction of path achieved)
    (plan, fraction) = arm_group.compute_cartesian_path(
        waypoints,   # Waypoints to follow
        0.01,        # End-effector step size
        0.0          # Jump threshold
    )
    node.get_logger().info("Cartesian path planned")
    
    # Publish the planned trajectory for visualization in RViz
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)
    
    # Execute the trajectory plan
    arm_group.execute(plan, wait=True)
    node.get_logger().info("Cartesian path finished. Shutting down")
    
    moveit_commander.roscpp_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()