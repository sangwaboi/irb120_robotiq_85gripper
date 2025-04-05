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
    
    # Create a node for logging purposes
    node = rclpy.create_node('moving_irb120_robot')
    
    # Initialize MoveIt Commander (this call may remain as is for compatibility)
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Create MoveIt Commander objects
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("irb_120")
    hand_group = moveit_commander.MoveGroupCommander("robotiq_85")
    
    # Create a publisher for displaying the planned trajectory in RViz
    display_trajectory_publisher = node.create_publisher(
        moveit_msgs.msg.DisplayTrajectory,
        '/move_group/display_planned_path',
        20
    )
    
    # Forward Kinematics: move the arm to a specified joint configuration
    def move_joint_arm(j0, j1, j2, j3, j4, j5):
        joint_goal = arm_group.get_current_joint_values()
        joint_goal[0] = j0
        joint_goal[1] = j1
        joint_goal[2] = j2
        joint_goal[3] = j3
        joint_goal[4] = j4
        joint_goal[5] = j5
        arm_group.go(joint_goal, wait=True)
        arm_group.stop()  # Ensure no residual movement

    # Move the Robotiq gripper by adjusting the master joint (assumed index 2)
    def move_joint_hand(gripper_finger1_joint):
        joint_goal = hand_group.get_current_joint_values()
        joint_goal[2] = gripper_finger1_joint  # Gripper master axis
        hand_group.go(joint_goal, wait=True)
        hand_group.stop()  # Ensure no residual movement

    node.get_logger().info("============ Printing robot state ============")
    node.get_logger().info(str(robot.get_current_state()))
    node.get_logger().info("")

    # Example of FK in a loop:
    for i in range(2):
        node.get_logger().info("Moving arm to pose_1")
        move_joint_arm(-pi/2, 0.5, 0.2, -1, 0.2, 0.2)
        time.sleep(1)
        node.get_logger().info("Opening gripper")
        move_joint_hand(0)
        time.sleep(1)
        node.get_logger().info("Moving arm to pose_2")
        move_joint_arm(0, 0, -0.2, 0, 1, 0.3)
        time.sleep(1)
        node.get_logger().info("Closing gripper")
        move_joint_hand(0.5)
        time.sleep(1)

    node.get_logger().info("All movements finished. Shutting down")
    moveit_commander.roscpp_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()