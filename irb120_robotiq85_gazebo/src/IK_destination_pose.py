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
    # Initialize rclpy and create a node
    rclpy.init(args=args)
    node = rclpy.create_node('moving_irb120_robot')

    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("irb_120")
    hand_group = moveit_commander.MoveGroupCommander("robotiq_85")

    # Create publisher for displaying planned trajectory in RViz
    display_trajectory_publisher = node.create_publisher(
        moveit_msgs.msg.DisplayTrajectory,
        '/move_group/display_planned_path',
        20
    )

    # Function to move the arm (using inverse kinematics) to a given pose target
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
        arm_group.stop()      # Ensure no residual movement
        arm_group.clear_pose_targets()

    # Function to move the gripper by setting the master joint value
    def move_joint_hand(gripper_finger1_joint):
        joint_goal = hand_group.get_current_joint_values()
        joint_goal[2] = gripper_finger1_joint  # Gripper master axis
        hand_group.go(joint_goal, wait=True)
        hand_group.stop()     # Ensure no residual movement

    # Main sequence (using a loop for demonstration)
    node.get_logger().info("Moving arm using IK destination poses")
    for i in range(1):
        node.get_logger().info("Moving arm to HOME point")
        move_pose_arm(0, 0.8, 0, 0.4, 0, 0.6)
        node.get_logger().info("Opening gripper")
        move_joint_hand(0)
        time.sleep(1)

        node.get_logger().info("Moving arm to point_1")
        move_pose_arm(0, 0.2, 0, 0.5, -0.25, 0.3)
        time.sleep(1)

        node.get_logger().info("Moving arm to point_2")
        move_pose_arm(0.5, 0.5, 0, 0.5, 0.25, 0.3)
        time.sleep(1)

        node.get_logger().info("Closing gripper to 0.4")
        move_joint_hand(0.4)
        time.sleep(1)

        node.get_logger().info("Moving arm to point_3")
        move_pose_arm(0.1, -pi/2, -0.7, 0.2, 0, 0.8)
        node.get_logger().info("Opening gripper")
        move_joint_hand(0)
        time.sleep(1)

        node.get_logger().info("Moving arm to point_4")
        move_pose_arm(0, pi/2, 0, 0, -0.5, 0.4)
        node.get_logger().info("Closing gripper to 0.6")
        move_joint_hand(0.6)
        time.sleep(1)

    node.get_logger().info("All movements finished. Shutting down")
    moveit_commander.roscpp_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()