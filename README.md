# irb120_robotiq_85gripper


Welcome to the upgraded and supercharged version of **irb120_robotiq85**! This repository originally contained a Gazebo simulation for the ABB IRB120 6-axis industrial robot with a Robotiq 85 2-finger gripper built for ROS Kinetic. We’ve taken the original base files and migrated everything to a modern ROS2/MoveIt2 tech stack to support new industrial exercises and learning processes. Huge thanks to the original creator for providing a solid foundation—we’re simply building on that legacy with some ROS2 magic!

---

## Overview

In this updated version, you can now:
- **Run a complete simulation** in Gazebo integrated with MoveIt2 (including state controllers, trajectory execution, and RViz visualization).
- **Integrate sensors** like a Kinect camera into the simulation.
- **Interface with MoveIt Commander** via Python scripts to test forward kinematics, inverse kinematics, and Cartesian path planning.
- **Leverage the latest ROS2 features** and MoveIt2 improvements for a more modular, flexible, and future-proof robotics learning platform.

Our primary goal is to develop newer industrial exercises and enable advanced learning using this state-of-the-art setup!

---

## Dependencies and Installation

### Prerequisites

- **ROS2 Distribution:**  
  (e.g., ROS2 Humble or Galactic)
- **Gazebo/ Ignition Gazebo:**  
  For simulation.
- **MoveIt2:**  
  For motion planning and control.
- **Python 3:**  
  All nodes and scripts are now Python 3-based.

### Required External Repositories

#### ABB IRB120 Industrial Robot
The simulation uses the ROS-Industrial ABB experimental metapackage:
```bash
cd ~/ros2_ws/src
git clone https://github.com/ros-industrial/abb_experimental.git -b kinetic-devel
git clone https://github.com/ros-industrial/abb.git -b kinetic-devel

##Clone the repository provided by Stanley Innovation:
cd ~/ros2_ws/src
git clone https://github.com/StanleyInnovation/robotiq_85_gripper.git

##Clone this updated ROS2 version:
cd ~/ros2_ws/src
git clone https://github.com/your_username/irb120_robotiq85.git

###Build your ROS2 workspace using colcon:

cd ~/ros2_ws
colcon build --packages-select irb120_robotiq85_gazebo irb120_robotiq85_moveit_config
source install/setup.bash

##Gazebo Simulation Modes
##Launch the basic Gazebo simulation (with state and command controllers):

ros2 launch irb120_robotiq85_gazebo irb120_robotiq85_gazebo.launch.py

##Launch the simulation integrated with MoveIt Commander:
ros2 launch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit.launch.py

##Launch the simulation with MoveIt Commander and RViz (including the MoveIt plugin):
ros2 launch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit_rviz.launch.py





##Kinect Integration

##Launch the simulation with an integrated Kinect camera:

ros2 launch irb120_robotiq85_gazebo irb120_robotiq85_kinect_gazebo.launch.py

##Launch the simulation with a Kinect camera and scene objects:

ros2 launch irb120_robotiq85_gazebo irb120_robotiq85_kinect_gazebo_world.launch.py



##MoveIt Commander API – Python Programs

##After launching the simulation with MoveIt Commander, run the following scripts:
##Forward Kinematics:
ros2 run irb120_robotiq85_gazebo forward_kinematics.py ## can also watch video in the media folder 

#Inverse Kinematics (Move Joints):
ros2 run irb120_robotiq85_gazebo IK_destination_pose.py

#Cartesian Path (IK Linear Motion):
ros2 run irb120_robotiq85_gazebo IK_cartesian_path.py

