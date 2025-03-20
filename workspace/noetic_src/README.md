# NOETIC Development of Robotic Pick and Place Using UR5 and OVGNet

A ROS Noetic-based implementation of robotic pick and place operations using a Universal Robots UR5 CB3 manipulator with Robotiq 2F-85 adaptive gripper integrated with OVGNet for object detection and grasping.

## Overview

This project combines:
- Universal Robots UR5 robotic arm control using ROS Noetic
  - UR5 CB3 control via ur_robot_driver
  - Real-time trajectory execution and monitoring
- Robotiq 2F-85 gripper integration
  - Gripper control through robotiq_modbus_rtu
  - Modbus RTU protocol for precise finger positioning
- OVGNet (Open Vocabulary Grasping Network) for object and grasp detection
- ROS packages for motion planning and execution
- Docker-based development environment

## Repository Structure

The workspace contains:
- `robotiq/` - Robotiq adaptive gripper description configuration [[Source](https://github.com/clearpathrobotics/robotiq.git)]
- `universal_robot/` - UR5 robot description and MoveIt! configurations [[Source](https://github.com/ros-industrial/universal_robot.git)]
- `ur_pick_and_place/` - Custom pick and place implementation

## Prerequisites

- ROS Noetic
- Docker
- ur_robot_driver (for CB3 and e-Series driver)
- MoveIt!

## Quick Start

### Robot Nodes

#### Extract calibration information

```sh
$ roslaunch ur_calibration calibration_correction.launch robot_ip:=${ROBOT_IP} target_filename:="${HOME}/ur5_calibration.yaml"
```

#### Run Robot Driver with the calibration file

```sh
$ roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=${ROBOT_IP} kinematics_config:=$(rospack find ur_pick_and_place)/etc/ur5_calibration.yaml
```

#### Run example Rviz interface

```sh
$ roslaunch ur_robot_driver example_rviz.launch
```

#### Run controllers script

```sh
$ rosrun ur_robot_driver test_move
```

#### Run MoveIt

```sh
$ roslaunch ur5_moveit_config moveit_planning_execution.launch
$ roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_moveit_config)/launch/moveit.rviz
```

### Gripper Nodes (Run Robot bringup first)

#### Run gripper action server

```sh
$ roslaunch robotiq_2f_gripper_control robotiq_action_server.launch comport comport:=/dev/ttyUSB0 joint_name:=finger_joint
```

#### Run gripper example action client

```sh
$ rosrun robotiq_2f_gripper_control robotiq_2f_action_client_example.py
```

### Custom Robot with Gripper Nodes

consists of 3 nodes:

- custom_ee_pose = Subscribe to /tf, listen to transformed robot position. Publish to /ee_pose, calculate end-effector position (obtained from /tf) and send it to /ee_pose.

- custom_ee_manipulator = Subscribe to /ee_pose. listen to end-effector position. Publish to /target_arm_joint, calculate target robot joint given target end-effector through Track_IK kinematic solver using solve_type=Manip1 (sqrt(det(J * J^T)) which Maximizes Manipulability that Computes Yoshikawa's manipulability index)  and send it to /target_arm_joint. It also subscribe to /joint_states to get the latest joint value to calculate inverse kinematic.

- custom_arm_controller = Subscribe to /arm_joint_trajectory, listen to joint trajectory command. Publish to /pos_joint_traj_controller/command, transmit joint trajectory command to the topic. Also, changes can be made while executing a trajectory, allowing sudden changes in direction without having to wait until the previously specified position is reached. 

configuring custom robot inverse kinematic using moveit:

- UR5 (arm group) = using track ik moveit plugin
- Robotiq 2F-85 (gripper group) = using kdl moveit plugin

####  Move Robot using MoveIt!

Launch custom robot bringup node

```sh
$ roslaunch ur_pick_and_place custom_ur5_bringup.launch
```

In Order for robot UR5 to be able ro receive external command from ROS, ensure that the UR5 robot is in ON state then make sure to run the program that has external command enable

Launch MoveIt! Rviz

```sh
$ roslaunch ur_pick_and_place moveit_rviz.launch
```

Launch MoveIt! configuration

```sh
$ roslaunch ur_pick_and_place custom_move_group.launch
```

Launch Custom Moveit Manipulator Node

```sh
$ rosrun ur_pick_and_place custom_moveit_manipulator.py
```

####  Run Intel RealSense SDK 2.0

For testing if camera is connected or not

```sh
$ sudo realsense-viewer
```

Or run as ROOT user on the container

```sh
$ docker exec -ti --user root ros_noetic bash
```

```sh
$ realsense-viewer
```

```sh
$ roslaunch realsense2_camera rs_camera.launch
```