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

- custom_robot_pose = Subscribe to /tf, listen to transformed robot position. Publish to /robot_pose, calculate end-effector position (obtained from combination of transforming /base_link to /flange and transforming /flange to /robotiq_arg2f_base_link) and send it to/robot_pose

- custom_robot_manipulator = Subscribe to /robot_pose. listen to end-effector position. Publish to /arm_joint_trajectory, calculate joint trajectory needed to achieve desired cartesian position through inverse kinematic or forward kinematic and send it to /arm_joint_trajectory

- custom_arm_controller = Subscribe to /arm_joint_trajectory, listen to joint trajectory command. Publish to /pos_joint_traj_controller/command, transmit joint trajectory command to the topic. Also, changes can be made while executing a trajectory, allowing sudden changes in direction without having to wait until the previously specified position is reached. 

#### Launch Rviz to display custom robot model

Launch custom robot bringup

```sh
$ roslaunch ur_pick_and_place custom_ur5_bringup.launch
```

Launch example rviz

```sh
$ roslaunch ur_pick_and_place example_rviz.launch
```