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

## TODO

- [x] Move UR robot via ur_robot_driver
- [x] Move Robotiq gripper via robotiq_modbus_rtu
- [ ] Include Robotiq gripper on UR5 URDF
- [ ] Design egosentric robot mounting for Intel Realsense F200
- [ ] Implement OVGNet on robot controller

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



