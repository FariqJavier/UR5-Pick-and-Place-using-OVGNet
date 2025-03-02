# TODO

- [x] Move UR robot via ur_robot_driver
- [x] Move Robotiq gripper via robotiq_modbus_rtu
- [x] Create custom package for ur pick and place
- [x] Attach robotiq gripper to ur robot flange link
- [x] Add joint state publisher node to custom ur launch file[1]
- [x] Include Robotiq gripper on UR5 URDF
- [x] Setup Moveit
- [ ] Create prototype rospy script of pick and place by publishing to rostopic robot without moveit
- [ ] Create roscpp script of pick and place by publishing to rostopic robot without moveit
- [ ] Design egosentric robot mounting for Intel Realsense F200
- [ ] Implement OVGNet on robot controller


# NOTE

[1] Since robot state and joint state are necessary to load gripper urdf

# DOCUMENTATION POSE MOVEIT

- Create new moveit planning group for arm, add kin. chain from base link to flange and using track_ik plugin
- Create new moveit planning group for gripper, add joint of finger_joint and using kdl plugin
- UR5 (arm group): 2 pose: arm_home for standby position and arm_ready for hanging-like position
- Robotiq 2F-85 (gripper group): 3 pose: gripper open, gripper_small_close for grabing smaller object, gripper_big_close for grabing bigger object
- Using FollowJointTrajectory controller for both arm and gripper


# PROBLEM

- There is possibility of gripper will damage object since the pick-and-place task will be autonomous by relying on object detection model which will be grabing object with difference size, mass, and structure because the path planning gripper is not dynamic, its the grabing motion already pre-defined with static position. SOLUTION: Research about the graspnet, is it comes with force and pressure estimation or just grasping pose estimation and also there is effort parameter on JointState and JointTrajectory msgs type that can be utilized
- There is possibility of data published on certain necessary topic earlier or later than data published on other necessary topic which the data from the both topic need to be processed at the same time. SOLUTION: store the latest data even though the data is not real-time that can affect overall motion planning or synchronize the data so both data got published at the same time.