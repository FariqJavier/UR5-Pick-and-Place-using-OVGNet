# TODO

- [x] Move UR robot via ur_robot_driver
- [x] Move Robotiq gripper via robotiq_modbus_rtu
- [x] Create custom package for ur pick and place
- [x] Attach robotiq gripper to ur robot flange link
- [x] Add joint state publisher node to custom ur launch file[1]
- [x] Include Robotiq gripper on UR5 URDF
- [ ] Create prototype rospy script of pick and place by publishing to rostopic robot without moveit
- [ ] Create roscpp script of pick and place by publishing to rostopic robot without moveit
- [ ] Design egosentric robot mounting for Intel Realsense F200
- [ ] Implement OVGNet on robot controller


# NOTE

[1] Since robot state and joint state are necessary to load gripper urdf