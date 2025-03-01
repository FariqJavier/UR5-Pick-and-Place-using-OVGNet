#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
from pynput.keyboard import Listener, Key

EF_POS_STEP = 0.05  # Position increment
EF_ANGLE_POS = 0.05  # Orientation increment
GRIPPER_JOINT_STEP = 0.005

control_msg = """
Control Your UR5 with MoveIt!
---------------------------
Moving end-effector around:
        W
   A    S    D

Arm position step += 0.05
Arm angle step += 0.05

W: Move Upward
A: Move Leftside
S: Move Downward
D: Move Rightside

SPACE KEY : force stop

CTRL-C to quit
"""

error_msg = """
Communications Failed
"""

class MoveItManipulator:
    def __init__(self):
        rospy.init_node('custom_moveit_manipulator', anonymous=True)

        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.latest_ef_pose = None
        self.target_ef_pose = None

        self.target_ef_traj = None
        
        self.latest_gripper_joint = None
        self.target_gripper_joint = None

        self.get_keys()

    def get_latest_ef_pose(self):
        self.latest_ef_pose = self.arm_group.get_current_pose().pose
        self.target_ef_pose = self.target_ef_pose

    def get_latest_gripper_joint(self):
        self.latest_gripper_joint = self.gripper_group.get_current_joint_values()

    def set_target_ef_pose_by_teleop(self, key):
        self.get_latest_ef_pose()
        try:
            if key.char == 'a':
                self.target_ef_pose.position.y += EF_POS_STEP
            elif key.char == 'd':
                self.target_ef_pose.position.y -= EF_POS_STEP
            elif key.char == 'w':
                self.target_ef_pose.position.x += EF_POS_STEP
            elif key.char == 's':
                self.target_ef_pose.position.x -= EF_POS_STEP    
            elif key == Key.up:
                self.target_ef_pose.position.z += EF_POS_STEP
            elif key == Key.down:
                self.target_ef_pose.position.z -= EF_POS_STEP
            elif key == Key.right:
                self.target_ef_pose.orientation.z += EF_POS_STEP
            elif key == Key.left:
                self.target_ef_pose.orientation.z -= EF_POS_STEP
            elif key.char == 'p':
                self.target_ef_pose.orientation.x += EF_POS_STEP
            elif key.char == ';':
                self.target_ef_pose.orientation.x -= EF_POS_STEP
            elif key.char == "'":
                self.target_ef_pose.orientation.x += EF_POS_STEP
            elif key.char == "l":
                self.target_ef_pose.orientation.x -= EF_POS_STEP
            
            self.plan_and_execute_pose()
        except AttributeError:
            pass

    def get_keys(self):
        try:
            with Listener(on_press=self.set_target_ef_pose_by_teleop) as listener:
                print(control_msg)
                listener.join()
        except KeyboardInterrupt:
            rospy.loginfo("Ctrl+C detected. Exiting gracefully...")
            rospy.signal_shutdown("KeyboardInterrupt")
            moveit_commander.roscpp_shutdown()
            sys.exit(0)

    def plan_and_execute_pose(self):
        if not self.target_ef_pose:
            rospy.logwarn("No target pose set. Skipping planning and execution.")
            return

        self.arm_group.set_pose_target(self.target_ef_pose)
        _, self.target_ef_traj, _, _ = self.arm_group.plan()
        rospy.loginfo("Executed planned trajectory: %s", self.target_ef_traj)

if __name__ == "__main__":
    print("MoveItManipulator script started")
    MoveItManipulator()
    # rospy.spin()