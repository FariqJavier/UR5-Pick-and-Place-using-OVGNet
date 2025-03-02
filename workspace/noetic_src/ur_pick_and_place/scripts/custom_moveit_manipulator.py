#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
from pynput.keyboard import Listener, Key,KeyCode

EF_POS_STEP = 0.05  # Position increment/decrement
EF_ANGLE_POS = 0.05  # Orientation increment/decrement
GRIPPER_JOINT_STEP = 0.1

ESCAPE_COMBS = [
    {Key.ctrl_l, KeyCode(char='c')},
    {Key.ctrl_r, KeyCode(char='c')}
]


CONTROL_MSG = """
Test Control Your UR5 with MoveIt!
---------------------------
Move Test end-effector around:
        W
   A    S    D

Arm position step +-= 0.05
Arm angle step +-= 0.05
Gripper Joint step +-= 0.1

W: Move Arm Upward
A: Move Arm Leftside
S: Move Arm Downward
D: Move Arm Rightside
Left Shift: Close Gripper
Right Shift: Right Gripper


CTRL-C to quit
"""

E_MSG = """
Communications Failed
"""

class MoveItManipulator:
    def __init__(self):
        rospy.init_node('custom_moveit_manipulator', anonymous=True)

        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.target_ef_pose = None
        self.target_ef_traj = None
        
        self.target_gripper_joint = None
        self.target_gripper_traj = None

        self.escape_key_pressed = None
        self.get_keys()

    def get_latest_ef_pose(self):
        return self.arm_group.get_current_pose()

    def get_latest_gripper_joint(self):
        return self.gripper_group.get_current_joint_values()

    def set_target_ef_pose_by_teleop(self, key):
        self.target_ef_pose = self.get_latest_ef_pose()
        self.target_gripper_joint = self.get_latest_gripper_joint()

        if key in [Key.ctrl_l, Key.ctrl_r]:  
            self.escape_key_pressed.add(key)
        elif key == KeyCode(char='c'):  
            self.escape_key_pressed.add(key)

        for comb in ESCAPE_COMBS:
            if comb.issubset(self.escape_key_pressed): 
                return False

        if key == Key.esc:
                return False
            
        if key == Key.up:
            self.target_ef_pose.pose.position.z += EF_POS_STEP
            self.plan_and_execute_pose()
        elif key == Key.down:
            self.target_ef_pose.pose.position.z -= EF_POS_STEP
            self.plan_and_execute_pose()
        elif key == Key.right:
            self.target_ef_pose.pose.orientation.z += EF_POS_STEP
            self.plan_and_execute_pose()
        elif key== Key.left:
            self.target_ef_pose.pose.orientation.z -= EF_POS_STEP
            self.plan_and_execute_pose()

        if key == Key.shift_r:
            self.target_gripper_joint[0] -= GRIPPER_JOINT_STEP
            self.plan_and_execute_pose()
        elif key == Key.shift_l:
            self.target_gripper_joint[0] += GRIPPER_JOINT_STEP
            self.plan_and_execute_pose()
            

        try:
            if key.char == 'a':
                self.target_ef_pose.pose.position.y += EF_POS_STEP
            elif key.char == 'd':
                self.target_ef_pose.pose.position.y -= EF_POS_STEP
            elif key.char == 'w':
                self.target_ef_pose.pose.position.x += EF_POS_STEP
            elif key.char == 's':
                self.target_ef_pose.pose.position.x -= EF_POS_STEP    
            elif key.char == 'p':
                self.target_ef_pose.pose.orientation.x += EF_POS_STEP
            elif key.char == ';':
                self.target_ef_pose.pose.orientation.x -= EF_POS_STEP
            elif key.char == "'":
                self.target_ef_pose.pose.orientation.x += EF_POS_STEP
            elif key.char == "l":
                self.target_ef_pose.pose.orientation.x -= EF_POS_STEP

            self.plan_and_execute_pose()
        except AttributeError:
            pass

    def get_keys(self):
        self.escape_key_pressed = set()

        with Listener(on_press=self.set_target_ef_pose_by_teleop, suppress=True) as listener:
            try:
                print(CONTROL_MSG)
                listener.join()
            finally:
                moveit_commander.roscpp_shutdown()

    def plan_and_execute_pose(self):
        if not self.target_ef_pose:
            rospy.logwarn("No arm target pose set. Skipping planning and execution.")
            return
        else:
            self.arm_group.set_pose_target(self.target_ef_pose)
            success, self.target_ef_traj, _, _ = self.arm_group.plan()
            print(success)
            self.arm_group.execute(self.target_ef_traj)
            rospy.loginfo("Executed arm planned trajectory: %s", self.target_ef_traj)

        if not self.target_gripper_joint:
            rospy.logwarn("No arm gripper joint set. Skipping planning and execution.")
            return
        else:
            self.gripper_group.set_joint_value_target(self.target_gripper_joint)
            success, self.target_gripper_traj, _, _ = self.gripper_group.plan()
            print(success)
            self.gripper_group.execute(self.target_gripper_traj)
            rospy.loginfo("Executed gripper planned trajectory: %s", self.target_gripper_traj)

        # if not self.target_ef_pose:
        #     rospy.logwarn("No target pose set. Skipping planning and execution.")
        #     return

        # self.arm_group.set_pose_target(self.target_ef_pose)
        # # self.arm_group.set_named_target("arm_ready")
        # success, self.target_ef_traj, _, _ = self.arm_group.plan()
        # print(success)
        # self.arm_group.execute(self.target_ef_traj)
        # rospy.loginfo("Executed planned trajectory: %s", self.target_ef_traj)

if __name__ == "__main__":
    print("MoveItManipulator script started")
    moveit_manipulator = MoveItManipulator()
    # moveit_manipulator.plan_and_execute_pose()