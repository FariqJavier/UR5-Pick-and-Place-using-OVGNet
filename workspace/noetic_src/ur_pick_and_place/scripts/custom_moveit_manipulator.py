#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
from pynput.keyboard import Listener, Key,KeyCode
from tf.transformations import euler_from_quaternion, quaternion_from_euler

EF_POS_STEP = 0.05  # Position increment/decrement
EF_ANGLE_STEP = 0.05  # Orientation increment/decrement
GRIPPER_JOINT_STEP = 0.1

ESCAPE_COMBS = [
    {Key.ctrl_l, KeyCode(char='c')},
    {Key.ctrl_r, KeyCode(char='c')}
]


CONTROL_MSG = """
Test Control Your UR5 with MoveIt!
---------------------------
Move Test end-effector around:

Positional Movement:

        W  
    A   S   D     "DownKey"    "UpKey"    

Oriental Movement:

        P
    L   ;   '     "LeftKey"    "RightKey"

Arm position step +-= 0.05
Arm angle step +-= 0.05
Gripper Joint step +-= 0.1

W: Move Arm +X axis
S: Move Arm -X axis
A: Move Arm +Y axis
D: Move Arm -Y axis
UpKey: Move Arm +Z axis
DownKey: Move Arm -Z axis

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
        self.is_move = False
        self.get_keys()

    def get_latest_ef_pose(self):
        return self.arm_group.get_current_pose()

    def get_latest_gripper_joint(self):
        return self.gripper_group.get_current_joint_values()

    def set_target_ef_pose_by_teleop(self, key):
        self.target_ef_pose = self.get_latest_ef_pose()
        self.target_gripper_joint = self.get_latest_gripper_joint()

        current_orientation = self.target_ef_pose.pose.orientation
        current_quaternion = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]

        target_roll, target_pitch, target_yaw = euler_from_quaternion(current_quaternion)

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
            self.is_move = True
        elif key == Key.down:
            self.target_ef_pose.pose.position.z -= EF_POS_STEP
            self.is_move = True
        elif key == Key.right:
            target_yaw += EF_ANGLE_STEP
            self.is_move = True
        elif key== Key.left:
            target_yaw -= EF_ANGLE_STEP
            self.is_move = True

        if key == Key.shift_r:
            self.target_gripper_joint[0] -= GRIPPER_JOINT_STEP
            self.is_move = True
        elif key == Key.shift_l:
            self.target_gripper_joint[0] += GRIPPER_JOINT_STEP
            self.is_move = True
            

        try:
            if key.char == 'a':
                self.target_ef_pose.pose.position.y += EF_POS_STEP
                self.is_move = True
            elif key.char == 'd':
                self.target_ef_pose.pose.position.y -= EF_POS_STEP
                self.is_move = True
            elif key.char == 'w':
                self.target_ef_pose.pose.position.x += EF_POS_STEP
                self.is_move = True
            elif key.char == 's':
                self.target_ef_pose.pose.position.x -= EF_POS_STEP    
                self.is_move = True
            elif key.char == 'p':
                target_roll += EF_ANGLE_STEP 
                self.is_move = True
            elif key.char == ';':
                target_roll -= EF_ANGLE_STEP 
                self.is_move = True
            elif key.char == "'":
                target_pitch += EF_ANGLE_STEP 
                self.is_move = True
            elif key.char == "l":
                target_pitch -= EF_ANGLE_STEP
                self.is_move = True

        except AttributeError:
            pass

        target_quaternion = quaternion_from_euler(target_roll, target_pitch, target_yaw)

        self.target_ef_pose.pose.orientation.x = target_quaternion[0]
        self.target_ef_pose.pose.orientation.y = target_quaternion[1]
        self.target_ef_pose.pose.orientation.z = target_quaternion[2]
        self.target_ef_pose.pose.orientation.w = target_quaternion[3]

        if self.is_move:
            self.plan_and_execute_pose()

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
            self.arm_group.execute(self.target_ef_traj)
            # rospy.loginfo("Executed arm planned trajectory: %s", self.target_ef_traj)
            rospy.loginfo(self.target_ef_pose)

        if not self.target_gripper_joint:
            rospy.logwarn("No arm gripper joint set. Skipping planning and execution.")
            return
        else:
            self.gripper_group.set_joint_value_target(self.target_gripper_joint)
            success, self.target_gripper_traj, _, _ = self.gripper_group.plan()
            self.gripper_group.execute(self.target_gripper_traj)
            rospy.loginfo("Executed gripper planned trajectory: %s", self.target_gripper_traj)

        # # self.arm_group.set_pose_target(self.target_ef_pose)
        # self.arm_group.set_named_target("arm_ready")
        # success, self.target_ef_traj, _, _ = self.arm_group.plan()
        # print(success)
        # self.arm_group.execute(self.target_ef_traj)
        # rospy.loginfo("Executed planned trajectory: %s", self.target_ef_traj)

if __name__ == "__main__":
    print("MoveItManipulator script started")
    moveit_manipulator = MoveItManipulator()
    # moveit_manipulator.plan_and_execute_pose()