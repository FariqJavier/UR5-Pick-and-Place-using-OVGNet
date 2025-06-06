#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped
import math
from pynput.keyboard import Listener, Key,KeyCode
from tf.transformations import quaternion_multiply, quaternion_from_euler
import numpy as np

#################################################################################
## NOT USING EULER CONVENTION SINCE IT WILL ENCOUNTER GIMBAL LOCK PROBLEM FOR PITCH AND YAW
##################################################################################


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

def normalize_quaternion(q):
    """ Normalize a quaternion to ensure it stays valid """
    norm = np.linalg.norm(q)
    return [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm] if norm > 0 else q
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

        self.current_quarternion = None
        self.target_quarternion = None

        self.escape_key_pressed = None
        self.is_move = False
        # self.get_keys()

    def get_latest_ef_pose(self):
        return self.arm_group.get_current_pose()

    def get_latest_gripper_joint(self):
        return self.gripper_group.get_current_joint_values()

    def set_target_ef_pose_by_teleop(self, key):
        self.target_ef_pose = self.get_latest_ef_pose()
        self.target_gripper_joint = self.get_latest_gripper_joint()

        current_orientation = self.target_ef_pose.pose.orientation
        self.current_quaternion = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]

        delta_quat = None

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
            delta_quat = quaternion_from_euler(EF_ANGLE_STEP, 0, 0)
        elif key== Key.left:
            delta_quat = quaternion_from_euler(-EF_ANGLE_STEP, 0, 0)

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
                delta_quat = quaternion_from_euler(0, 0, EF_ANGLE_STEP) 
            elif key.char == ';':
                delta_quat = quaternion_from_euler(0, 0, -EF_ANGLE_STEP)
            elif key.char == "'":
                delta_quat = quaternion_from_euler(0, EF_ANGLE_STEP, 0) 
            elif key.char == "l":
                delta_quat = quaternion_from_euler(0, -EF_ANGLE_STEP, 0)

        except AttributeError:
            pass

        if delta_quat is not None:
            self.target_quaternion = quaternion_multiply(self.current_quaternion, delta_quat)
            self.target_quaternion = normalize_quaternion(self.target_quaternion)

            # Apply the new quaternion to the pose
            self.target_ef_pose.pose.orientation.x = self.target_quaternion[0]
            self.target_ef_pose.pose.orientation.y = self.target_quaternion[1]
            self.target_ef_pose.pose.orientation.z = self.target_quaternion[2]
            self.target_ef_pose.pose.orientation.w = self.target_quaternion[3]

            self.is_move = True

        if self.is_move:
            self.target_ef_pose.pose.position.x = round(self.target_ef_pose.pose.position.x, 4)
            self.target_ef_pose.pose.position.y = round(self.target_ef_pose.pose.position.y, 4)
            self.target_ef_pose.pose.position.z = round(self.target_ef_pose.pose.position.z, 4)
            
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
        # if not self.target_ef_pose:
        #     rospy.logwarn("No arm target pose set. Skipping planning and execution.")
        #     return
        # else:
        #     self.arm_group.set_pose_target(self.target_ef_pose)
        #     success, self.target_ef_traj, _, _ = self.arm_group.plan()
        #     if not success:
        #         rospy.logwarn("MoveIt failed to generate a valid trajectory. Stopping execution.")
        #         return
        #     self.arm_group.execute(self.target_ef_traj)
        #     # rospy.loginfo("Executed arm planned trajectory: %s", self.target_ef_traj)

        # if not self.target_gripper_joint:
        #     rospy.logwarn("No arm gripper joint set. Skipping planning and execution.")
        #     return
        # else:
        #     self.gripper_group.set_joint_value_target(self.target_gripper_joint)
        #     success, self.target_gripper_traj, _, _ = self.gripper_group.plan()
        #     if not success:
        #         rospy.logwarn("MoveIt failed to generate a valid trajectory. Stopping execution.")
        #         return
        #     self.gripper_group.execute(self.target_gripper_traj)
        #     # rospy.loginfo("Executed gripper planned trajectory: %s", self.target_gripper_traj)

        # self.arm_group.set_pose_target(self.target_ef_pose)
        self.arm_group.set_named_target("arm_ready")
        success, self.target_ef_traj, _, _ = self.arm_group.plan()
        print(success)
        self.arm_group.execute(self.target_ef_traj, wait=True)
        # rospy.loginfo("Executed planned trajectory: %s", self.target_ef_traj)
        rospy.loginfo("Executed ready pose trajectory")
        current_joint_values = self.arm_group.get_current_joint_values()
        current_joint_values[0] += math.radians(90)  # rotate the first joint by 90 degrees
        self.arm_group.set_joint_value_target(current_joint_values)
        success, traj_plan, _, _ = self.arm_group.plan()
        self.arm_group.stop() # Ensures no residual movement
        self.arm_group.clear_pose_targets()
        if not success:
            rospy.logerr("Failed to plan movement to place rotate pose.")
            return
        self.arm_group.execute(traj_plan, wait=True)
        rospy.loginfo("Executed planned trajectory to place rotate pose ...")

        current_pose = self.arm_group.get_current_pose().pose
        rospy.loginfo(f'Current Pose before placing: {current_pose}')
        place_pose = PoseStamped()
        place_pose.header.stamp = rospy.Time.now()
        place_pose.header.frame_id = "world"  # Use the world frame or the base frame of your robot
        place_pose.pose.position.x = current_pose.position.x
        place_pose.pose.position.y = current_pose.position.y
        place_pose.pose.position.z = current_pose.position.z - 0.10 # Adjust Z position for placing
        # Tetap gunakan orientasi saat ini
        place_pose.pose.orientation = current_pose.orientation

        self.arm_group.set_pose_target(place_pose)
        success, traj_plan, _, _ = self.arm_group.plan()
        self.arm_group.stop() # Ensures no residual movement
        self.arm_group.clear_pose_targets()
        if not success:
            rospy.logerr("Failed to plan movement to place pose.")
            return
        self.arm_group.execute(traj_plan, wait=True)
        rospy.loginfo("Executed planned trajectory to place pose ...")
        self.gripper_group.set_named_target("gripper_big_close")
        success_2, self.target_gripper_traj, _, _ = self.gripper_group.plan()
        if not success_2:
            rospy.logerr("Failed to plan gripper close trajectory.")
            return
        self.gripper_group.execute(self.target_gripper_traj, wait=True)
        rospy.loginfo("Executed gripper ready pose trajectory")
        rospy.sleep(2.0)  # Wait for the gripper to close
        self.gripper_group.set_named_target("gripper_open")
        success_3, self.target_gripper_traj, _, _ = self.gripper_group.plan()
        if not success_3:
            rospy.logerr("Failed to plan gripper open trajectory.")
            return
        self.gripper_group.execute(self.target_gripper_traj, wait=True)
        rospy.loginfo("Executed gripper ready pose trajectory")
        rospy.sleep(2.0)
        self.arm_group.set_named_target("arm_ready")
        success_4, self.target_ef_traj, _, _ = self.arm_group.plan()
        if not success_4:
            rospy.logerr("Failed to plan arm ready pose trajectory.")
            return
        self.arm_group.execute(self.target_ef_traj, wait=True)
        # rospy.loginfo("Executed planned trajectory: %s", self.target_ef_traj)
        rospy.loginfo("Executed ready pose trajectory")

if __name__ == "__main__":
    print("MoveItManipulator script started")
    moveit_manipulator = MoveItManipulator()
    moveit_manipulator.plan_and_execute_pose()