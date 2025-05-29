#!/usr/bin/env python3
import rospy
import moveit_commander
import os
import sys
import numpy as np
from std_msgs.msg import Bool
import math
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, OrientationConstraint
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class CustomPickAndPlaceMotion:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('custom_pick_and_place_motion', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.reference_frame = "world"
        # self.eef_link = self.arm_group.get_end_effector_link()  # Ditambahkan

        self.best_grasp_pose_sub = rospy.Subscriber('/best_grasp_pose', PoseStamped, self.best_grasp_pose_callback)
        self.completed_motion_pub = rospy.Publisher('/completed_motion', Bool, queue_size=10)

    def best_grasp_pose_callback(self, msg):
        try:
            rospy.loginfo(f'Current Pose: {self.arm_group.get_current_pose()}')
            rospy.loginfo('Best Grasp Pose Received: %s', msg)
            rospy.loginfo("Executing Picking Motion ...")

            self.run_picking_pose(msg)
            # self.run_ready_pose()
            
            self.completed_motion_pub.publish(Bool(data=False))
            rospy.loginfo("Finish Pick and Place motion for an object ...")
        except Exception as e:
            rospy.logerr(f"Error processing command: {str(e)}")

    def run_picking_pose(self, pose_msg):
        """
        Moves the robot to designed grasping pose with orientation constraint.
        """
        try:
            # # 1. Buat orientation constraint agar gripper menghadap ke bawah (Z gripper searah -Z world)
            # oc = OrientationConstraint()
            # oc.header.frame_id = self.reference_frame
            # oc.link_name = self.arm_group.get_end_effector_link() 

            # # Quaternion rotasi 180 derajat di sumbu X → arah Z menghadap ke bawah
            # qx, qy, qz, qw = quaternion_from_euler(3.14, 0, 0)
            # oc.orientation.x = qx
            # oc.orientation.y = qy
            # oc.orientation.z = qz
            # oc.orientation.w = qw

            # oc.absolute_x_axis_tolerance = 1.57  # 90 derajat
            # oc.absolute_y_axis_tolerance = 1.57
            # oc.absolute_z_axis_tolerance = 3.14  # Boleh rotasi penuh di Z
            # oc.weight = 1.0

            # constraints = Constraints()
            # constraints.orientation_constraints.append(oc)

            # # 2. Set constraint ke move group
            # self.arm_group.set_path_constraints(constraints)

            # 3. Set target pose dan plan

            # Test gerakan orientasi ke posisi picking
            # Convert the target quaternion (orientation_target) to roll, pitch, yaw
            target_roll, target_pitch, target_yaw = euler_from_quaternion([
                pose_msg.pose.orientation.x, 
                pose_msg.pose.orientation.y, 
                pose_msg.pose.orientation.z, 
                pose_msg.pose.orientation.w
            ])
            target_yaw -= math.radians(33)  # Tambahkan rotasi 180 derajat pada yaw
            current_roll, current_pitch, current_yaw = euler_from_quaternion([
                self.arm_group.get_current_pose().pose.orientation.x, 
                self.arm_group.get_current_pose().pose.orientation.y, 
                self.arm_group.get_current_pose().pose.orientation.z, 
                self.arm_group.get_current_pose().pose.orientation.w
            ])
            # Roll Motion
            # roll_pose_stamped = PoseStamped()
            # roll_pose_stamped.header.stamp = rospy.Time.now()
            # roll_pose_stamped.header.frame_id = self.reference_frame
            # roll_pose_stamped.pose.position = self.arm_group.get_current_pose().pose.position
            # # Convert roll to quaternion and set it
            # roll_quat = quaternion_from_euler(roll, 0, 0)
            # roll_pose_stamped.pose.orientation.x = roll_quat[0]
            # roll_pose_stamped.pose.orientation.y = roll_quat[1]
            # roll_pose_stamped.pose.orientation.z = roll_quat[2]
            # roll_pose_stamped.pose.orientation.w = roll_quat[3]
            # rospy.loginfo(f'Executing Pose (Orientation: Roll): {roll_pose_stamped}')
            # self.arm_group.set_pose_target(roll_pose_stamped)
            # success, traj_plan, _, _ = self.arm_group.plan()
            # self.arm_group.stop()
            # self.arm_group.clear_pose_targets()
            # # self.arm_group.clear_path_constraints()  # 4. Hapus constraint setelah plan
            # if not success:
            #     rospy.logerr("Failed to plan movement to picking pose (Orientation: Roll).")
            #     return
            # self.arm_group.execute(traj_plan, wait=True)
            # rospy.loginfo("Executed planned trajectory to picking pose (Orientation: Roll) ...")

            # # Pitch Motion
            # pitch_pose_stamped = PoseStamped()
            # pitch_pose_stamped.header.stamp = rospy.Time.now()
            # pitch_pose_stamped.header.frame_id = self.reference_frame
            # pitch_pose_stamped.pose.position = self.arm_group.get_current_pose().pose.position
            # # Convert roll to quaternion and set it
            # pitch_quat = quaternion_from_euler(0, pitch, 0)
            # pitch_pose_stamped.pose.orientation.x = pitch_quat[0]
            # pitch_pose_stamped.pose.orientation.y = pitch_quat[1]
            # pitch_pose_stamped.pose.orientation.z = pitch_quat[2]
            # pitch_pose_stamped.pose.orientation.w = pitch_quat[3]
            # rospy.loginfo(f'Executing Pose (Orientation: Pitch): {pitch_pose_stamped}')
            # self.arm_group.set_pose_target(pitch_pose_stamped)
            # success, traj_plan, _, _ = self.arm_group.plan()
            # self.arm_group.stop()
            # self.arm_group.clear_pose_targets()
            # # self.arm_group.clear_path_constraints()  # 4. Hapus constraint setelah plan
            # if not success:
            #     rospy.logerr("Failed to plan movement to picking pose (Orientation: Pitch).")
            #     return
            # self.arm_group.execute(traj_plan, wait=True)
            # rospy.loginfo("Executed planned trajectory to picking pose (Orientation: Pitch) ...")

            # Yaw Motion
            yaw_pose_stamped = PoseStamped()
            yaw_pose_stamped.header.stamp = rospy.Time.now()
            yaw_pose_stamped.header.frame_id = self.reference_frame
            yaw_pose_stamped.pose.position = self.arm_group.get_current_pose().pose.position
            # Convert yaw to quaternion and set it
            yaw_quat = quaternion_from_euler(current_roll, current_pitch, target_yaw)
            yaw_pose_stamped.pose.orientation.x = yaw_quat[0]
            yaw_pose_stamped.pose.orientation.y = yaw_quat[1]
            yaw_pose_stamped.pose.orientation.z = yaw_quat[2]
            yaw_pose_stamped.pose.orientation.w = yaw_quat[3]
            rospy.loginfo(f'Executing Pose (Orientation: Yaw): {yaw_pose_stamped}')
            self.arm_group.set_pose_target(yaw_pose_stamped)
            success, traj_plan, _, _ = self.arm_group.plan()
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            # self.arm_group.clear_path_constraints()  # 4. Hapus constraint setelah plan
            if not success:
                rospy.logerr("Failed to plan movement to picking pose (Orientation: Yaw).")
                return
            self.arm_group.execute(traj_plan, wait=True)
            rospy.loginfo("Executed planned trajectory to picking pose (Orientation: Yaw) ...")

            # Gerakan translasi ke posisi picking
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.reference_frame
            pose_stamped.pose.position.x = pose_msg.pose.position.x
            pose_stamped.pose.position.y = pose_msg.pose.position.y
            pose_stamped.pose.position.z = pose_msg.pose.position.z
            pose_stamped.pose.orientation = self.arm_group.get_current_pose().pose.orientation

            rospy.loginfo(f'Executing Pose (Translation): {pose_stamped}')

            self.arm_group.set_pose_target(pose_stamped)
            success, traj_plan, _, _ = self.arm_group.plan()

            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            # self.arm_group.clear_path_constraints()  # 4. Hapus constraint setelah plan

            if not success:
                rospy.logerr("Failed to plan movement to picking pose (Translation).")
                return

            self.arm_group.execute(traj_plan, wait=True)
            rospy.loginfo("Executed planned trajectory to picking pose (Translation) ...")

            # self.gripper_group.set_named_target("gripper_big_close")
            # success, traj_plan, _, _ = self.arm_group.plan()
            # self.gripper_group.stop() # Ensures no residual movement
            # self.gripper_group.clear_pose_targets()

            # if not success:
            #     rospy.logerr("Failed to plan movement closing gripper.")
            #     return

            # self.gripper_group.execute(traj_plan, wait=True)
            # rospy.loginfo("Executed planned trajectory to Big Closing Gripper ...")

        except Exception as e:
            rospy.logerr("An error occurred while moving to picking pose: %s", str(e))

    def run_ready_pose(self):
        """
        Moves the robot to a ready pose.
        """
        try:
            self.arm_group.set_named_target("arm_ready")
            success, traj_plan, _, _ = self.arm_group.plan()
            self.arm_group.stop() # Ensures no residual movement
            self.arm_group.clear_pose_targets()

            if not success:
                rospy.logerr("Failed to plan movement to ready pose.")
                return

            self.arm_group.execute(traj_plan, wait=True)
            rospy.loginfo("Executed planned trajectory to ready pose ...")

        except Exception as e:
            rospy.logerr("An error occurred while moving to ready pose: %s", str(e))

if __name__ == "__main__":
    try:
        print("Custom Pick And Place Motion script started")
        CustomPickAndPlaceMotion()
        rospy.spin()
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)