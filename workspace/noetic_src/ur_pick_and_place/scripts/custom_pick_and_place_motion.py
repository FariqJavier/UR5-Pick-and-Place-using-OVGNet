#!/usr/bin/env python3
import rospy
import moveit_commander
import os
import sys
import numpy as np
from std_msgs.msg import Bool
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

            # self.run_picking_pose(msg)
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
            yaw_pose_stamped = PoseStamped()
            yaw_pose_stamped.header.stamp = rospy.Time.now()
            yaw_pose_stamped.header.frame_id = self.reference_frame
            # Set the current position (keep the same position as the current pose)
            yaw_pose_stamped.pose.position = self.arm_group.get_current_pose().pose.position

            # Apply a 90-degree yaw rotation (90 degrees = π/2 radians)
            yaw_angle = 90  # degrees
            yaw_radians = yaw_angle * (3.14159 / 180)  # Convert to radians

            # Convert the yaw rotation to a quaternion (roll=0, pitch=0, yaw=90 degrees)
            quaternion = quaternion_from_euler(0, 0, yaw_radians)

            # Set the orientation (quaternion) of the pose
            yaw_pose_stamped.pose.orientation.x = quaternion[0]
            yaw_pose_stamped.pose.orientation.y = quaternion[1]
            yaw_pose_stamped.pose.orientation.z = quaternion[2]
            yaw_pose_stamped.pose.orientation.w = quaternion[3]

            # Log the pose
            rospy.loginfo(f'Executing Pose (Orientation: Yaw 90 degrees): {yaw_pose_stamped}')
            # Set the target pose for the robot
            self.arm_group.set_pose_target(yaw_pose_stamped)
            # Plan and execute the movement
            success, traj_plan, _, _ = self.arm_group.plan()
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            if not success:
                rospy.logerr("Failed to plan movement to 90-degree yaw pose.")
                return
            self.arm_group.execute(traj_plan, wait=True)
            rospy.loginfo("Executed planned trajectory to 90-degree yaw pose ...")

            # Test gerakan translasi ke posisi picking
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

            # # Test gerakan orientasi ke posisi picking
            # # Convert the target quaternion (orientation_target) to roll, pitch, yaw
            # roll, pitch, yaw = euler_from_quaternion([pose_msg.pose.orientation.x, 
            #                                         pose_msg.pose.orientation.y, 
            #                                         pose_msg.pose.orientation.z, 
            #                                         pose_msg.pose.orientation.w])
            # # Roll Motion
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

            # # Yaw Motion
            # yaw_pose_stamped = PoseStamped()
            # yaw_pose_stamped.header.stamp = rospy.Time.now()
            # yaw_pose_stamped.header.frame_id = self.reference_frame
            # yaw_pose_stamped.pose.position = self.arm_group.get_current_pose().pose.position
            # # Convert yaw to quaternion and set it
            # yaw_quat = quaternion_from_euler(0, 0, yaw)
            # yaw_pose_stamped.pose.orientation.x = yaw_quat[0]
            # yaw_pose_stamped.pose.orientation.y = yaw_quat[1]
            # yaw_pose_stamped.pose.orientation.z = yaw_quat[2]
            # yaw_pose_stamped.pose.orientation.w = yaw_quat[3]
            # rospy.loginfo(f'Executing Pose (Orientation: Yaw): {yaw_pose_stamped}')
            # self.arm_group.set_pose_target(yaw_pose_stamped)
            # success, traj_plan, _, _ = self.arm_group.plan()
            # self.arm_group.stop()
            # self.arm_group.clear_pose_targets()
            # # self.arm_group.clear_path_constraints()  # 4. Hapus constraint setelah plan
            # if not success:
            #     rospy.logerr("Failed to plan movement to picking pose (Orientation: Yaw).")
            #     return
            # self.arm_group.execute(traj_plan, wait=True)
            # rospy.loginfo("Executed planned trajectory to picking pose (Orientation: Yaw) ...")

            self.gripper_group.set_named_target("gripper_big_close")
            success, traj_plan, _, _ = self.arm_group.plan()
            self.gripper_group.stop() # Ensures no residual movement
            self.gripper_group.clear_pose_targets()

            if not success:
                rospy.logerr("Failed to plan movement closing gripper.")
                return

            self.gripper_group.execute(traj_plan, wait=True)
            rospy.loginfo("Executed planned trajectory to Big Closing Gripper ...")

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