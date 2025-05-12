#!/usr/bin/env python3
import rospy
import moveit_commander
import os
import sys
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class CustomPickAndPlaceMotion:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('custom_pick_and_place_motion', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        # self.eef_link = self.move_group.get_end_effector_link()
        self.reference_frame = "base_link"

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
        Moves the robot to designed grasping pose.
        """
        try:
            self.arm_group.set_pose_target(pose_msg)
            success, traj_plan, _, _ = self.arm_group.plan()
            self.arm_group.stop() # Ensures no residual movement
            self.arm_group.clear_pose_targets()

            if not success:
                rospy.logerr("Failed to plan movement to picking pose.")
                return

            # self.arm_group.execute(traj_plan, wait=True)
            rospy.loginfo("Executed planned trajectory to picking pose ...")

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