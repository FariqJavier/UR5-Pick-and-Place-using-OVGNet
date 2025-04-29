#!/usr/bin/env python3
import rospy
import moveit_commander
import os
import sys
import numpy as np
from std_msgs.msg import Bool

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'src'))

from detecting_utils import generate_joint_space_centered_motion, generate_pose_space_centered_motion

class CustomDetectingMotion:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('custom_detecting_motion', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        # self.eef_link = self.move_group.get_end_effector_link()
        self.reference_frame = "base_link"

        self.status_pub = rospy.Publisher("/start_motion", Bool, queue_size=10)

        self.num_elements= 5 # Number of elements in the motion sequence on each axis

        """ JOINT SPACE INTERPOLATION APPROACH """
        self.shoulder_pan_joint_interval = 0.2 # In radians
        self.shoulder_lift_joint_interval = 0.2 # In radians
        self.elbow_joint_interval = 0.2 # In radians
        self.wrist_1_joint_interval = 0.2 # In radians
        self.wrist_2_interval = 0.2 # In radians
        self.wrist_3_interval = 0.2 # In radians

        """ POSE SPACE INTERPOLATION APPROACH """
        self.cartesian_interval = 0.05 # In meters
        self.target_distance = 0.42 # In meters
        self.axis_of_motion = (1.0, 0.0, 0.0) # Move along world Y-axis
        self.pointing_axis = (0.0, 1.0, 0.0) # End effector pointing X-axis
        self.constraint_axis = (1.0, 0.0, 0.0) # Try to keep X-axis level constraint

    def get_base_joint_values(self):
        """
        Get the current joint values of the robot's arm.
        """
        return self.arm_group.get_current_joint_values()
    
    def get_base_pose(self):
        """
        Get the current pose in list of the robot's end effector.
        """
        current_pose = self.arm_group.get_current_pose().pose
        pose_position = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
        # pose_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        pose_orientation = current_pose.orientation
        return pose_position, pose_orientation
    
    def run_joint_space_detecting_motion(self):
        """
        Executes the motion sequence in joint space.
        """
        try:
            sequence_center = self.get_base_joint_values()
            sequence = generate_joint_space_centered_motion(
                sequence_center, 
                self.num_elements, 
                self.shoulder_pan_joint_interval, 
                self.shoulder_lift_joint_interval,
                self.elbow_joint_interval,
                self.wrist_1_joint_interval,
                self.wrist_2_interval,
                self.wrist_3_interval
            )

            for idx, joint_values in enumerate(sequence):
                self.arm_group.set_joint_value_target(joint_values)
                success, traj_plan, _, _ = self.arm_group.plan()
                self.arm_group.stop() # Ensures no residual movement
                self.arm_group.clear_pose_targets()

                if not success:
                    rospy.logerr("Failed to plan movement for joint values: %s", joint_values)
                    continue

                self.arm_group.execute(traj_plan, wait=True)
                rospy.loginfo("Executed planned trajectory to detecting motion ...")
                self.status_pub.publish(Bool(data=True))
                rospy.loginfo("Taking visual input from angles %d ...", idx)
                rospy.sleep(1)

        except Exception as e:
            rospy.logerr("An error occurred: %s", str(e))

    def run_pose_space_detecting_motion(self):
        """
        Executes the motion sequence in pose space.
        """
        try:
            center_joint_angles = self.get_base_joint_values()
            center_position, center_orientation = self.get_base_pose()
            sequence = generate_pose_space_centered_motion(
                center_position,
                center_orientation, 
                self.num_elements,
                self.axis_of_motion, 
                self.cartesian_interval,
                self.target_distance,
                self.pointing_axis,
                self.constraint_axis
            )

            for idx, pose in enumerate(sequence):
                self.arm_group.set_pose_target(pose)
                success, traj_plan, _, _ = self.arm_group.plan()
                self.arm_group.stop() # Ensures no residual movement
                self.arm_group.clear_pose_targets()

                if not success:
                    rospy.logerr("Failed to plan movement for pose: %s", pose)
                    continue

                self.arm_group.execute(traj_plan, wait=True)
                rospy.loginfo("Executed planned trajectory to detecting motion ...")
                self.status_pub.publish(Bool(data=True))
                rospy.loginfo("Taking visual input from angles %d ...", idx)
                rospy.sleep(1)

        except Exception as e:
            rospy.logerr("An error occurred: %s", str(e))

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
            self.status_pub.publish(Bool(data=False))
            rospy.loginfo("Finish taking visual input from different angles ...")

        except Exception as e:
            rospy.logerr("An error occurred while moving to ready pose: %s", str(e))

if __name__ == "__main__":
    try:
        detecting_motion = CustomDetectingMotion()
        # detecting_motion.run_pose_space_detecting_motion()
        detecting_motion.run_ready_pose()
        detecting_motion.run_joint_space_detecting_motion()
        detecting_motion.run_ready_pose()
        rospy.loginfo("Finished executing detecting motion and moving to ready pose.")
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)