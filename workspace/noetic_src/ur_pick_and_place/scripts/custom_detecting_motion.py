#!/usr/bin/env python3
import rospy
import moveit_commander
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'src'))

from detecting_utils import generate_centered_motion_sequence

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
        self.shoulder_pan_joint_interval = 0.1 # In radians
        self.wrist_2_interval = 0.1 # In radians
        self.wrist_3_interval = 0.05 # In radians
        self.num_elements= 7

    def get_base_joint_values(self):
        print("Getting base joint values...")
        # Get the current joint values of the arm group
        return self.arm_group.get_current_joint_values()
    
    def run_detecting_motion(self):
        """
        Executes the motion sequence.
        """
        try:
            sequence_center = self.get_base_joint_values()
            sequence = generate_centered_motion_sequence(
                sequence_center, 
                self.num_elements, 
                self.shoulder_pan_joint_interval, 
                self.wrist_2_interval,
                self.wrist_3_interval
            )

            for joint_values in sequence:
                self.arm_group.set_joint_value_target(joint_values)
                success, traj_plan, _, _ = self.arm_group.plan()

                if not success:
                    rospy.logerr("Failed to plan movement for joint values: %s", joint_values)
                    continue

                self.arm_group.execute(traj_plan, wait=True)
                rospy.loginfo("Executed planned trajectory to detecting motion: %s", traj_plan)
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

            if not success:
                rospy.logerr("Failed to plan movement to ready pose.")
                return

            self.arm_group.execute(traj_plan, wait=True)
            rospy.loginfo("Executed planned trajectory to ready pose: %s", traj_plan)

        except Exception as e:
            rospy.logerr("An error occurred while moving to ready pose: %s", str(e))

if __name__ == "__main__":
    try:
        detecting_motion = CustomDetectingMotion()
        detecting_motion.run_detecting_motion()
        detecting_motion.run_ready_pose()
        rospy.loginfo("Finished executing detecting motion and moving to ready pose.")
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)