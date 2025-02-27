#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

class MoveItManipulator:
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        rospy.init_node('custom_moveit_manipulator', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.target_arm_pose = None
        self.target_arm_traj = None

    def get_latest_arm_pose(self):
        return self.arm_group.get_current_pose()

    def set_target_arm_pose(self):
        self.target_arm_pose = self.get_latest_arm_pose()
        self.target_arm_pose.pose.position.z += 0.05


    def plan_and_execute(self):
        if not self.target_arm_pose:
            rospy.logwarn("No target pose set. Skipping planning and execution.")
            return

        self.arm_group.set_pose_target(self.target_arm_pose)
        _, self.target_arm_traj, _, _ = self.arm_group.plan()
        self.arm_group.execute(self.target_arm_traj)
        rospy.loginfo("Executed planned trajectory: %s", self.target_arm_traj)

if __name__ == "__main__":
    print("MoveItManipulator script started")
    moveit_manipulator = MoveItManipulator()
    moveit_manipulator.set_target_arm_pose()
    moveit_manipulator.plan_and_execute()