#!/usr/bin/env python3
import rospy
import moveit_commander
import tf
import yaml
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RobotManipulator:
    def __init__(self):
        rospy.init_node('custom_robot_manipulator')
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        
        self.arm_pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, self.arm_pose_callback)
        self.arm_traj_pub = rospy.Publisher("/arm_joint_trajectory", JointTrajectory, queue_size=10)
        
        self.arm_ik_solver = IK("base_link", "flange")
        self.arm_joint_names = [ 
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        
    def arm_pose_callback(self, msg):
        current_arm_pose = msg.pose
        rospy.loginfo("Received end-effector position: %s", current_arm_pose)

        # Set a new target position (e.g., move slightly upwards)
        target_arm_pose = PoseStamped()
        target_arm_pose.header.stamp = rospy.Time.now()
        target_arm_pose.header.frame_id = "base_link"

        target_arm_pose.pose.position.x = current_arm_pose.position.x
        target_arm_pose.pose.position.y = current_arm_pose.position.y
        target_arm_pose.pose.position.z = current_arm_pose.position.z + 0.2  # Move 20cm up
        target_arm_pose.pose.orientation = current_arm_pose.orientation  # Keep the same orientation

        # Compute inverse kinematics to get joint positions
        sol = self.arm_ik_solver.get_ik(
            [0.0] * len(self.arm_joint_names),  # Initial joint seed
            target_arm_pose.pose.position.x, target_arm_pose.pose.position.y, target_arm_pose.pose.position.z,
            target_arm_pose.pose.orientation.x, target_arm_pose.pose.orientation.y, target_arm_pose.pose.orientation.z, target_arm_pose.pose.orientation.w
        )
        
        if sol:
            self.publish_arm_trajectory(sol)
        else:
            rospy.logwarn("IK solution not found")
    
    def publish_arm_trajectory(self, arm_joint_positions):
        traj_msg = JointTrajectory()
        # traj_msg.arm_joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = arm_joint_positions
        point.time_from_start = rospy.Duration(1.0)
        
        traj_msg.points.append(point)
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = "base_link"
        self.arm_traj_pub.publish(traj_msg)
        rospy.loginfo("Published joint trajectory: %s", traj_msg)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    try:
        robotManipulator = RobotManipulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()