#!/usr/bin/env python3
import rospy
import moveit_commander
import tf
import yaml
import message_filters
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class RobotManipulator:
    def __init__(self):
        rospy.init_node('custom_robot_manipulator', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")

        self.arm_traj_pub = rospy.Publisher("/arm_trajectory", JointTrajectory, queue_size=10)
        
        self.arm_ik_solver = IK("base_link", "flange")
        self.arm_joint_names = [ 
            "elbow_joint",
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # Use message_filters to synchronize topics
        traj_sub = message_filters.Subscriber("/joint_states", JointState)
        pose_sub = message_filters.Subscriber("/robot_pose", PoseStamped)

        self.sync = message_filters.ApproximateTimeSynchronizer([traj_sub, pose_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.synced_callback)
        
    # def arm_traj_callback(self, msg):
    #     self.latest_arm_traj = list(msg.position[:len(self.arm_joint_names)])
    
    # def arm_pose_callback(self, msg):
    #     if not self.latest_arm_traj or len(self.latest_arm_traj) != len(self.arm_joint_names):
    #         rospy.logwarn("Invalid initial joint configuration for IK. Waiting for valid joint states.")
    #         return

    #     latest_arm_pose = msg.pose

    #     # Set a new target position (e.g., move slightly upwards)
    #     target_arm_pose = PoseStamped()
    #     target_arm_pose.header.stamp = rospy.Time.now()
    #     target_arm_pose.header.frame_id = ""

    #     target_arm_pose.pose.position.x = latest_arm_pose.position.x
    #     target_arm_pose.pose.position.y = latest_arm_pose.position.y
    #     target_arm_pose.pose.position.z = latest_arm_pose.position.z + 0.05
    #     target_arm_pose.pose.orientation = latest_arm_pose.orientation  # Keep the same orientation

    #     # Compute inverse kinematics to get joint positions
    #     sol = self.arm_ik_solver.get_ik(
    #         self.latest_arm_traj,
    #         target_arm_pose.pose.position.x, target_arm_pose.pose.position.y, target_arm_pose.pose.position.z,
    #         target_arm_pose.pose.orientation.x, target_arm_pose.pose.orientation.y, target_arm_pose.pose.orientation.z, target_arm_pose.pose.orientation.w
    #     )
        
    #     if sol:
    #         self.target_arm_traj = sol
    #         self.publish_arm_traj()
    #     else:
    #         rospy.logwarn("IK solution not found")

    def publish_arm_traj(self):
        if self.target_arm_traj is None:
            rospy.logwarn("No valid trajectory to publish")
            return

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names

        # Only need pos for /scaled_pos_joint_traj_controller
        point = JointTrajectoryPoint()
        point.positions = self.target_arm_traj
        # point.velocities = [0.0] * len(self.arm_joint_names)
        # point.accelerations = [0.0] * len(self.arm_joint_names)
        point.time_from_start = rospy.Duration(1.0)

        traj_msg.points.append(point)
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = ""

        self.arm_traj_pub.publish(traj_msg)
        rospy.loginfo("Published joint trajectory: %s", traj_msg)

    def synced_callback(self, joint_msg, pose_msg):
        latest_arm_traj = list(joint_msg.position[:len(self.arm_joint_names)])

        if len(latest_arm_traj) != len(self.arm_joint_names):
            rospy.logwarn("Invalid initial joint configuration for IK. Waiting for valid joint states.")
            return

        latest_arm_pose = pose_msg.pose

        # Set a new target position (e.g., move slightly upwards)
        target_arm_pose = PoseStamped()
        target_arm_pose.header.stamp = rospy.Time.now()
        target_arm_pose.header.frame_id = ""

        target_arm_pose.pose.position.x = latest_arm_pose.position.x
        target_arm_pose.pose.position.y = latest_arm_pose.position.y
        target_arm_pose.pose.position.z = latest_arm_pose.position.z + 0.05
        target_arm_pose.pose.orientation = latest_arm_pose.orientation  # Keep the same orientation

        # Compute inverse kinematics to get joint positions
        sol = self.arm_ik_solver.get_ik(
            latest_arm_traj,
            target_arm_pose.pose.position.x, target_arm_pose.pose.position.y, target_arm_pose.pose.position.z,
            target_arm_pose.pose.orientation.x, target_arm_pose.pose.orientation.y, target_arm_pose.pose.orientation.z, target_arm_pose.pose.orientation.w
        )
        
        if sol:
            self.target_arm_traj = sol
            self.publish_arm_traj()
        else:
            rospy.logwarn("IK solution not found")

if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    try:
        print("RobotManipulator script started")
        robot_manipulator = RobotManipulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()