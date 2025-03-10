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
        moveit_commander.roscpp_initialize([])
        rospy.init_node('custom_robot_manipulator', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.latest_arm_traj = None
        self.target_arm_traj = None
        self.target_arm_pose = None

        self.arm_traj_pub = rospy.Publisher("/arm_trajectory", JointTrajectory, queue_size=10)
        
        self.arm_traj_sub = rospy.Subscriber("/joint_states", JointState, self.arm_traj_callback)
        self.arm_pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, self.arm_pose_callback)

        self.arm_ik_solver = IK("base_link", "flange")
        self.arm_joint_names = [ 
            "elbow_joint",
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        
    def arm_traj_callback(self, msg):
        if len(msg.position) != 6:
            # rospy.logwarn("Received JointState with unexpected number of joints: %s", msg.name)
            return  # Ignore messages that are not for the UR5 arm
        
        self.latest_arm_traj = list(msg.position)
    
    def arm_pose_callback(self, msg):
        if not self.latest_arm_traj or len(self.latest_arm_traj) != len(self.arm_joint_names):
            rospy.logwarn("Invalid initial joint configuration for IK. Waiting for valid joint states.")
            return

        latest_arm_pose = msg.pose

        # Set a new target position (e.g., move slightly upwards)
        self.target_arm_pose = PoseStamped()
        self.target_arm_pose.header.stamp = rospy.Time.now()
        self.target_arm_pose.header.frame_id = ""

        self.target_arm_pose.pose.position.x = latest_arm_pose.position.x
        self.target_arm_pose.pose.position.y = latest_arm_pose.position.y
        self.target_arm_pose.pose.position.z = latest_arm_pose.position.z + 0.05
        self.target_arm_pose.pose.orientation = latest_arm_pose.orientation  # Keep the same orientation

        # Compute inverse kinematics to get joint positions
        sol = self.arm_ik_solver.get_ik(
            self.latest_arm_traj,
            self.target_arm_pose.pose.position.x, self.target_arm_pose.pose.position.y, self.target_arm_pose.pose.position.z,
            self.target_arm_pose.pose.orientation.x, self.target_arm_pose.pose.orientation.y, self.target_arm_pose.pose.orientation.z, self.target_arm_pose.pose.orientation.w
        )
        
        if sol:
            self.target_arm_traj = list(sol)
            self.publish_arm_traj()
        else:
            rospy.logwarn("IK solution not found")

    def create_arm_traj_msg(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = self.target_arm_traj
        # point.velocities = [0.0] * len(self.arm_joint_names)
        # point.accelerations = [0.0] * len(self.arm_joint_names)
        point.time_from_start = rospy.Duration(1.0)

        traj_msg.points.append(point)
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = ""

        return traj_msg

    def publish_arm_traj(self):
        while not rospy.is_shutdown():
            if self.target_arm_traj is None:
                rospy.logwarn("No valid trajectory to publish")
            else:
                traj_msg = self.create_arm_traj_msg()
                print(self.arm_group.get_current_pose())
                print(self.arm_group.get_current_joint_values())
                print(self.arm_group.get_current_rpy())
                print(self.arm_group.get_current_state())
                # self.arm_traj_pub.publish(traj_msg)
                # rospy.loginfo("Published joint trajectory: %s", traj_msg)
                self.arm_group.set_joint_value_target(self.target_arm_traj)
                success, plan, _, _ = self.arm_group.plan()
                if success: 
                    self.arm_group.execute(plan, wait=True)
                    rospy.loginfo("Executed trajectory using MoveIt!")
                else:
                    rospy.logwarn("Failed to generate a valid plan.")

            rospy.sleep(1) # Publish every 1 second
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    print("RobotManipulator script started")
    robot_manipulator = RobotManipulator()
    robot_manipulator.publish_arm_traj()