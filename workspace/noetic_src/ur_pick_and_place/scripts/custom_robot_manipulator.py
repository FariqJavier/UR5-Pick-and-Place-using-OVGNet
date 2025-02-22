#!/usr/bin/env python3
import rospy
import moveit_commander
import tf
import yaml
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class InverseKinematics:
    def __init__(self):
        rospy.init_node('custom_robot_manipulator')
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        
        self.pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, self.pose_callback)
        self.traj_pub = rospy.Publisher("/arm_joint_trajectory", JointTrajectory, queue_size=10)
        
        # self.load_configuration()
        
        self.ik_solver = IK("base_link", "flange")
        self.joint_names = [ 
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        
    # def load_configuration(self):
    #     with open('~/catkin_ws/src/ur_pick_and_place/config/ur5/joint_limits.yaml', 'r') as file:
    #         self.joint_limits = yaml.safe_load(file)
    #     with open('~/catkin_ws/src/ur_pick_and_place/config/ur5//physical_parameters.yaml', 'r') as file:
    #         self.physical_params = yaml.safe_load(file)
    #     with open('~/catkin_ws/src/ur_pick_and_place/config/ur5/calibration.yaml', 'r') as file:
    #         self.kinematics = yaml.safe_load(file)
    #     with open('~/catkin_ws/src/ur_pick_and_place/config/ur5/controllers.yaml', 'r') as file:
    #         self.controllers = yaml.safe_load(file)
        
    def pose_callback(self, msg):
        current_pose = msg.pose
        rospy.loginfo("Received end-effector position: %s", current_pose)

        # Set a new target position (e.g., move slightly upwards)
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = current_pose.position.x
        target_pose.pose.position.y = current_pose.position.y
        target_pose.pose.position.z = current_pose.position.z + 0.2  # Move 20cm up
        target_pose.pose.orientation = current_pose.orientation  # Keep the same orientation

        # Compute inverse kinematics to get joint positions
        sol = self.ik_solver.get_ik(
            [0.0] * len(self.joint_names),  # Initial joint seed
            target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
            target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w
        )
        
        if sol:
            self.publish_trajectory(sol)
        else:
            rospy.logwarn("IK solution not found")
    
    def publish_trajectory(self, joint_positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(1.0)
        
        traj_msg.points.append(point)
        traj_msg.header.stamp = rospy.Time.now()
        self.traj_pub.publish(traj_msg)
        rospy.loginfo("Published joint trajectory: %s", traj_msg)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    try:
        node = InverseKinematics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()