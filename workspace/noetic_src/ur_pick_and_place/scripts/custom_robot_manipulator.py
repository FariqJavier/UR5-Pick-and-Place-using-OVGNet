#!/usr/bin/env python3
import rospy
import moveit_commander
import tf
import yaml
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class IKNode:
    def __init__(self):
        rospy.init_node('ik_solver_node')
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
        
        self.pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, self.pose_callback)
        self.traj_pub = rospy.Publisher("/arm_joint_trajectory", JointTrajectory, queue_size=10)
        
        self.load_configuration()
        
        self.ik_solver = IK("base_link", "tool0")
        self.joint_names = self.arm_group.get_joints()
        
    def load_configuration(self):
        with open('/mnt/data/joint_limits.yaml', 'r') as file:
            self.joint_limits = yaml.safe_load(file)
        with open('/mnt/data/physical_parameters.yaml', 'r') as file:
            self.physical_params = yaml.safe_load(file)
        with open('/mnt/data/calibration.yaml', 'r') as file:
            self.kinematics = yaml.safe_load(file)
        with open('/mnt/data/controllers.yaml', 'r') as file:
            self.controllers = yaml.safe_load(file)
        
    def pose_callback(self, msg):
        target_pose = msg.pose
        sol = self.ik_solver.get_ik(
            [0.0] * len(self.joint_names),  # Initial joint seed
            target_pose.position.x, target_pose.position.y, target_pose.position.z,
            target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w
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

if __name__ == '__main__':
    moveit_commander.roscpp_initialize([])
    try:
        node = IKNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()