#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL

import tf

class RobotPose:
    def __init__(self):
        rospy.init_node('custom_ee_pose', anonymous=True)

        self.arm_joint_names = [ 
            "shoulder_pan_joint",  # Changed order to match UR standard
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.latest_arm_joints = None
        self.pose_pub = rospy.Publisher("/ee_pose", PoseStamped, queue_size=10)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        # Get robot description from parameter server
        robot = URDF.from_parameter_server()
        _, self.kdl_tree = treeFromUrdfModel(robot)
        self.arm_chain = self.kdl_tree.getChain('base_link', 'robotiq_arg2f_base_link')
        
        # Create solver
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.arm_chain)

    def joint_callback(self, msg):
        '''
        Need to reorder joint values to match controller joint order.
        The scaled_pos_joint_traj_controller is expecting joint names ordered as:
        [   "shoulder_pan_joint", 
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"     ]
        But joint_states is giving us joint names in a different order:
        [   "elbow_joint", 
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"     ]
        '''
        if len(msg.position) != 6:
            return
        # self.latest_arm_joints = list(msg.position)

        # Reorder joints to match KDL chain order
        joint_dict = dict(zip(msg.name, msg.position))
        ordered_joints = [joint_dict[name] for name in self.arm_joint_names]
        self.latest_arm_joints = ordered_joints

    def verify_fk_accuracy(self, position, quarternion):
        try:
            
            # Get TF pose for comparison
            self.listener = tf.TransformListener()
            self.listener.waitForTransform('/base_link', '/robotiq_arg2f_base_link', 
                                        rospy.Time(0), rospy.Duration(1.0))
            (trans, quat) = self.listener.lookupTransform('/base_link', 
                                                        '/robotiq_arg2f_base_link', 
                                                        rospy.Time(0))
            
            # # Compare positions
            # pos_diff = [
            #     abs(position.x - trans[0]),
            #     abs(position.y - trans[1]),
            #     abs(position.z - trans[2])
            # ]

            # # Compare orientations
            # quat_diff = [
            #     abs(quarternion.x - quat[0]),
            #     abs(quarternion.y - quat[1]),
            #     abs(quarternion.z - quat[2]),
            #     abs(quarternion.w - quat[3])
            # ]
            
            # rospy.loginfo("Position difference (FK vs TF): [%.4f, %.4f, %.4f]", *pos_diff)
            # rospy.loginfo("Orientation difference (FK vs TF): [%.4f, %.4f, %.4f, %.4f]", *quat_diff)
            
            end_effector_pose = PoseStamped()
            end_effector_pose.header.stamp = rospy.Time.now()
            end_effector_pose.header.frame_id = "base_link"

            end_effector_pose.pose.position.x = trans[0]
            end_effector_pose.pose.position.y = trans[1]
            end_effector_pose.pose.position.z = trans[2]

            end_effector_pose.pose.orientation.x = quat[0]
            end_effector_pose.pose.orientation.y = quat[1]
            end_effector_pose.pose.orientation.z = quat[2]
            end_effector_pose.pose.orientation.w = quat[3]

            rospy.loginfo("Received pose from TF: %s", end_effector_pose)

        except Exception as e:
            rospy.logerr("Verification failed: %s", str(e))
            return False

    def get_end_effector_pose(self):
        if not self.latest_arm_joints:
            rospy.logwarn("No joint states received yet")
            return None

        try:
            # Convert joint angles to KDL JntArray
            joints = PyKDL.JntArray(len(self.latest_arm_joints))
            for i in range(len(self.latest_arm_joints)):
                joints[i] = self.latest_arm_joints[i]

            # Calculate forward kinematics
            end_effector_frame = PyKDL.Frame()
            self.fk_solver.JntToCart(joints, end_effector_frame)
            
            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "base_link"
            
            # Position
            pose_msg.pose.position.x = end_effector_frame.p.x()
            pose_msg.pose.position.y = end_effector_frame.p.y()
            pose_msg.pose.position.z = end_effector_frame.p.z()
            
            # Orientation (convert rotation matrix to quaternion)
            rotation = PyKDL.Rotation(end_effector_frame.M)
            quat = rotation.GetQuaternion()
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            # # Verify FK accuracy
            # self.verify_fk_accuracy(pose_msg.pose.position, pose_msg.pose.orientation)
            
            # # Debug output
            # rospy.loginfo("FK solution:")
            # rospy.loginfo("End-effector Position: [%.4f, %.4f, %.4f]", 
            #               pose_msg.pose.position.x,
            #               pose_msg.pose.position.y,
            #               pose_msg.pose.position.z)
            # rospy.loginfo("End-effector Orientation: [%.4f, %.4f, %.4f]", 
            #               pose_msg.pose.orientation.x,
            #               pose_msg.pose.orientation.y,
            #               pose_msg.pose.orientation.z,
            #               pose_msg.pose.orientation.w)
            
            return pose_msg
            
        except Exception as e:
            rospy.logerr("FK calculation failed: %s", str(e))
            return None

    def publish_pose_on_callback(self):
        while not rospy.is_shutdown():
            pose = self.get_end_effector_pose()
            if not pose:
                rospy.logwarn("No valid pose to publish")
            else:
                self.pose_pub.publish(pose)
                rospy.loginfo("Published End-effector Pose to /ee_pose: %s", pose)
            # rospy.sleep(1/125)
            rospy.sleep(0.1) # Publish every 1 second

if __name__ == "__main__":
    print("RobotPose script started")
    robot_pose = RobotPose()
    robot_pose.publish_pose_on_callback()