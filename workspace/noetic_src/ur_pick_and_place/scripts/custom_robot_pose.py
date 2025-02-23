#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
class RobotPose:
    def __init__(self):
        rospy.init_node('custom_robot_pose', anonymous=True)
        self.listener = tf.TransformListener()
        self.pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10)

    def get_end_effector_position(self):
        try:
            self.listener.waitForTransform('/base_link', '/flange', rospy.Time(0), rospy.Duration(4.0))
            self.listener.waitForTransform('/flange', '/robotiq_arg2f_base_link', rospy.Time(0), rospy.Duration(4.0))

            (trans_base_to_flange, quat_base_to_flange) = self.listener.lookupTransform('/base_link', '/flange', rospy.Time(0))
            (trans_flange_to_gripper, quat_flange_to_gripper) = self.listener.lookupTransform('/flange', '/robotiq_arg2f_base_link', rospy.Time(0))

            end_effector_position = PoseStamped()
            end_effector_position.header.stamp = rospy.Time.now()
            end_effector_position.header.frame_id = "base_link"

            end_effector_position.pose.position.x = trans_base_to_flange[0] + trans_flange_to_gripper[0]
            end_effector_position.pose.position.y = trans_base_to_flange[1] + trans_flange_to_gripper[1]
            end_effector_position.pose.position.z = trans_base_to_flange[2] + trans_flange_to_gripper[2]

            quat_combined = tf.transformations.quaternion_multiply(quat_base_to_flange, quat_flange_to_gripper)

            end_effector_position.pose.orientation.x = quat_combined[0]
            end_effector_position.pose.orientation.y = quat_combined[1]
            end_effector_position.pose.orientation.z = quat_combined[2]
            end_effector_position.pose.orientation.w = quat_combined[3]

            return end_effector_position
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Transform not available!")
            return None

    def publish_pose(self):
        while not rospy.is_shutdown():
            pose = self.get_end_effector_position()
            if pose:
                self.pose_pub.publish(pose)
                rospy.loginfo("Published End-effector Pose to /robot_pose: %s", pose)
            self.rate.sleep()

if __name__ == "__main__":
    print("RobotPose script started")

    robot_pose = RobotPose()
    robot_pose.publish_pose()