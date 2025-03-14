#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class RobotPose:
    def __init__(self):
        rospy.init_node('custom_ee_pose', anonymous=True)
        self.listener = tf.TransformListener()
        self.pose_pub = rospy.Publisher("/ee_pose", PoseStamped, queue_size=10)

    def get_end_effector_pose(self):
        try:
            self.listener.waitForTransform('/base_link', '/robotiq_arg2f_base_link', rospy.Time(0), rospy.Duration(4.0))
            (trans, quat) = self.listener.lookupTransform('/base_link', '/robotiq_arg2f_base_link', rospy.Time(0))

            end_effector_position = PoseStamped()
            end_effector_position.header.stamp = rospy.Time.now()
            end_effector_position.header.frame_id = "base_link"

            end_effector_position.pose.position.x = trans[0]
            end_effector_position.pose.position.y = trans[1]
            end_effector_position.pose.position.z = trans[2]

            end_effector_position.pose.orientation.x = quat[0]
            end_effector_position.pose.orientation.y = quat[1]
            end_effector_position.pose.orientation.z = quat[2]
            end_effector_position.pose.orientation.w = quat[3]

            return end_effector_position
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform not available!")
            rospy.logwarn("TF Error: %s. Retrying in 1s...", str(e))
            rospy.sleep(1)

    def publish_pose_on_callback(self):
        while not rospy.is_shutdown():
            pose = self.get_end_effector_pose()
            if not pose:
                rospy.logwarn("No valid pose to publish")
            else:
                self.pose_pub.publish(pose)
                rospy.loginfo("Published End-effector Pose to /robot_pose: %s", pose)
            # rospy.sleep(1/125)
            rospy.sleep(0.1) # Publish every 1 second

if __name__ == "__main__":
    print("RobotPose script started")
    robot_pose = RobotPose()
    robot_pose.publish_pose_on_callback()