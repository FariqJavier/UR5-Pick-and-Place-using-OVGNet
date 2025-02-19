#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def get_end_effector_position(listener):

    # Wait for the transforms to be available
    listener.waitForTransform('/base_link', '/flange', rospy.Time(0), rospy.Duration(4.0))
    listener.waitForTransform('/flange', '/robotiq_arg2f_base_link', rospy.Time(0), rospy.Duration(4.0))

    try:
        # Get the transform from /base_link to /flange
        (trans_base_to_flange, quat_base_to_flange) = listener.lookupTransform('/base_link', '/flange', rospy.Time(0))

        # Get the transform from /flange to /robotiq_arg2f_base_link (gripper)
        (trans_flange_to_gripper, quat_flange_to_gripper) = listener.lookupTransform('/flange', '/robotiq_arg2f_base_link', rospy.Time(0))

        # Combine the transformations to get the gripper's position relative to /base_link
        end_effector_position = Pose()

        # Add translation
        end_effector_position.position.x = trans_base_to_flange[0] + trans_flange_to_gripper[0]
        end_effector_position.position.y = trans_base_to_flange[1] + trans_flange_to_gripper[1]
        end_effector_position.position.z = trans_base_to_flange[2] + trans_flange_to_gripper[2]

        # Multiply quaternions (base_to_flange * flange_to_gripper) to get the final orientation
        quat_combined = tf.transformations.quaternion_multiply(quat_base_to_flange, quat_flange_to_gripper)

        # Set the orientation of the end-effector
        end_effector_position.orientation.x = quat_combined[0]
        end_effector_position.orientation.y = quat_combined[1]
        end_effector_position.orientation.z = quat_combined[2]
        end_effector_position.orientation.w = quat_combined[3]

        return end_effector_position
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Transform not available!")
        return None

if __name__ == "__main__":
    print("RobotPose script started")

    # Initialize the node
    rospy.init_node('custom_robot_pose', anonymous=True)
    # Create a TF listener
    listener = tf.TransformListener()
    # Initialize the publisher
    pose_pub = rospy.Publisher("/robot_pose", Pose, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        pose = get_end_effector_position(listener) 
        pose_pub.publish(pose)
        rospy.loginfo("Published End-effector Pose to /robot_pose: %s", pose)
        rate.sleep()