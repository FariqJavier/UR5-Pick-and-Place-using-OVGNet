#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory

class CustomArmController:
    def __init__(self):
        rospy.init_node('custom_arm_controller', anonymous=True)
        self.arm_joint_sub = rospy.Subscriber("/arm_joint_trajectory", JointTrajectory, self.arm_joint_callback)
        self.arm_joint_pub = rospy.Publisher("/scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        # self.rate = rospy.Rate(10)  # Publish at 10Hz

    def arm_joint_callback(self, msg):
        joint_msg = msg
        rospy.loginfo("Received arm joint value: %s", msg)
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.header.frame_id = "base_link"

        self.arm_joint_pub.publish(joint_msg)
        # self.rate.sleep()

if __name__ == "__main__":
    try:
        print("CustomArmController script started")
        custom_arm_controller = CustomArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass