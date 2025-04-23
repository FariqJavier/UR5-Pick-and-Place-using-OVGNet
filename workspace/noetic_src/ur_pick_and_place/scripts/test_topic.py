#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publisher():
    # Initialize the ROS node
    rospy.init_node('test_publisher', anonymous=True)
    
    # Create a publisher object
    pub = rospy.Publisher('/test_topic', String, queue_size=10)
    
    # Set the publishing rate (10 Hz)
    rate = rospy.Rate(10)
    
    count = 0
    while not rospy.is_shutdown():
        msg = f"Test message {count}"
        rospy.loginfo(f"Publishing: {msg}")
        pub.publish(msg)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
