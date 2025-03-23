#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class WebApp:
    def __init__(self):
        rospy.init_node('custom_web_app', anonymous=True)
        self.web_sub = rospy.Subscriber('/linguistic_cmd', String, self.linguisticc_callback)
        self.cmd_status_pub = rospy.Publisher('/cmd_status', String, queue_size=10)

    def linguisticc_callback(self, msg):
        try:
            rospy.loginfo('Web Command Received: %s', msg.data)
            status_msg = String()
            status_msg.data = f"SUCCESS: Received command '{msg.data}'"
            self.cmd_status_pub.publish(status_msg)
        except Exception as e:
            rospy.logerr(f"Error processing command: {str(e)}")
            status_msg = String()
            status_msg.data = "ERROR: Failed to process command"
            self.cmd_status_pub.publish(status_msg)

if __name__ == "__main__":
    print("WebApp script started")
    web_app = WebApp()
    rospy.spin()
