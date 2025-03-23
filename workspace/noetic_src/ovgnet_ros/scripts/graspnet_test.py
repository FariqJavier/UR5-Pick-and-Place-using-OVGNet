#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import torch

class GraspNetNode:
    def __init__(self):
        rospy.init_node('graspnet_node')
        self.bridge = CvBridge()
        
        # Verify CPU-only PyTorch
        rospy.loginfo(f"PyTorch CUDA available: {torch.cuda.is_available()}")
        rospy.loginfo(f"Using device: {'cuda' if torch.cuda.is_available() else 'cpu'}")
        
        # Subscribers
        self.color_sub = rospy.Subscriber(
            '/camera/color/image_raw', 
            Image, 
            self.color_callback,
            queue_size=1
        )
        self.depth_sub = rospy.Subscriber(
            '/camera/depth/image_rect_raw', 
            Image, 
            self.depth_callback,
            queue_size=1
        )
        self.camera_info_sub = rospy.Subscriber(
            '/camera/color/camera_info',
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        # Class variables
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def color_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frames()
        except Exception as e:
            rospy.logerr(f"Color callback error: {e}")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg)
            self.process_frames()
        except Exception as e:
            rospy.logerr(f"Depth callback error: {e}")

    def process_frames(self):
        if self.latest_rgb is None or self.latest_depth is None:
            return

        # Add your GraspNet processing here
        # This will run when both RGB and depth frames are available
        pass

if __name__ == '__main__':
    try:
        node = GraspNetNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass