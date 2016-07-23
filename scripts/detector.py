#!/usr/bin/python
import roslib
import rospy
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from pluto.msg import DetectResult
from pluto_common import *

class Detector:

    current_camera = None
    camera_subscription = None
    image_cv = None

    def __init__(self):
        rospy.Subscriber("/pluto/current_camera", String, self.camera_change)

    def camera_change(self, command):
        self.current_camera = command.data
        rospy.loginfo("Detector: current camera changed to %s", self.current_camera)
        if self.camera_subscription:
            self.camera_subscription.unregister()
        if self.current_camera == "ASUS_CAMERA":
            self.camera_subscription = rospy.Subscriber("/Asus_Camera/rgb/image_raw", Image, self.process_image)
        elif self.current_camera == "CREATIVE_CAMERA":
            self.camera_subscription = rospy.Subscriber("/Creative_Camera/rgb/image_raw", Image, self.process_image)

    def process_image(self, image):
        bridge = cv_bridge.CvBridge()
        try:
            self.image_cv = bridge.imgmsg_to_cv2(image, "bgr8")
        except cv_bridge.CvBridgeError, cv_bridge_except:
            rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str(cv_bridge_except))
            return

if __name__ == "__main__":
    rospy.init_node("detector")
    detector = Detector()
    rospy.spin()