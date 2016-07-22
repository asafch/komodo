#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv_bridge
from sensor_msgs.msg import Image

def callback(data):
    rospy.loginfo("------------------CALLBACK-------------------------")
    bridge = cv_bridge.CvBridge()
    try:
        arm_camera_image_cv = bridge.imgmsg_to_cv2( data, "bgr8" )
    except cv_bridge.CvBridgeError, cv_bridge_except:
        rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str( cv_bridge_except ))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/Creative_Camera/rgb/image_raw", Image, callback)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()