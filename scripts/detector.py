#!/usr/bin/python
import roslib
import rospy
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from common import *

class Detector:

    current_camera = None
    camera_subscription = None
    bridge = None
    processed_image_publisher = None
    offset = 3
    wheel_publisher = None
    state = ""
    ball_at_middle_Y_of_Asus_Camera = False
    ball_at_proper_Y_of_Front_Camera = False
    ball_at_proper_X_of_Front_Camera = False
    ball_positioned = False

    def __init__(self):
        init_arguments(self)
        self.state = "NO_SEARCH"
        rospy.Subscriber("/pluto/detector/current_camera", String, self.camera_change)
        rospy.Subscriber("/pluto/detector/state_change", String, self.state_change)
        self.bridge = cv_bridge.CvBridge()
        self.processed_image_publisher = rospy.Publisher("/pluto/processed_image", Image, queue_size = 10)
        self.wheel_publisher = rospy.Publisher("pluto/robot_movement/command", String, queue_size = 10)

    def camera_change(self, command):
        self.current_camera = command.data
        rospy.loginfo("Detector: current camera changed to %s", self.current_camera)
        if self.camera_subscription:
            self.camera_subscription.unregister()
        if self.current_camera == "ASUS_CAMERA":
            self.camera_subscription = rospy.Subscriber(adjust_namespace(self.is_simulation, "/Asus_Camera/rgb/image_raw"), Image, self.process_image)
        elif self.current_camera == "CREATIVE_CAMERA":
            self.camera_subscription = rospy.Subscriber(adjust_namespace(self.is_simulation, "/Creative_Camera/rgb/image_raw"), Image, self.process_image)
        elif self.current_camera == "FRONT_CAMERA":
            self.camera_subscription = rospy.Subscriber(adjust_namespace(self.is_simulation, "/Front_Camera/image_raw"), Image, self.process_image)

    def state_change(self, command):
        if command.data == "SEARCH":
            self.state = "SEARCH"
            rospy.loginfo("Detector: starting to search for ball")
        elif command.data == "NO_SEARCH":
            self.state = "NO_SEARCH"
            rospy.loginfo("Detector: stopped searching for ball")

    def process_image(self, image):
        if self.state == "NO_SEARCH":
            return
        image_cv = self.bridge.imgmsg_to_cv2(image, "bgr8")
        # im = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
        # hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
        # red_lower1 = np.array([120,100,100],np.uint8)
        # red_upper1 = np.array([179,255,255],np.uint8)
        # red_lower2 = np.array([0,100,100],np.uint8)
        # red_upper2 = np.array([10,255,255],np.uint8)
        # mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        # mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        # res1 = cv2.bitwise_and(im,im, mask= mask1)
        # res = cv2.bitwise_and(im,im, mask= mask2)
        # res |= res1
        # im = res
        # im1  = np.array(im, np.uint8)
        # #(thresh, im) = cv2.threshold(im, 20, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        # thresh, bw_im = cv2.threshold(im1, 1, 255, cv2.THRESH_BINARY)
        blurred_image = cv2.GaussianBlur(image_cv, (5, 5), 0)
        # cv2.imshow("blurred_image", blurred_image)
        # cv2.waitKey(0)
        bw_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("bw_image", bw_image)
        # cv2.waitKey(0)
        params = cv2.SimpleBlobDetector_Params()
        im = bw_image
        params.filterByInertia = False
        params.filterByConvexity = True
        params.filterByColor = False
        params.filterByCircularity = True
        params.filterByArea = True
        params.minArea = 20.0
        params.maxArea = 500.0
        params.minConvexity = 0.87
        params.maxConvexity = 1.0
        params.minCircularity = 0.2
        # Create a detector with the parameters, according to your OpenCV version (2 or 3)
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else : 
            detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs
        keypoints = detector.detect(bw_image)
        circles = []
        for keypoint in keypoints:
            x = keypoint.pt[0]
            y = keypoint.pt[1]
            r = keypoint.size / 2.0
            circles.append([x, y, r])
        target = [0, 0, 0]
        max_circle = None
        if circles:
            circles     = np.uint16(np.around(circles))
            max_r       = 0.0
            max_circle  = circles[0] 
            for circle in circles:
                if circle[2] > max_r:
                    max_r       = circle[2]
                    max_circle  = circle
            target = max_circle
        processed_image = cv2.drawKeypoints(image_cv, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        center = (target[0], target[1])
        cv2.circle(processed_image, center, target[2], (255, 0, 0), 1, 8, 0)
        # publish the keypoints and target circle superimposed on the source image from the camera
        self.processed_image_publisher.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
        # rospy.loginfo("x: %d, y: %d, radius: %d", target[0], target[1], target[2])
        if max_circle != None and self.current_camera == "ASUS_CAMERA" and abs(target[0] - (image.width / 2)) < self.offset and not self.ball_at_middle_Y_of_Asus_Camera:
            self.ball_at_middle_Y_of_Asus_Camera = True
            self.wheel_publisher.publish("STOP-BALL_FOUND")
            rospy.loginfo("Detector: ball found")
            # self.state = "NO_SEARCH"
        elif max_circle != None and self.current_camera == "ASUS_CAMERA" and abs(target[1] - (image.height)) < self.offset * 4:
            # self.ball_at_middle_Y_of_Asus_Camera = True
            self.wheel_publisher.publish("STOP-BALL_AT_BOTTOM_OF_FRAME")
            rospy.loginfo("Detector: ball is at bottom of Asus Camera frame")
            # self.state = "NO_SEARCH"
        elif max_circle != None and self.current_camera == "FRONT_CAMERA" and abs(target[1] - 1190) > self.offset and not self.ball_at_proper_Y_of_Front_Camera:
            if target[1] - 1190 > 0:
                self.wheel_publisher.publish("BACKWARD")
            else:
                self.wheel_publisher.publish("FORWARD")
        elif max_circle != None and self.current_camera == "FRONT_CAMERA" and abs(target[1] - 1190) < self.offset and not self.ball_at_proper_Y_of_Front_Camera:
            self.ball_at_proper_Y_of_Front_Camera = True
            self.wheel_publisher.publish("STOP-BALL_FOUND")
        elif max_circle != None and self.current_camera == "FRONT_CAMERA" and abs(target[0] - 1080) > self.offset and not self.ball_at_proper_X_of_Front_Camera:
            if target[0] - 1080 > 0:
                self.wheel_publisher.publish("RIGHT")
            else:
                self.wheel_publisher.publish("LEFT")
        elif max_circle != None and self.current_camera == "FRONT_CAMERA" and abs(target[0] - 1080) < self.offset and not self.ball_at_proper_X_of_Front_Camera and not self.ball_positioned:
            self.ball_at_proper_X_of_Front_Camera = True
            self.ball_positioned = True
            self.wheel_publisher.publish("STOP-BALL_AT_POSITION")

if __name__ == "__main__":
    rospy.init_node("detector")
    detector = Detector()
    rospy.spin()