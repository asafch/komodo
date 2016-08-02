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
    processed_image_bw_publisher = None
    offset = 3
    wheel_publisher = None
    state = ""
    ball_at_middle_Y_of_Asus_Camera = False
    ball_at_proper_Y_of_Front_Camera = False
    ball_at_proper_X_of_Front_Camera = False
    ball_positioned = False
    front_camera_x_reference = 0
    front_camera_y_reference = 0

    def __init__(self):
        init_arguments(self)
        self.state = "NO_SEARCH"
        rospy.Subscriber("/pluto/detector/current_camera", String, self.camera_change)
        rospy.Subscriber("/pluto/detector/state_change", String, self.state_change)
        self.bridge = cv_bridge.CvBridge()
        self.processed_image_publisher = rospy.Publisher("/pluto/processed_image", Image, queue_size = 10)
        self.processed_image_bw_publisher = rospy.Publisher("/pluto/processed_image_bw", Image, queue_size = 10)
        self.wheel_publisher = rospy.Publisher("pluto/robot_movement/command", String, queue_size = 10)
        # the image resolution of the front camera varies between the real camera and the simulated one
        self.front_camera_x_reference = 1080 if self.is_simulation else 107
        self.front_camera_y_reference = 1190 if self.is_simulation else 160

    def camera_change(self, command):
        self.current_camera = command.data
        rospy.loginfo("Detector: current camera changed to %s", self.current_camera)
        if self.camera_subscription:
            self.camera_subscription.unregister()
        if self.current_camera == "ASUS_CAMERA":
            self.ball_at_middle_Y_of_Asus_Camera = False
            self.ball_positioned = False
            self.camera_subscription = rospy.Subscriber(adjust_namespace(self.is_simulation, "/Asus_Camera/rgb/image_raw"), Image, self.process_image)
        # elif self.current_camera == "CREATIVE_CAMERA":
        #     self.camera_subscription = rospy.Subscriber(adjust_namespace(self.is_simulation, "/Creative_Camera/rgb/image_raw" if self.is_simulation else "/arm_cam_node/image_raw"), Image, self.process_image)
        elif self.current_camera == "FRONT_CAMERA":
            self.ball_at_proper_Y_of_Front_Camera = False
            self.ball_at_proper_X_of_Front_Camera = False
            self.camera_subscription = rospy.Subscriber(adjust_namespace(self.is_simulation, "/Front_Camera/image_raw" if self.is_simulation else "/front_cam_node/image_raw"), Image, self.process_image)

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
        blurred_image = cv2.GaussianBlur(image_cv, (5, 5), 0)

        (lower, upper) = ([100, 70, 40], [255, 200, 150])

        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(blurred_image, lower, upper)
        output = cv2.bitwise_and(blurred_image, blurred_image, mask = mask)

        (lower, upper) = ([100, 0, 0], [255, 160, 100])

        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(output, lower, upper)
        output = cv2.bitwise_and(output, output, mask = mask)

        bw_image = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        params = cv2.SimpleBlobDetector_Params()
        params.filterByInertia = False
        params.filterByConvexity = True
        params.filterByColor = False # online sources indicate that this feature isn't working properly, hence the manual color filtration used earlier in the code
        params.filterByCircularity = True
        params.filterByArea = True
        params.minArea = 200.0
        params.maxArea = 20000.0
        params.minConvexity = 0.6
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
        processed_image_bw = cv2.drawKeypoints(bw_image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        center = (target[0], target[1])
        cv2.circle(processed_image_bw, center, target[2], (255, 0, 0), 1, 8, 0)
        processed_image = cv2.drawKeypoints(image_cv, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.circle(processed_image, center, target[2], (255, 0, 0), 1, 8, 0)
        # publish the keypoints and target circle superimposed on the source image from the camera and on the b&w image
        self.processed_image_publisher.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
        self.processed_image_bw_publisher.publish(self.bridge.cv2_to_imgmsg(processed_image_bw, "bgr8"))
        rospy.loginfo("x: %d, y: %d, radius: %d", target[0], target[1], target[2])
        if max_circle != None and self.current_camera == "ASUS_CAMERA" and abs(target[0] - (image.width / 2)) < self.offset and not self.ball_at_middle_Y_of_Asus_Camera:
            self.ball_at_middle_Y_of_Asus_Camera = True
            self.wheel_publisher.publish("STOP-BALL_FOUND")
            rospy.loginfo("Detector: ball found")
        elif max_circle != None and self.current_camera == "ASUS_CAMERA" and abs(target[1] - (image.height)) < (image.height / 15.0) and self.ball_at_middle_Y_of_Asus_Camera:
            self.wheel_publisher.publish("STOP-BALL_AT_BOTTOM_OF_FRAME")
            rospy.loginfo("Detector: ball is at bottom of Asus Camera frame")
        # elif max_circle != None and self.current_camera == "FRONT_CAMERA" and abs(target[1] - self.front_camera_y_reference) > self.offset and not self.ball_at_proper_Y_of_Front_Camera:
        #     if target[1] - 1190 > 0:
        #         self.wheel_publisher.publish("BACKWARD")
        #     else:
        #         self.wheel_publisher.publish("FORWARD")
        # elif max_circle != None and self.current_camera == "FRONT_CAMERA" and abs(target[1] - self.front_camera_y_reference) < self.offset and not self.ball_at_proper_Y_of_Front_Camera:
        #     self.ball_at_proper_Y_of_Front_Camera = True
        #     self.wheel_publisher.publish("STOP-BALL_FOUND")
        # elif max_circle != None and self.current_camera == "FRONT_CAMERA" and abs(target[0] - self.front_camera_x_reference) > self.offset and not self.ball_at_proper_X_of_Front_Camera:
        #     if target[0] - 1080 > 0:
        #         self.wheel_publisher.publish("RIGHT")
        #     else:
        #         self.wheel_publisher.publish("LEFT")
        # elif max_circle != None and self.current_camera == "FRONT_CAMERA" and abs(target[0] - self.front_camera_x_reference) < self.offset and not self.ball_at_proper_X_of_Front_Camera and not self.ball_positioned:
        #     self.ball_at_proper_X_of_Front_Camera = True
        #     self.ball_positioned = True
        #     self.wheel_publisher.publish("STOP-BALL_AT_POSITION")

if __name__ == "__main__":
    rospy.init_node("detector")
    detector = Detector()
    rospy.spin()