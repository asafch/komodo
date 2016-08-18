#!/usr/bin/python
import roslib
import rospy
import cv2
import numpy as np
import cv_bridge
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from common import *
from jupiter.msg import BallPosition

class Detector:

    current_camera = None
    camera_subscription = None
    bridge = None
    processed_image_publisher = None
    processed_image_bw_publisher = None
    offset = 100
    wheel_publisher = None
    state = ""
    ball_at_middle_X_of_Asus_Camera = False
    ball_at_proper_Y_of_Front_Camera = False
    ball_at_proper_X_of_Front_Camera = False
    ball_positioned = False
    front_camera_x_reference = 0
    front_camera_y_reference = 0
    move_robot_or_arm = ""
    ball_position = None

    def __init__(self):
        init_arguments(self)
        self.state = "NO_SEARCH"
        rospy.Subscriber("/jupiter/detector/current_camera", String, self.camera_change)
        rospy.Subscriber("/jupiter/detector/state_change", String, self.state_change)
        self.robot_movement_publisher = rospy.Publisher("/jupiter/robot_movement/command", String, queue_size = 10)
        self.state_machine_publisher = rospy.Publisher("/jupiter/robot_movement/result", String, queue_size = 10)
        self.bridge = cv_bridge.CvBridge()
        self.processed_image_publisher = rospy.Publisher("/jupiter/processed_image", Image, queue_size = 10)
        self.processed_image_bw_publisher = rospy.Publisher("/jupiter/processed_image_bw", Image, queue_size = 10)
        self.ball_position_publisher = rospy.Publisher("/jupiter/ball_position", BallPosition, queue_size = 10)
        self.ball_position = BallPosition()
        self.ball_position.detected = False

    def camera_change(self, command):
        self.current_camera = command.data
        rospy.loginfo("Detector: current camera changed to %s", self.current_camera)
        if self.camera_subscription:
            self.camera_subscription.unregister()
        if self.current_camera == "ASUS_CAMERA":
            self.ball_at_middle_X_of_Asus_Camera = False
            self.ball_at_bottom_message_sent = False
            self.ball_positioned = False
            self.offset = 100
            self.camera_subscription = rospy.Subscriber(adjust_namespace(self.is_simulation, "/Asus_Camera/rgb/image_raw"), Image, self.process_image)
        elif self.current_camera == "ARM_CAMERA":
            self.camera_subscription = rospy.Subscriber("/Creative_Camera/rgb/image_raw" if self.is_simulation else "/komodo_1/arm_cam_node/image_raw", Image, self.process_image)
            self.move_robot_or_arm = "MOVE_ROBOT"
        # elif self.current_camera == "FRONT_CAMERA":
        #     self.ball_at_proper_Y_of_Front_Camera = False
        #     self.ball_at_proper_X_of_Front_Camera = False
        #     self.offset = 5
        #     self.camera_subscription = rospy.Subscriber(adjust_namespace(self.is_simulation, "/Front_Camera/image_raw" if self.is_simulation else "/front_cam_node/image_raw"), Image, self.process_image)

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
        blurred_image = cv2.GaussianBlur(image_cv, (9, 9), 0)
        # The two cameras have different sensors, so their color rendition varies. Adjust for this issue when trying to filter the red colors in the image.
        if self.current_camera == "ASUS_CAMERA":
            (lower, upper) = ([0, 0, 100], [55, 55, 255]) # dark red
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            mask = cv2.inRange(blurred_image, lower, upper)
            output = cv2.bitwise_and(blurred_image, blurred_image, mask = mask)
        else: # ARM_CAMERA
            blurred_image2 = cv2.GaussianBlur(image_cv, (9, 9), 0)
            (lower, upper) = ([0, 0, 100], [60, 100, 255])
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            mask = cv2.inRange(blurred_image, lower, upper)
            output_dark_orange = cv2.bitwise_and(blurred_image, blurred_image, mask = mask)
            (lower2, upper2) = ([65, 50, 170], [100, 70, 255])
            lower2 = np.array(lower2, dtype = "uint8")
            upper2 = np.array(upper2, dtype = "uint8")
            mask2 = cv2.inRange(blurred_image2, lower2, upper2)
            output_light_orange = cv2.bitwise_and(blurred_image2, blurred_image2, mask = mask2)
            output = output_light_orange
            cv2.bitwise_or(output_dark_orange, output_light_orange, output)
        image_grayscale = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        (thresh, image_binary) = cv2.threshold(image_grayscale, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        params = cv2.SimpleBlobDetector_Params()
        params.filterByInertia = False
        params.filterByConvexity = True
        params.filterByColor = False
        params.filterByCircularity = True
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 2500 if self.current_camera == "ASUS_CAMERA" else 38400
        params.minConvexity = 0.2
        params.maxConvexity = 1.0
        params.minCircularity = 0.25
        params.maxCircularity = 1.0
        if self.current_camera == "FRONT_CAMERA":
            params.minDistBetweenBlobs = 20.0
        # Create a detector with the parameters, according to your OpenCV version (2 or 3)
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3:
            detector = cv2.SimpleBlobDetector(params)
        else: 
            detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs
        keypoints = detector.detect(image_binary)
        circles = []
        for keypoint in keypoints:
            x = keypoint.pt[0]
            y = keypoint.pt[1]
            r = keypoint.size / 2.0
            circles.append([x, y, r])
        target = None
        if circles:
            circles     = np.uint16(np.around(circles))
            max_r       = 0.0
            target  = circles[0] 
            for circle in circles:
                if circle[2] > max_r and (circle[1] >= (image.height * 0.5) if self.current_camera == "ASUS_CAMERA" else True):
                    max_r       = circle[2]
                    target  = circle
        if target != None:
            processed_image_bw = cv2.drawKeypoints(image_binary, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            center = (target[0], target[1])
            cv2.circle(processed_image_bw, center, target[2], (255, 0, 0), 1, 8, 0)
            processed_image = cv2.drawKeypoints(image_cv, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.circle(processed_image, center, target[2], (255, 0, 0), 1, 8, 0)
            # publish the keypoints and target circle superimposed on the source image from the camera and on the b&w image
            self.processed_image_publisher.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
            self.processed_image_bw_publisher.publish(self.bridge.cv2_to_imgmsg(processed_image_bw, "bgr8"))
            if target[2]:
                rospy.loginfo("x: %d, y: %d, radius: %d", target[0], target[1], target[2])
                # rospy.loginfo("w:%d h:%d", image.width, image.height)

            if self.current_camera == "ASUS_CAMERA" and self.asus_ballpark(target[0], image) and not self.ball_at_middle_X_of_Asus_Camera:
                self.ball_at_middle_X_of_Asus_Camera = True
                self.robot_movement_publisher.publish("STOP-BALL_FOUND")
                rospy.loginfo("Detector: ball found")
            elif target != None and self.current_camera == "ASUS_CAMERA" and abs(target[1] - (image.height)) < (image.height / 10.0) and self.ball_at_middle_X_of_Asus_Camera and not self.ball_at_bottom_message_sent:
                self.ball_at_bottom_message_sent = True
                self.robot_movement_publisher.publish("STOP-BALL_AT_BOTTOM_OF_FRAME")
                # self.state_machine_publisher.publish("BALL_AT_BOTTOM_OF_FRAME")
                rospy.loginfo("Detector: ball is at bottom of Asus Camera frame")
            elif target != None and self.current_camera == "ARM_CAMERA" and self.move_robot_or_arm == "MOVE_ROBOT":
                if self.is_simulation: # the real arm cam emits an upside-down image, so adjust for orientation
                    if target[1] < (image.height / 10.0):
                        if target[0] < image.width * 0.4:
                            self.robot_movement_publisher.publish("FORWARD-LEFT")
                        elif target[0] > image.width * 0.6:
                            self.robot_movement_publisher.publish("FORWARD-RIGHT")
                        else:
                            self.robot_movement_publisher.publish("FORWARD")
                    else:
                        self.move_robot_or_arm = "MOVE_ARM"
                        self.robot_movement_publisher.publish("STOP-READY_TO_GRAB")
                else:
                    if target[1] > (image.height / 10.0):
                        if target[0] < image.width * 0.4:
                            self.robot_movement_publisher.publish("FORWARD-RIGHT")
                        elif target[0] > image.width * 0.6:
                            self.robot_movement_publisher.publish("FORWARD-LEFT")
                        else:
                            self.robot_movement_publisher.publish("FORWARD")
                    else:
                        self.move_robot_or_arm = "MOVE_ARM"
                        self.robot_movement_publisher.publish("STOP-READY_TO_GRAB")
            elif target != None and self.current_camera == "ARM_CAMERA" and self.move_robot_or_arm == "MOVE_ARM":
                rospy.loginfo("Detector: publishing ball position")
                self.ball_position.detected = True
                self.ball_position.x = target[0]
                self.ball_position.y = target[1]
                self.ball_position.radius = target[2]
                self.ball_position.img_width = image.width
                self.ball_position.img_height = image.height
                self.ball_position_publisher.publish(self.ball_position)
                self.state = "NO_SEARCH"
            # elif self.current_camera == "FRONT_CAMERA" and abs(target[0] - self.front_camera_x_reference) > self.offset and not self.ball_at_proper_X_of_Front_Camera and self.previous_state != 2:
            #     if target[0] - self.front_camera_x_reference > 0:
            #         self.wheel_publisher.publish("RIGHT")
            #     else:
            #         self.wheel_publisher.publish("LEFT")
            # elif self.current_camera == "FRONT_CAMERA" and abs(target[0] - self.front_camera_x_reference) < self.offset and not self.ball_at_proper_X_of_Front_Camera and self.previous_state != 2:
            #     self.previous_state = 2
            #     self.ball_at_proper_X_of_Front_Camera = True
            #     self.wheel_publisher.publish("STOP-BALL_FOUND")
            #     # self.ball_positioned = True
            #     # self.wheel_publisher.publish("STOP-BALL_AT_POSITION")
            # elif self.current_camera == "FRONT_CAMERA" and abs(target[1] - self.front_camera_y_reference) > self.offset and not self.ball_at_proper_Y_of_Front_Camera and target[2] <= 8 and self.previous_state != 3:
            #     if target[1] - self.front_camera_y_reference > 0:
            #         self.wheel_publisher.publish("BACKWARD")
            #     else:
            #         self.wheel_publisher.publish("FORWARD")
            # elif self.current_camera == "FRONT_CAMERA" and abs(target[1] - self.front_camera_y_reference) < self.offset and not self.ball_at_proper_Y_of_Front_Camera and not self.ball_positioned and self.previous_state != 3:
            #     self.previous_state = 3
            #     self.ball_at_proper_Y_of_Front_Camera = True
            #     # self.wheel_publisher.publish("STOP-BALL_FOUND")
            #     self.ball_positioned = True
            #     self.wheel_publisher.publish("STOP-BALL_AT_POSITION")
            #     rospy.loginfo("TARGET: %r", target)

    def asus_ballpark(self, x, image):
        return (image.width * 0.65) <= x and x <= (image.width * 0.85)

if __name__ == "__main__":
    rospy.init_node("detector")
    detector = Detector()
    rospy.spin()