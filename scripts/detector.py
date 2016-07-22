#!/usr/bin/env python

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
    sample_counter                      = 0
    top_camera_image_cv                 = 0
    arm_camera_image_cv                 = 0
    active_top_camera_image_capture     = True
    active_arm_camera_image_capture     = True
    detect_result_publisher             = 0
    is_simulation                       = False
    request_tag                         = ""
    request_pending                     = False
    
    def DETECTOR_RATE( self ):
        return 5
        
    def detector_image_width( self ):
        
        if "scan_top" == self.request_tag:
            
            return 640
            
        elif "scan_arm" == self.request_tag:

            if True == self.is_simulation:
                return 640
            else:
                return 640 // 2
                
        
    def detector_image_height( self ):
        if "scan_top" == self.request_tag:
            
            return 480
            
        elif "scan_arm" == self.request_tag:

            if True == self.is_simulation:
                return 480
            else:
                return 480 // 2
                
        
    def __init__( self ):
        rospy.loginfo( "Detector innitialized " )
        
        init_arguments( self )
        
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, "/Asus_Camera/rgb/image_raw"     ), Image, self.top_camera_listener_cb)
        
        if True == self.is_simulation:
            arm_camera_topic = "/Creative_Camera/rgb/image_raw"
        else:
            arm_camera_topic = "/arm_cam_node/image_raw"
        
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, arm_camera_topic ), Image, self.arm_camera_listener_cb)
        
        rospy.Subscriber("/pluto/detect/command", String, self.get_ready_detect_ball )
        self.detect_result_publisher = rospy.Publisher('/pluto/detect/result', DetectResult, queue_size=10 )
        

    def top_camera_listener_cb( self, rgb_image ):
        if True == self.active_top_camera_image_capture:
            
            #rospy.loginfo( "top_camera_listener_cb, {}".format(self.sample_counter) )
            bridge = cv_bridge.CvBridge()

            try:
                self.top_camera_image_cv = bridge.imgmsg_to_cv2( rgb_image, "bgr8" )

            except cv_bridge.CvBridgeError, cv_bridge_except:
                rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str( cv_bridge_except ))
                return
            
            #if 10 == self.sample_counter:
            #    cv2.imshow( "rgb", self.top_camera_image_cv )
            #    cv2.waitKey()
            
            self.sample_counter = self.sample_counter + 1
            
            if (True == self.request_pending) and ("scan_top" == self.request_tag):
                self.request_pending = False
                self.detect_ball()
            
    def arm_camera_listener_cb( self, rgb_image ):
        if True == self.active_arm_camera_image_capture:
            
            #rospy.loginfo( "top_camera_listener_cb, {}".format(self.sample_counter) )
            bridge = cv_bridge.CvBridge()

            try:
                self.arm_camera_image_cv = bridge.imgmsg_to_cv2( rgb_image, "bgr8" )

            except cv_bridge.CvBridgeError, cv_bridge_except:
                rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str( cv_bridge_except ))
                return
            
            #if 10 == self.sample_counter:
            #    cv2.imshow( "rgb", self.top_camera_image_cv )
            #    cv2.waitKey()
            
            self.sample_counter = self.sample_counter + 1
            
            if (True == self.request_pending) and ("scan_arm" == self.request_tag):
                self.request_pending = False
                self.detect_ball()

    def find_the_ball( self, image_cv, min_radius = 2, max_radius = 100, canny_higher_threshold = 100, canny_accumulator_threshold = 4, debug = False, hide_top_pixels = 0 ):
        # return: 
        #           1) is_ball_found - boolean 
        #           2) center_coordinates_and_radius - tuple (x,y,r)
        # Algorithm: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
        
        image_height, image_width = image_cv.shape[:2]
        
        if hide_top_pixels > 0:
            # cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) -> None
            cv2.rectangle( image_cv, (0,0), (image_width,hide_top_pixels), (0,0,0), -1 )

        hsv_image = cv2.cvtColor( image_cv, cv2.COLOR_BGR2HSV );

        red_hue_range_lower = cv2.inRange( hsv_image, (0, 100, 100), (10, 255, 255) )
        red_hue_range_upper = cv2.inRange( hsv_image, (160, 100, 100), (179, 255, 255) )

        red_hue_image = cv2.addWeighted( red_hue_range_lower, 1.0, red_hue_range_upper, 1.0, 0.0 );
        
        if True == debug:
            cv2.imshow( "red_hue_image", red_hue_image )
            cv2.waitKey()
        
        red_hue_range_blured = cv2.GaussianBlur(red_hue_image, (9, 9), 2, 2);
        
        print( image_height/8, canny_higher_threshold, canny_accumulator_threshold, min_radius, max_radius )

        # http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=houghcircles#houghcircles
        '''
        HoughCircles
            Finds circles in a grayscale image using the Hough transform.
            The function finds circles in a grayscale image using a modification of the Hough transform.
            
        Note: Usually the function detects the centers of circles well. 
            However, it may fail to find correct radii. You can assist to the 
            function by specifying the radius range ( minRadius and 
            maxRadius ) if you know it. Or, you may ignore the returned 
            radius, use only the center, and find the correct radius using an 
            additional procedure.
            
        Python: cv2.HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) -> circles
        
        Parameters: 
            image - 8-bit, single-channel, grayscale input image.
            circles - Output vector of found circles. Each vector is encoded as a 3-element floating-point vector  (x, y, radius) .
            circle_storage - In C function this is a memory storage that will contain the output sequence of found circles.
            method - Detection method to use. Currently, the only implemented method is CV_HOUGH_GRADIENT , which is basically 21HT , described in [Yuen90].
            dp - Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
            minDist - Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
            param1 - First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
            param2 - Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
            minRadius - Minimum circle radius.
            maxRadius - Maximum circle radius.
        '''
        
        circles = cv2.HoughCircles( red_hue_range_blured, cv2.cv.CV_HOUGH_GRADIENT, 1, image_height/8, param1=canny_higher_threshold,param2=canny_accumulator_threshold,minRadius=min_radius,maxRadius=max_radius)


        #if True == debug:
        #    cv2.imshow( "red_hue_range_blured", red_hue_range_blured )
        #    cv2.waitKey()

        is_ball_found                   = circles is not None
        center_coordinates_and_radius   = (0, 0, 0) 

        if True == is_ball_found:
            circles     = np.uint16(np.around(circles))
            max_r       = 0
            max_circle  = circles[0][0] 
            
            for circle in circles[0,:]:
                if circle[2] > max_r:
                    max_r       = circle[2]
                    max_circle  = circle
            
            print max_circle

            if True == debug:
                # draw the outer circle
                cv2.circle(image_cv,(max_circle[0],max_circle[1]),max_circle[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(image_cv,(max_circle[0],max_circle[1]),2,(0,0,255),3)
            
                cv2.imshow( "detected ball", image_cv )
                cv2.waitKey()
                
            center_coordinates_and_radius = max_circle

            
            
        return is_ball_found, center_coordinates_and_radius

    def get_ready_detect_ball( self, string_command ):
        self.request_tag        = string_command.data
        self.request_pending    = True

    def detect_ball( self ):
        
        detect_result = DetectResult()
        detect_result.is_ball_detected  = False
        detect_result.request_tag       = self.request_tag
        
        if "scan_top" == detect_result.request_tag:
            working_image_cv = self.top_camera_image_cv
            
            
            max_radius                      = 25
            hide_top_pixels                 = 190
            
            if True == self.is_simulation:
                min_radius                  = 1
                canny_accumulator_threshold = 2
                canny_higher_threshold      = 100
            else:
                min_radius                  = 2
                canny_accumulator_threshold = 12
                canny_higher_threshold      = 200
            
        elif "scan_arm" == detect_result.request_tag:
            working_image_cv = self.arm_camera_image_cv
            
            max_radius                      = 150
            hide_top_pixels                 = 0
            
            if True == self.is_simulation:
                min_radius                  = 3
                canny_accumulator_threshold = 2
                canny_higher_threshold      = 100
            else:
                min_radius                  = 3
                canny_accumulator_threshold = 12
                canny_higher_threshold      = 200
            
        else:
            raise Exception('Unsupported command to detector, support only: \"scan_top\" or \"scan_arm\".')
        
        if self.sample_counter > 0:
            
            image_height, image_width = working_image_cv.shape[:2]
            
            #rospy.loginfo( "image_width: {}, image_height: {}".format( image_width, image_height ) )
            
            process_image = ( image_width == self.detector_image_width() ) and ( image_height == self.detector_image_height() )
            
            if True == process_image:
                is_ball_found, center_coordinates_and_radius = self.find_the_ball( working_image_cv, min_radius = min_radius, max_radius = max_radius, canny_higher_threshold = canny_higher_threshold, canny_accumulator_threshold = canny_accumulator_threshold, debug = False, hide_top_pixels = hide_top_pixels )
                
                detect_result.is_ball_detected = is_ball_found
                
                if True == is_ball_found:
                    rospy.loginfo( "detect_ball found: x: {}, y: {}, r: {}".format( center_coordinates_and_radius[0], center_coordinates_and_radius[1], center_coordinates_and_radius[2] ) )
                    
                    detect_result.detected_x = center_coordinates_and_radius[0]
                    detect_result.detected_y = center_coordinates_and_radius[1]
                    detect_result.detected_r = center_coordinates_and_radius[2]
                else:
                    rospy.loginfo( "detect_ball not found")
        
        self.detect_result_publisher.publish( detect_result )

        
if __name__ == '__main__':
    try:
        rospy.init_node("pluto_detector")
        detector = Detector()
        
        #rate = rospy.Rate( detector.DETECTOR_RATE() )
        
        while not rospy.is_shutdown():
            
            rospy.spin()
            #detector.detect_ball()
            
            #rate.sleep()
            
    except rospy.ROSInterruptException:
        pass