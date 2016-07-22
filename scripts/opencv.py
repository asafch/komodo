#!/usr/bin/env python

import cv2
import numpy as np

class Detector:
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
        
# /home/alexds9/pluto_backups/pictures/top/with_ball    5
# /home/alexds9/pluto_backups/pictures/top/no_ball      7
# /home/alexds9/pluto_backups/pictures/arm/with_ball    9
# /home/alexds9/pluto_backups/pictures/arm/no_ball      1

data_with_ball = [ ["/home/alexds9/pluto_backups/pictures/top/with_ball", 5, True  ], 
                   ["/home/alexds9/pluto_backups/pictures/arm/with_ball", 9, False ] ]
                   
data_no_ball   = [ ["/home/alexds9/pluto_backups/pictures/top/no_ball", 7, True  ], 
                   ["/home/alexds9/pluto_backups/pictures/arm/no_ball", 1, False ] ]

def test_data_array( data, desired_result ):
    detector = Detector()
    
    for folder_and_amount in data:
        [ folder, amount, is_top ] = folder_and_amount
        for i in range(1,amount + 1):
            file_path = folder + "/image_" + str(i) + ".png"
            bgr_image = cv2.imread(file_path,1)
            
            if True == is_top:
                min_radius                  = 2
                max_radius                  = 25
                canny_higher_threshold      = 200
                canny_accumulator_threshold = 12
                hide_top_pixels             = 190
                
            else:
                min_radius                  = 3
                max_radius                  = 150
                canny_higher_threshold      = 200
                canny_accumulator_threshold = 12
                hide_top_pixels             = 0
                
                
            
            debug = False
                
            is_ball_found, center_coordinates_and_radius = detector.find_the_ball( bgr_image, min_radius = min_radius, max_radius = max_radius, canny_higher_threshold = canny_higher_threshold, canny_accumulator_threshold = canny_accumulator_threshold, debug = debug, hide_top_pixels = hide_top_pixels )
    
            if is_ball_found != desired_result:
                print "Not matching desired value: " + file_path
            else:
                # print "OK: " + file_path
                pass
            
def test_all_data():
    test_data_array( data_with_ball, True  )
    test_data_array( data_no_ball,   False )
    
test_all_data()
'''
detector = Detector()

min_radius                  = 2
max_radius                  = 25
canny_higher_threshold      = 200
canny_accumulator_threshold = 12
hide_top_pixels             = 190

bgr_image = cv2.imread('/home/alexds9/pluto_backups/pictures/top/no_ball/image_7.png',1)
is_ball_found, center_coordinates_and_radius = detector.find_the_ball( bgr_image, min_radius = min_radius, max_radius = max_radius, canny_higher_threshold = canny_higher_threshold, canny_accumulator_threshold = canny_accumulator_threshold, debug = True, hide_top_pixels = hide_top_pixels )
'''
#print is_ball_found
#print center_coordinates_and_radius
