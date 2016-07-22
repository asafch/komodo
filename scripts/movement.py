#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
import time
from pluto_common import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class Movement:
    counter             = 1
    command_to_robot    = 0
    move_command        = 0
    base_value_x        = 0
    base_value_y        = 0
    base_value_angle    = 0
    odometry_last       = 0
    moving              = False
    move_robot_command  = 0
    is_fine             = False
    

    def SAMPLE_FREQUENCY( self ):
        return 20
        
    def MOVE_LEFT( self ):
        return "LEFT"
        
    def MOVE_RIGHT( self ):
        return "RIGHT"
        
    def MOVE_FORWARD( self ):
        return "FORWARD"
        
    def MOVE_BACKWARD( self ):
        return "BACKWARD"
        
    def MOVE_STOP( self ):
        return "STOP"
        
    def MOVE_MODE_FINE( self ):
        return "FINE"
        
    def speed_linear_get( self ):
        if True == self.is_fine:
            if True == self.is_simulation:
                return 0.2
            else:
                return 0.1
        else:
            if True == self.is_simulation:
                return 0.5
            else:
                return 0.2
        
    def speed_angular_get( self ):
        
        if True == self.is_fine:
            if True == self.is_simulation:
                return 0.2
            else:
                return 0.75
        else:
            if True == self.is_simulation:
                return 0.5
            else:
                return 0.85
        
    def epsilon_angular_get( self ):
        if True == self.is_fine:
            return 2.0
        else:
            return 5.0
        
    def epsilon_linear_get( self ):
        if True == self.is_fine:
            
            if True == self.is_simulation:
                desired_distance_move_cm = 1.0
            else:
                desired_distance_move_cm = 1.0
                
        else:
            
            if True == self.is_simulation:
                desired_distance_move_cm = 5.0
            else:
                desired_distance_move_cm = 10.0
            
        distance_to_move_meter = desired_distance_move_cm / 100.0
        return distance_to_move_meter
        
    def calculate_linear_offset( self, p1, p2 ):
        [x1, y1] = p1
        [x2, y2] = p2
        
        return ((((x2 - x1) ** 2) + ((y2 - y1) ** 2)) ** 0.5)
        
    def calculate_angular_offset( self, a1, a2 ):
        if a1 > a2:
            b = a1 - a2
        else:
            b = a2 - a1
        
        if b > 180.0:
            b = 360.0 - b
            
        return b
        
        
    def odometry_get_angle( self, odometry ):
        
        q = odometry.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion( [q.w, q.x, q.y, q.z] )
        
        return (roll*180/math.pi) + 180

    def odometry_get_x( self, odometry ):
        return odometry.pose.pose.position.x
        
    def odometry_get_y( self, odometry ):
        return odometry.pose.pose.position.y
        
    def __init__( self ):
        rospy.loginfo( "Movement innitialized " )
        
        init_arguments( self )
        
        self.move_robot_command = Twist()
        
        self.move_command = self.MOVE_STOP()
        
        self.command_to_robot = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/diff_driver/command' ), Twist, queue_size=10)
        
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/diff_driver/odometry' ) , Odometry, self.odometry_updated )
        
        rospy.Subscriber("/pluto/movement/command", String, self.move )
        self.move_result_publisher = rospy.Publisher('/pluto/movement/done', String, queue_size=10 )

    def odometry_updated( self, odometry ):
        self.odometry_last = odometry
        
        if True == self.moving:
            self.command_to_robot.publish( self.move_robot_command )
    
            if ( self.MOVE_LEFT() == self.move_command ) or ( self.MOVE_RIGHT() == self.move_command ):

                last_value_angle     = self.odometry_get_angle( self.odometry_last ) 
                if self.calculate_angular_offset( last_value_angle, self.base_value_angle ) >= self.epsilon_angular_get():
                    rospy.loginfo("Movement: 1, offset {}, epsilon {}".format(self.calculate_angular_offset( last_value_angle, self.base_value_angle ), self.epsilon_angular_get()))
                    self.move( String( self.MOVE_STOP() ) )
                
            elif ( self.MOVE_FORWARD() == self.move_command ) or ( self.MOVE_BACKWARD() == self.move_command ):
                
                last_value_x         = self.odometry_get_x( self.odometry_last )
                last_value_y         = self.odometry_get_y( self.odometry_last )
                if self.calculate_linear_offset( [last_value_x, last_value_y], [self.base_value_x, self.base_value_y] ) >= self.epsilon_linear_get():
                    rospy.loginfo("Movement: 2, offset {}, epsilon {}".format(self.calculate_linear_offset( [last_value_x, last_value_y], [self.base_value_x, self.base_value_y] ), self.epsilon_linear_get()))
                    self.move( String( self.MOVE_STOP() ) )
        

    def move_linear( self, velocity ):
        
        self.move_robot_command.linear.x     = velocity
        self.move_robot_command.linear.y     = 0
        self.move_robot_command.linear.z     = 0
        self.move_robot_command.angular.x    = 0
        self.move_robot_command.angular.y    = 0
        self.move_robot_command.angular.z    = 0
        
        self.command_to_robot.publish( self.move_robot_command )
        
    def move_turn( self, velocity ):
        
        self.move_robot_command.linear.x     = 0
        self.move_robot_command.linear.y     = 0
        self.move_robot_command.linear.z     = 0
        self.move_robot_command.angular.x    = 0
        self.move_robot_command.angular.y    = 0
        self.move_robot_command.angular.z    = velocity
        
        self.command_to_robot.publish( self.move_robot_command )
        
    def move_stop( self ):
        move_robot_command = Twist()

        self.command_to_robot.publish(move_robot_command)

    #def move_receive_command( self, command ):
    #    self.move_command = command
            
            
    def move( self, command ):
        self.move_command = command.data
        
        self.base_value_x         = self.odometry_get_x( self.odometry_last )
        self.base_value_y         = self.odometry_get_y( self.odometry_last )
        self.base_value_angle     = self.odometry_get_angle( self.odometry_last ) 
        
        rospy.loginfo( "Movement: x: {}, y: {}, a: {}".format( self.base_value_x, self.base_value_y, self.base_value_angle ) )
        
        if      self.MOVE_LEFT()      == self.move_command:
            rospy.loginfo("Movement: LEFT")
            self.moving = True
            self.move_turn( self.speed_angular_get() )
            
        elif    self.MOVE_RIGHT()     == self.move_command:
        
            rospy.loginfo("Movement: RIGHT")
            self.moving = True
            self.move_turn( -self.speed_angular_get() )
            
        elif    self.MOVE_FORWARD()   == self.move_command:
        
            rospy.loginfo("Movement: FORWARD")
            self.moving = True
            self.move_linear( self.speed_linear_get() )
            
        elif    self.MOVE_BACKWARD()  == self.move_command:
        
            rospy.loginfo("Movement: BACKWARD")
            self.moving = True
            self.move_linear( -self.speed_linear_get() )
            
        elif    self.MOVE_STOP()      == self.move_command:
        
            rospy.loginfo("Movement: STOP")
            self.moving = False
            self.move_stop()
            self.move_result_publisher.publish( "move_done" )
            
        elif    self.MOVE_MODE_FINE() == self.move_command:
        
            rospy.loginfo("Movement: FINE")
            self.is_fine = True
            
        else:
            rospy.loginfo("Movement: unknown command! \"{0}\"".format(self.move_command))
            self.move_stop()
        
        
        
        

if __name__ == '__main__':
    try:
        # lobal movement
        rospy.init_node("pluto_movement")
        movement = Movement()

        #rate = rospy.Rate( movement.SAMPLE_FREQUENCY() )
        while not rospy.is_shutdown():

            rospy.spin()

            #rate.sleep()
        
    except rospy.ROSInterruptException:
        pass

