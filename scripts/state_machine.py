#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String, Bool
from pluto.msg import DetectResult
import time
from pluto_common import *

class StateMachine:
    state = ""
    arm_init_message_sent = False
    camera_publisher = None
    camera_state_publisher = None
    robot_movement_publisher = None


    def __init__(self):
        rospy.loginfo("State machine: initializing")
        self.state = "INIT_ARM"
        self.arm_movement_publisher = rospy.Publisher("/pluto/arm_movement/command", String, queue_size = 10)
        rospy.Subscriber("/pluto/arm_movement/result", String, self.arm_initialized)
        self.camera_publisher = rospy.Publisher("/pluto/detector/current_camera", String, queue_size = 10)
        self.camera_state_publisher = rospy.Publisher("/pluto/detector/state_change", String, queue_size = 10)
        self.robot_movement_publisher = rospy.Publisher("/pluto/robot_movement/command", String, queue_size = 10)
        rospy.Subscriber("/pluto/robot_movement/result", String, self.robot_movement_done)
        time.sleep(20) # allow moveArm.py to load before sending the first arm initialization command
        rospy.loginfo("State machine: initialized")
        rospy.loginfo("State machine: state is INIT_ARM")

    def arm_initialized(self, command):
        if command.data == "ARM_INIT_DONE":
            self.state = "SEARCH_BALL"
            self.robot_movement_publisher.publish("SEARCH_BALL")
            self.camera_state_publisher.publish("SEARCH")
            self.camera_publisher.publish("ASUS_CAMERA")
            rospy.loginfo("State machine: state changed to SEARCH_BALL")

    def robot_movement_done(self, command):
        if command.data == "BALL_FOUND" and self.state == "SEARCH_BALL":
            self.state = "ADVANCE"
            rospy.loginfo("State machine: state changed to ADVANCE")
            self.robot_movement_publisher.publish("FORWARD")
        elif command.data == "NO_BALL":
            self.state = "END"
            rospy.signal_shutdown("State machine: no ball found, exiting...")

    def main_loop(self):
        rospy.loginfo("State machine: main loop started")
        while True:
            if self.state == "INIT_ARM" and not self.arm_init_message_sent:
                self.arm_movement_publisher.publish("INIT_ARM")
                rospy.loginfo("State machine: arm initialization message sent")
                self.arm_init_message_sent = True

if __name__ == '__main__':
    try:
        rospy.init_node("state_machine")
        machine = StateMachine()
        machine.main_loop()
        rospy.spin()        
    except rospy.ROSInterruptException:
        pass
