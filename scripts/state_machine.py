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


    def __init__(self):
        rospy.loginfo("State machine: initializing")
        self.state = "INIT_ARM"
        self.arm_movement_publisher = rospy.Publisher("/pluto/arm_movement/command", String, queue_size = 10)
        rospy.Subscriber("/pluto/arm_movement/result", String, self.arm_movement_done)
        self.camera_publisher = rospy.Publisher("/pluto/current_camera", String, queue_size = 10)
        time.sleep(10) # allow moveArm.py to load before sending the first arm initialization command
        rospy.loginfo("State machine: initialized")
        rospy.loginfo("State machine: state is INIT_ARM")

    def arm_movement_done(self, command):
        if command.data == "ARM_INIT_DONE":
            self.state = "SEARCH_BALL"
            rospy.loginfo("State machine: state changed to SEARCH_BALL")

    def main_loop(self):
        rospy.loginfo("State machine: main loop started")
        while True:
            if self.state == "INIT_ARM" and not self.arm_init_message_sent:
                self.arm_movement_publisher.publish("INIT_ARM")
                rospy.loginfo("State machine: arm initialization message sent")
                self.arm_init_message_sent = True
                self.camera_publisher.publish("ASUS_CAMERA")



if __name__ == '__main__':
    try:
        rospy.init_node("state_machine")
        machine = StateMachine()
        machine.main_loop()
        rospy.spin()        
    except rospy.ROSInterruptException:
        pass
