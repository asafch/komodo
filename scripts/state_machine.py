#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import String
import time
from pluto_common import *

class StateMachine:
    state = ""
    arm_init_message_sent = False
    camera_publisher = None
    camera_state_publisher = None
    robot_movement_publisher = None
    advance_message_sent = False
    front_camera_message_sent = False
    deploy_arm_message_sent = False
    search_ball_message_sent = False
    grab_message_sent = False

    def __init__(self):
        rospy.loginfo("State machine: initializing")
        self.state = "INIT_ARM"
        self.arm_movement_publisher = rospy.Publisher("/pluto/arm_movement/command", String, queue_size = 10)
        rospy.Subscriber("/pluto/arm_movement/result", String, self.arm_movement_result)
        self.camera_publisher = rospy.Publisher("/pluto/detector/current_camera", String, queue_size = 10)
        self.camera_state_publisher = rospy.Publisher("/pluto/detector/state_change", String, queue_size = 10)
        self.robot_movement_publisher = rospy.Publisher("/pluto/robot_movement/command", String, queue_size = 10)
        rospy.Subscriber("/pluto/robot_movement/result", String, self.robot_movement_done)
        time.sleep(20) # allow moveArm.py to load before sending the first arm initialization command
        rospy.loginfo("State machine: initialized")
        rospy.loginfo("State machine: state is INIT_ARM")

    def arm_movement_result(self, command):
        if command.data == "ARM_INITIALIZED":
            self.state = "SEARCH_BALL"
        elif command.data == "ARM_DEPLOYED":
            self.state = "GRAB"

    def robot_movement_done(self, command):
        if command.data == "BALL_FOUND" and self.state == "SEARCH_BALL":
            self.state = "ADVANCE"
        elif command.data == "NO_BALL":
            self.state = "END"
        elif command.data == "BALL_AT_BOTTOM_OF_FRAME":
            self.state = "CENTER_THE_BALL"
        elif command.data == "BALL_AT_POSITION":
            self.state = "DEPLOY_ARM"


    def main_loop(self):
        rospy.loginfo("State machine: main loop started")
        while True:
            if self.state == "INIT_ARM" and not self.arm_init_message_sent:
                self.arm_movement_publisher.publish("INIT_ARM")
                rospy.loginfo("State machine: arm initialization message sent")
                self.arm_init_message_sent = True
            elif self.state == "ADVANCE" and not self.advance_message_sent:
                self.advance_message_sent = True
                rospy.loginfo("State machine: state changed to ADVANCE")
                self.robot_movement_publisher.publish("FORWARD")
            elif self.state == "END":
                rospy.loginfo("State machine: state changed to END")
                rospy.signal_shutdown("State machine: no ball found, exiting...")
            elif self.state == "SEARCH_BALL" and not self.search_ball_message_sent:
                self.search_ball_message_sent = True
                self.robot_movement_publisher.publish("SEARCH_BALL")
                self.camera_state_publisher.publish("SEARCH")
                self.camera_publisher.publish("ASUS_CAMERA")
                rospy.loginfo("State machine: state changed to SEARCH_BALL")
            elif self.state == "CENTER_THE_BALL" and not self.front_camera_message_sent:
                self.front_camera_message_sent = True
                rospy.loginfo("State machine: state changed to CENTER_THE_BALL")
                self.camera_publisher.publish("FRONT_CAMERA")
            elif self.state == "DEPLOY_ARM" and not self.deploy_arm_message_sent:
                self.deploy_arm_message_sent = True
                rospy.loginfo("State machine: state changed to DEPLOY_ARM")
                self.arm_movement_publisher.publish("DEPLOY_ARM")
                self.camera_state_publisher.publish("NO_SEARCH")
            elif self.state == "GRAB" and not self.grab_message_sent:
                self.grab_message_sent = True
                rospy.loginfo("State machine: state changed to GRAB")
                self.camera_publisher.publish("CREATIVE_CAMERA")
                self.camera_state_publisher.publish("SEARCH")

if __name__ == '__main__':
    try:
        rospy.init_node("state_machine")
        machine = StateMachine()
        machine.main_loop()
        rospy.spin()        
    except rospy.ROSInterruptException:
        pass
