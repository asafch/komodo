#!/usr/bin/env python 
import sys
import copy
import rospy
import geometry_msgs.msg
import time
import numpy as np
from threading import Lock
from std_msgs.msg import String, Float64
from jupiter.msg import BallPosition
from jupiter.srv import Fingers
from common import *
from control_msgs.msg import JointControllerState
from dynamixel_msgs.msg import JointState as dxl_JointState

class ArmMovement:
    arm_movement_command = ""
    fingers_movement_command = ""
    arm_result_sent = False
    # target_elevator             = 0.0
    target_base                 = 0.0
    target_shoulder             = 0.0
    target_elbow1               = 0.0
    target_elbow2               = 0.0
    target_wrist                = 0.0
    target_lfinger              = 0.0
    target_rfinger              = 0.0
    is_accomplished_lfinger     = False
    is_accomplished_rfinger     = False
    # is_accomplished_elevator    = False
    is_accomplished_base        = False
    is_accomplished_shoulder    = False
    is_accomplished_elbow1      = False
    is_accomplished_elbow2      = False
    is_accomplished_wrist       = False
    error_value                 = 0.02
    arm_lock = Lock()
    fingers_lock = Lock()
    # elevator_subscriber = None
    base_subscriber = None
    shoulder_subscriber = None
    elbow1_subscriber = None
    elbow2_subscriber = None
    wrist_subscriber = None
    # lfinger_subscriber = None
    # rfinger_subscriber = None
    jointType = None
    first_group_moving = 0
    group1_executed = False
    group2_executed = False
    current_joint = ""
    ball_proper_x = False
    pixels_error = 10

    def __init__(self):
        rospy.loginfo("Arm movement: initializing")
        init_arguments(self)
        if self.is_simulation:
            self.jointType = JointControllerState
        else:
            self.jointType = dxl_JointState
        # arm publisher and suscriber
        rospy.Subscriber("/jupiter/arm_movement/command", String, self.process_command)
        rospy.Subscriber("/jupiter/ball_position", BallPosition, self.process_ball_position)
        self.arm_movement_result_publisher = rospy.Publisher("/jupiter/arm_movement/result", String, queue_size = 10)
        self.camera_state_publisher = rospy.Publisher("/jupiter/detector/state_change", String, queue_size = 10)
        # joint command publishers
        # self.elevator_publisher   = rospy.Publisher(adjust_namespace(self.is_simulation, "/elevator_controller/command"), Float64, queue_size = 10)
        self.base_publisher       = rospy.Publisher(adjust_namespace(self.is_simulation, "/base_rotation_controller/command"), Float64, queue_size = 10)
        self.shoulder_publisher   = rospy.Publisher(adjust_namespace(self.is_simulation, "/shoulder_controller/command"), Float64, queue_size = 10)
        self.elbow1_publisher     = rospy.Publisher(adjust_namespace(self.is_simulation, "/elbow1_controller/command"), Float64, queue_size = 10)
        self.elbow2_publisher     = rospy.Publisher(adjust_namespace(self.is_simulation, "/elbow2_controller/command"), Float64, queue_size = 10)
        self.wrist_publisher      = rospy.Publisher(adjust_namespace(self.is_simulation, "/wrist_controller/command"), Float64, queue_size = 10)
        rospy.wait_for_service("fingers")
        self.fingers_service = rospy.ServiceProxy("fingers", Fingers)
        rospy.loginfo("Arm movement: initialized")

    def process_command(self, command):
        rospy.loginfo("Arm movement: command received: %s", command.data)
        if command.data == "INIT_ARM":
            self.fingers_movement_command = "OPEN_FINGERS"
            self.init_arm()
            self.arm_result_sent = False
            self.arm_movement_command = command.data
            self.initialize_arm_subscribers()
            self.first_group_moving = 1
        elif command.data == "DEPLOY_ARM":
            self.deploy_arm()
            self.arm_result_sent = False
            self.arm_movement_command = command.data
            self.initialize_arm_subscribers()
            self.first_group_moving = 2
        elif command.data == "GRAB":
            self.fingers_movement_command = "CLOSE_FINGERS"
            self.close_fingers()
        elif command.data == "BRING_TO_BASKET":
            self.bring_to_basket()
            self.arm_result_sent = False
            self.arm_movement_command = command.data
            self.initialize_arm_subscribers()
            self.first_group_moving = 1
        elif command.data == "RELEASE_BALL":
            self.fingers_movement_command = "RELEASE_BALL"
            self.open_fingers()

    def process_ball_position(self, message):
        rospy.loginfo("Arm movement: ball pos msg\n%r", message)
        if message.detected:
            self.initialize_arm_subscribers()
            x_offset = message.x - message.img_width / 2
            y_offset = message.y - message.img_height / 2
            if abs(x_offset) > self.pixels_error and abs(self.target_shoulder - 1.4) > 0.1: # Arm isn't centered with respect to the ball. Don't bring to center if shoulder is at target value.
                self.move_arm("GROUP_1", 0.0, self.target_base + np.sign(x_offset) * 0.01, self.target_shoulder, self.target_elbow1, self.target_elbow2, self.target_wrist)
                rospy.loginfo("moved base")
                time.sleep(0.4)
            elif self.current_joint == "SHOULDER":
                self.current_joint = "ELBOW2"
                self.move_arm("GROUP_1", 0.0, self.target_base, min(1.4, self.target_shoulder + 0.03), self.target_elbow1, self.target_elbow2, self.target_wrist)
                rospy.loginfo("moved shoulder")
                time.sleep(0.4)
            else: # "ELBOW2"
                self.current_joint = "SHOULDER"
                if message.img_height / 3 <= message.y and message.y <= message.img_height * 0.66 and abs(self.target_shoulder - 1.4) > self.error_value / 2: 
                    self.move_arm("GROUP_1", 0.0, self.target_base, self.target_shoulder, self.target_elbow1, max(1.2, self.target_elbow2 - 0.03), self.target_wrist)
                    rospy.loginfo("moved elbow2")
                    time.sleep(0.4)
                elif abs(self.target_shoulder - 1.4) <= self.error_value / 2:
                    self.move_arm("GROUP_1", 0.0, self.target_base, self.target_shoulder, self.target_elbow1, max(1.2, self.target_elbow2 - 0.03), self.target_wrist)
                    rospy.loginfo("moved elbow2")
                    time.sleep(0.4)
                else:
                    self.camera_state_publisher.publish("SEARCH")
        elif abs(self.target_shoulder - 1.4) <= self.error_value / 2:
            self.move_arm("GROUP_1", 0.0, self.target_base, self.target_shoulder, self.target_elbow1, max(1.2, self.target_elbow2 - 0.03), self.target_wrist)
            rospy.loginfo("moved elbow2")
            time.sleep(0.4)
        else:
            self.camera_state_publisher.publish("SEARCH")

    def move_arm(self, first_group, elevator, base, shoulder, elbow1, elbow2, wrist):
        self.arm_lock.acquire()
        rospy.loginfo("lock acquired")
        self.group1_executed = False
        self.group2_executed = False
        # self.is_accomplished_elevator    = False
        self.is_accomplished_base        = False
        self.is_accomplished_shoulder    = False
        self.is_accomplished_elbow1      = False
        self.is_accomplished_elbow2      = False
        self.is_accomplished_wrist       = False
        # self.target_elevator = elevator
        self.target_base = base
        self.target_shoulder = shoulder
        self.target_elbow1 = elbow1
        self.target_elbow2 = elbow2
        self.target_wrist = wrist
        if first_group == "GROUP_1":
            rospy.loginfo("Arm movement: group 1 moves first")
            self.first_group_moving = 1
            self.move_group1()
        else:
            rospy.loginfo("Arm movement: group 2 moves first")
            self.first_group_moving = 2
            self.move_group2()
        self.arm_lock.release()

    # Dividing the joints into groups is necessary in order to avoid physical contact between the arm and elevator.
    # When initializing the arm or bringing it to the basket group 1 moves first; when deploying the arm group 2 moves first. 
    def move_group1(self):
        # self.elevator_publisher.publish(self.target_elevator)
        self.base_publisher.publish(self.target_base)
        self.shoulder_publisher.publish(self.target_shoulder)

    # Dividing the joints into groups is necessary in order to avoid physical contact between the arm and elevator.
    # When initializing the arm or bringing it to the basket group 1 moves first; when deploying the arm group 2 moves first. 
    def move_group2(self):
        self.elbow1_publisher.publish(self.target_elbow1)
        self.elbow2_publisher.publish(self.target_elbow2)
        self.wrist_publisher.publish(self.target_wrist)

    def open_fingers(self):
        rospy.loginfo("Arm movement: opening fingers...")
        try:
            self.fingers_service("OPEN_FINGERS", -0.5)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        if self.fingers_movement_command == "RELEASE_BALL":
            rospy.loginfo("Arm movement: ball released")
            self.arm_movement_result_publisher.publish("BALL_RELEASED")
        elif self.fingers_movement_command == "OPEN_FINGERS":
            rospy.loginfo("Arm movement: fingers are open")

    def close_fingers(self):
        rospy.loginfo("Arm movement: closing fingers...")
        try:
            self.fingers_service("CLOSE_FINGERS", 0.12)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.arm_movement_result_publisher.publish("BALL_GRABBED")
        rospy.loginfo("Arm movement: fingers are closed")

    def init_arm(self):
        rospy.loginfo("Arm movement: initializing arm...")
        self.move_arm("GROUP_1", 0.0, -1.0, 0.0, 1.5, -2.44, 1.6)
        self.open_fingers()

    def deploy_arm(self):
        rospy.loginfo("Arm movement: deploying arm...")
        self.move_arm("GROUP_2", 0.0, 0.0, 0.0, 0.0, 2.0, 0.0)

    def bring_to_basket(self):
        rospy.loginfo("Arm movement: bringing arm to basket...")
        self.move_arm("GROUP_1", 0.0, -1.0, 0.0, 1.5, -2.44, 1.6)

    # def elevator_result(self, result):
    #     self.arm_lock.acquire()
    #     self.is_accomplished_elevator = abs(self.target_elevator - result.process_value) <= self.error_value
    #     self.check_arm_command_executed()
    #     self.arm_lock.release()

    def base_result(self, result):
        self.arm_lock.acquire()
        value = None
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_base = abs(self.target_base - value) <= self.error_value
        self.check_group1_executed()
        self.arm_lock.release()

    def shoulder_result(self, result):
        self.arm_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_shoulder = abs(self.target_shoulder - value) <= self.error_value * 4
        self.check_group1_executed()
        self.arm_lock.release()

    def elbow1_result(self, result):
        self.arm_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_elbow1 = abs(self.target_elbow1 - value) <= self.error_value * 2
        self.check_group2_executed()
        self.arm_lock.release()

    def elbow2_result(self, result):
        self.arm_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_elbow2 = abs(self.target_elbow2 - value) <= self.error_value * 2
        self.check_group2_executed()
        self.arm_lock.release()

    def wrist_result(self, result):
        self.arm_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_wrist = abs(self.target_wrist - value) <= self.error_value
        self.check_group2_executed()
        self.arm_lock.release()

    def initialize_arm_subscribers(self):
        #joint state suscribers
        # self.elevator_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/elevator_controller/state"), self.jointType, self.elevator_result)
        self.base_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/base_rotation_controller/state"), self.jointType, self.base_result)
        self.shoulder_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/shoulder_controller/state"), self.jointType, self.shoulder_result)
        self.elbow1_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/elbow1_controller/state"), self.jointType, self.elbow1_result)
        self.elbow2_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/elbow2_controller/state"), self.jointType, self.elbow2_result)
        self.wrist_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/wrist_controller/state"), self.jointType, self.wrist_result)
    
    def shutdown_group1_subscribers(self):
        # self.elevator_subscriber.unregister()
        self.base_subscriber.unregister()
        self.shoulder_subscriber.unregister()

    def shutdown_group2_subscribers(self):
        self.elbow1_subscriber.unregister()
        self.elbow2_subscriber.unregister()
        self.wrist_subscriber.unregister()

    def check_group1_executed(self):
        if not self.group1_executed and self.is_accomplished_base and self.is_accomplished_shoulder:
            self.shutdown_group1_subscribers()
            self.group1_executed = True
            rospy.loginfo("Arm movement: group 1 at requested position")
            if self.first_group_moving == 1:
                self.move_group2()
            elif self.group2_executed:
                if self.arm_movement_command == "DEPLOY_ARM":
                    self.arm_movement_command = "ADJUST_ARM"
                    self.arm_movement_result_publisher.publish("ARM_DEPLOYED")
                    rospy.loginfo("Arm movement: arm deployed")
                    self.arm_result_sent = True
                    self.current_joint = "SHOULDER"
                elif self.arm_movement_command == "BRINGING_TO_BALL":
                    self.close_fingers()
                else:
                    rospy.logwarn("Arm movement: illegal arm state: %s", self.arm_movement_command)
            else:
                self.group1_executed = False
                self.initialize_arm_subscribers()


    def check_group2_executed(self):
        if not self.group2_executed and self.is_accomplished_elbow1 and self.is_accomplished_elbow2 and self.is_accomplished_wrist:
            self.shutdown_group2_subscribers()
            self.group2_executed = True
            rospy.loginfo("Arm movement: group 2 at requested position")
            if self.first_group_moving == 2:
                self.move_group1()
            elif self.group1_executed:
                if self.arm_movement_command == "INIT_ARM":
                    self.arm_movement_result_publisher.publish("ARM_INITIALIZED")
                    rospy.loginfo("Arm movement: initialization complete")
                    self.arm_result_sent = True
                elif self.arm_movement_command == "BRING_TO_BASKET":
                    self.arm_movement_result_publisher.publish("ARM_AT_BASKET")
                    rospy.loginfo("Arm movement: arm at basket")
                    self.arm_result_sent = True
                elif self.arm_movement_command == "BRINGING_TO_BALL":
                    self.close_fingers()
                else: # bring arm to ball
                    if self.target_shoulder == 1.4 and self.target_elbow2 == 1.2:
                        self.initialize_arm_subscribers()
                        self.arm_lock.release()
                        self.arm_movement_command = "BRINGING_TO_BALL"
                        self.move_arm("GROUP_2", 0.0, self.target_base, 1.55, 0, 0.85, 0.0) # Place fingers at the ball's sides. Group 2 moves first so the fingers won't collide with the stool
                        rospy.loginfo("Arm movement: final positioning before grabbing")
                        self.arm_lock.acquire()
                    else:
                        self.camera_state_publisher.publish("SEARCH")
            else:
                self.group2_executed = False
                self.initialize_arm_subscribers()


if __name__ == "__main__":
  try:
    rospy.init_node("arm_movement")
    arm_movement = ArmMovement()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
