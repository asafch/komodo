#!/usr/bin/env python 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from threading import Lock
from std_msgs.msg import String, Float64
from common import *
from control_msgs.msg import JointControllerState
from dynamixel_msgs.msg import JointState as dxl_JointState
from pluto.srv import Grip

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
    arm_lock = None
    fingers_lock = None
    # elevator_subscriber = None
    base_subscriber = None
    shoulder_subscriber = None
    elbow1_subscriber = None
    elbow2_subscriber = None
    wrist_subscriber = None
    lfinger_subscriber = None
    rfinger_subscriber = None
    jointType = None

    def __init__(self):
        rospy.loginfo("Arm movement: initializing")
        init_arguments(self) 
        self.arm_lock = Lock()
        self.fingers_lock = Lock()
        if self.is_simulation:
            self.jointType = JointControllerState
        else:
            self.jointType = dxl_JointState
        #arm publisher and suscriber
        rospy.Subscriber("/pluto/arm_movement/command", String, self.process_command)
        self.arm_movement_result_publisher = rospy.Publisher("/pluto/arm_movement/result", String, queue_size = 10)
        #joint command publishers
        # self.elevator_publisher   = rospy.Publisher(adjust_namespace(self.is_simulation, "/elevator_controller/command"), Float64, queue_size = 10)
        self.base_publisher       = rospy.Publisher(adjust_namespace(self.is_simulation, "/base_rotation_controller/command"), Float64, queue_size = 10)
        self.shoulder_publisher   = rospy.Publisher(adjust_namespace(self.is_simulation, "/shoulder_controller/command"), Float64, queue_size = 10)
        self.elbow1_publisher     = rospy.Publisher(adjust_namespace(self.is_simulation, "/elbow1_controller/command"), Float64, queue_size = 10)
        self.elbow2_publisher     = rospy.Publisher(adjust_namespace(self.is_simulation, "/elbow2_controller/command"), Float64, queue_size = 10)
        self.wrist_publisher      = rospy.Publisher(adjust_namespace(self.is_simulation, "/wrist_controller/command"), Float64, queue_size = 10)
        self.lfinger_publisher    = rospy.Publisher(adjust_namespace(self.is_simulation, "/left_finger_controller/command"), Float64, queue_size = 10)
        self.rfinger_publisher    = rospy.Publisher(adjust_namespace(self.is_simulation, "/right_finger_controller/command"), Float64, queue_size = 10)
        rospy.loginfo("Arm movement: initialized")

    def process_command(self, command):
        rospy.loginfo("Arm movement: command received: %s", command.data)
        if command.data == "INIT_ARM":
            self.init_arm()
            self.arm_result_sent = False
            self.arm_movement_command = command.data
            self.initialize_arm_subscribers()
        elif command.data == "DEPLOY_ARM":
            self.deploy_arm()
            self.arm_result_sent = False
            self.arm_movement_command = command.data
            self.initialize_arm_subscribers()
        elif command.data == "GRAB":
            self.close_fingers()
        elif command.data == "BRING_TO_BASKET":
            self.bring_to_basket()
            self.arm_result_sent = False
            self.arm_movement_command = command.data
            self.initialize_arm_subscribers()

    def move_arm(self, elevator, base, shoulder, elbow1, elbow2, wrist):
        self.arm_lock.acquire()
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
        # self.elevator_publisher.publish(elevator)
        self.base_publisher.publish(base)
        self.shoulder_publisher.publish(shoulder)
        self.elbow1_publisher.publish(elbow1)
        self.elbow2_publisher.publish(elbow2)
        self.wrist_publisher.publish(wrist)
        self.arm_lock.release()

    def move_fingers(self, position):
        self.fingers_lock.acquire()
        self.initialize_finger_subscribers()
        self.fingers_result_sent = False
        self.fingers_movement_command = position
        if position == "OPEN_FINGERS":
            rospy.loginfo("Arm movement: opening fingers")
            target_lfinger = -1.0
            target_rfinger = 1.0
        elif position == "CLOSE_FINGERS":
            rospy.loginfo("Arm movement: closing fingers")
            target_lfinger = 0.5
            target_rfinger = -0.5
        is_accomplished_lfinger = False
        is_accomplished_rfinger = False
        self.lfinger_publisher.publish(self.target_lfinger)
        self.rfinger_publisher.publish(self.target_rfinger)
        self.fingers_lock.release()

    def open_fingers(self):
        self.move_fingers("OPEN_FINGERS")

    def close_fingers(self):
        self.move_fingers("CLOSE_FINGERS")

    def init_arm(self):
        rospy.loginfo("Arm movement: initializing arm...")
        self.move_arm(0.0, 1.0, 1.5, 2.0, -2.0, 0.0)
        self.open_fingers()

    def deploy_arm(self):
        rospy.loginfo("Arm movement: deploying arm...")
        self.move_arm(0.0, 0.0, 1.55, 0.0, 1.47, 0.0)

    def bring_to_basket(self):
        rospy.loginfo("Arm movement: bringing to basket...")
        self.move_arm(0.0, 1.0, 1.5, 2.0, -2.0, 0.0)

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
        self.check_arm_command_executed()
        self.arm_lock.release()

    def shoulder_result(self, result):
        self.arm_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_shoulder = abs(self.target_shoulder - value) <= self.error_value * 4
        self.check_arm_command_executed()
        self.arm_lock.release()

    def elbow1_result(self, result):
        self.arm_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_elbow1 = abs(self.target_elbow1 - value) <= self.error_value
        self.check_arm_command_executed()
        self.arm_lock.release()

    def elbow2_result(self, result):
        self.arm_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_elbow2 = abs(self.target_elbow2 - value) <= self.error_value
        self.check_arm_command_executed()
        self.arm_lock.release()

    def wrist_result(self, result):
        self.arm_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_wrist = abs(self.target_wrist - value) <= self.error_value
        self.check_arm_command_executed()
        self.arm_lock.release()

    def lfinger_result(self, result):
        self.fingers_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_lfinger = abs(self.target_lfinger - value) <= self.error_value * 2
        self.check_fingers_command_executed()
        self.fingers_lock.release()

    def rfinger_result(self, result):
        self.fingers_lock.acquire()
        if self.is_simulation:
            value = result.process_value
        else:
            value = result.current_pos
        self.is_accomplished_rfinger = abs(self.target_rfinger - value) <= self.error_value * 2
        self.check_fingers_command_executed()
        self.fingers_lock.release()

    def initialize_arm_subscribers(self):
        #joint state suscribers
        # self.elevator_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/elevator_controller/state"), self.jointType, self.elevator_result)
        self.base_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/base_rotation_controller/state"), self.jointType, self.base_result)
        self.shoulder_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/shoulder_controller/state"), self.jointType, self.shoulder_result)
        self.elbow1_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/elbow1_controller/state"), self.jointType, self.elbow1_result)
        self.elbow2_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/elbow2_controller/state"), self.jointType, self.elbow2_result)
        self.wrist_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/wrist_controller/state"), self.jointType, self.wrist_result)

    def initialize_finger_subscribers(self):
        self.lfinger_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/left_finger_controller/state"), self.jointType, self.lfinger_result)
        self.rfinger_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/right_finger_controller/state"), self.jointType, self.rfinger_result)
    
    def shutdown_arm_subscribers(self):
        # self.elevator_subscriber.unregister()
        self.base_subscriber.unregister()
        self.shoulder_subscriber.unregister()
        self.elbow1_subscriber.unregister()
        self.elbow2_subscriber.unregister()
        self.wrist_subscriber.unregister()

    def shutdown_finger_subscribers(self):
        self.lfinger_subscriber.unregister()
        self.rfinger_subscriber.unregister()

    def check_fingers_command_executed(self):
        # rospy.loginfo("fingers %r", self.fingers_result_sent)
        if not self.fingers_result_sent:
            check = self.is_accomplished_lfinger and self.is_accomplished_rfinger
            if check:
                self.shutdown_finger_subscribers()
                # self.arm_movement_result_publisher.publish("FINGERS_???")
                rospy.loginfo("Arm movement: fingers are %s", self.fingers_movement_command)
                self.arm_movement_result_publisher.publish("BALL_GRABBED")
                self.fingers_result_sent = True

    def check_arm_command_executed(self):
        if not self.arm_result_sent:
            # check = (self.is_accomplished_elevator and \
            check = (self.is_accomplished_base and \
                self.is_accomplished_shoulder and \
                self.is_accomplished_elbow1 and \
                self.is_accomplished_elbow2 and \
                self.is_accomplished_wrist)
            if check:
                self.shutdown_arm_subscribers()
                if self.arm_movement_command == "INIT_ARM":
                    self.arm_movement_result_publisher.publish("ARM_INITIALIZED")
                    rospy.loginfo("Arm movement: initialization complete")
                elif self.arm_movement_command == "DEPLOY_ARM":
                    self.arm_movement_result_publisher.publish("ARM_DEPLOYED")
                    rospy.loginfo("Arm movement: arm deployed")
                elif self.arm_movement_command == "BRING_TO_BASKET":
                    self.arm_movement_result_publisher.publish("ARM_AT_BASKET")
                    rospy.loginfo("Arm movement: arm at basket")
                self.arm_result_sent = True


if __name__ == "__main__":
  try:
    rospy.init_node("arm_movement")
    arm_movement = ArmMovement()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
