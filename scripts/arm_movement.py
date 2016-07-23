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
from pluto_common import *
from control_msgs.msg import JointControllerState
from dynamixel_msgs.msg import JointState as dxl_JointState
from pluto.msg import DetectResult
from pluto.srv import Grip

class ArmMovement:
    move_command = ""
    response_sent = False
    target_elevator             = 0.0
    target_base                 = 0.0
    target_shoulder             = 0.0
    target_elbow1               = 0.0
    target_elbow2               = 0.0
    target_wrist                = 0.0
    target_lfinger              = 0.0
    target_rfinger              = 0.0
    is_accomplished_lfinger     = False
    is_accomplished_rfinger     = False
    is_accomplished_elevator    = False
    is_accomplished_base        = False
    is_accomplished_shoulder    = False
    is_accomplished_elbow1      = False
    is_accomplished_elbow2      = False
    is_accomplished_wrist       = False
    error_value                 = 0.02
    lock = None
    elevator_subscriber = None
    base_subscriber = None
    shoulder_subscriber = None
    elbow1_subscriber = None
    elbow2_subscriber = None
    wrist_subscriber = None
    lfinger_subscriber = None
    rfinger_subscriber = None

    def __init__(self):
        rospy.loginfo("Arm movement: initializing")
        init_arguments(self) 
        self.lock = Lock()
        #arm publisher and suscriber
        rospy.Subscriber("/pluto/arm_movement/command", String, self.process_command)
        self.movement_result_publisher = rospy.Publisher("/pluto/arm_movement/result", String, queue_size = 10)
        #joint command publishers
        self.elevator_publisher   = rospy.Publisher(pluto_add_namespace(self.is_simulation, "/elevator_controller/command"), Float64, queue_size = 10)
        self.base_publisher       = rospy.Publisher(pluto_add_namespace(self.is_simulation, "/base_rotation_controller/command"), Float64, queue_size = 10)
        self.shoulder_publisher   = rospy.Publisher(pluto_add_namespace(self.is_simulation, "/shoulder_controller/command"), Float64, queue_size = 10)
        self.elbow1_publisher     = rospy.Publisher(pluto_add_namespace(self.is_simulation, "/elbow1_controller/command"), Float64, queue_size = 10)
        self.elbow2_publisher     = rospy.Publisher(pluto_add_namespace(self.is_simulation, "/elbow2_controller/command"), Float64, queue_size = 10)
        self.wrist_publisher      = rospy.Publisher(pluto_add_namespace(self.is_simulation, "/wrist_controller/command"), Float64, queue_size = 10)
        self.lfinger_publisher    = rospy.Publisher(pluto_add_namespace(self.is_simulation, "/left_finger_controller/command"), Float64, queue_size = 10)
        self.rfinger_publisher    = rospy.Publisher(pluto_add_namespace(self.is_simulation, "/right_finger_controller/command"), Float64, queue_size = 10)
        rospy.loginfo("Arm movement: initialized")

    def process_command(self, command):
        self.move_command = command.data
        self.response_sent = False
        rospy.loginfo("Arm movement: command received: %s", command.data)
        self.initialize_subscribers()
        if self.move_command == "INIT_ARM":
            self.init_arm()
        elif self.move_command == "DEPLOY_ARM":
            self.deploy_arm()

    def move_arm(self, elevator, base, shoulder, elbow1, elbow2, wrist):
        self.lock.acquire()
        is_accomplished_elevator         = False
        self.is_accomplished_base        = False
        self.is_accomplished_shoulder    = False
        self.is_accomplished_elbow1      = False
        self.is_accomplished_elbow2      = False
        self.is_accomplished_wrist       = False
        self.target_elevator = elevator
        self.target_base = base
        self.target_shoulder = shoulder
        self.target_elbow1 = elbow1
        self.target_elbow2 = elbow2
        self.target_wrist = wrist
        self.elevator_publisher.publish(elevator)
        self.base_publisher.publish(base)
        self.shoulder_publisher.publish(shoulder)
        self.elbow1_publisher.publish(elbow1)
        self.elbow2_publisher.publish(elbow2)
        self.wrist_publisher.publish(wrist)
        self.lock.release()

    def open_fingers(self):
        self.lock.acquire()
        rospy.loginfo("Arm movement: opening fingers")
        is_accomplished_lfinger = False
        is_accomplished_rfinger = False
        target_lfinger = -0.5
        target_rfinger = 0.5
        self.lfinger_publisher.publish(-0.5)
        self.rfinger_publisher.publish(0.5)
        self.lock.release()

    def close_fingers(self):
        self.lock.acquire()
        rospy.loginfo("Arm movement: closing fingers")
        is_accomplished_lfinger = False
        is_accomplished_rfinger = False
        target_lfinger = 0.2
        target_rfinger = -0.2
        self.rfinger_publisher.publish(0.2)
        self.rfinger_publisher.publish(-0.2)
        self.lock.release()

    def init_arm(self):
        self.move_arm(0.0, 0.0, 1.5, 2.0, -2.0, 0.0)
        self.open_fingers()

    def deploy_arm(self):
        self.move_arm(0.0, 0.0, 1.55, 0.0, 2.3, 0.0)

    def elevator_result(self, result):
        self.lock.acquire()
        self.is_accomplished_elevator = abs(self.target_elevator - result.process_value) <= self.error_value
        self.lock.release()
        self.check_command_executed()
        # if not self.is_accomplished_elevator:
        #     rospy.loginfo("elevator")
        #     rospy.loginfo(abs(self.target_elevator - result.process_value))

    def base_result(self, result):
        self.lock.acquire()
        self.is_accomplished_base = abs(self.target_base - result.process_value) <= self.error_value
        self.check_command_executed()
        self.lock.release()
        # if not self.is_accomplished_base:
        #     rospy.loginfo("base")
        #     rospy.loginfo(abs(self.target_base - result.process_value))

    def shoulder_result(self, result):
        self.lock.acquire()
        self.is_accomplished_shoulder = abs(self.target_shoulder - result.process_value) <= (self.error_value + 0.02)
        self.check_command_executed()
        self.lock.release()
        # if not self.is_accomplished_shoulder:
        #     rospy.loginfo("shoulder")
        #     rospy.loginfo(abs(self.target_shoulder - result.process_value))

    def elbow1_result(self, result):
        self.lock.acquire()
        self.is_accomplished_elbow1 = abs(self.target_elbow1 - result.process_value) <= self.error_value
        self.check_command_executed()
        self.lock.release()
        # if not self.is_accomplished_elbow1:
        #     rospy.loginfo("elbow1")
        #     rospy.loginfo(abs(self.target_elbow1 - result.process_value))

    def elbow2_result(self, result):
        self.lock.acquire()
        self.is_accomplished_elbow2 = abs(self.target_elbow2 - result.process_value) <= self.error_value
        self.check_command_executed()
        self.lock.release()
        # if not self.is_accomplished_elbow2:
        #     rospy.loginfo("elbow2")
        #     rospy.loginfo(abs(self.target_elbow2 - result.process_value))

    def wrist_result(self, result):
        self.lock.acquire()
        self.is_accomplished_wrist = abs(self.target_wrist - result.process_value) <= self.error_value
        self.check_command_executed()
        self.lock.release()
        # if not self.is_accomplished_wrist:
        #     rospy.loginfo("wrist")
        #     rospy.loginfo(abs(self.target_wrist - result.process_value))

    def lfinger_result(self, result):
        self.lock.acquire()
        self.is_accomplished_wrist = abs(self.target_lfinger - result.process_value) <= self.error_value
        self.check_fingers()
        self.lock.release()

    def rfinger_result(self, result):
        self.lock.acquire()
        self.is_accomplished_rfinger = abs(self.target_rfinger - result.process_value) <= self.error_value
        self.check_fingers()
        self.lock.release()

    def initialize_subscribers(self):
        if self.is_simulation:
            JointType = JointControllerState
        else:
            JointType = dxl_JointState
        #joint state suscribers
        self.elevator_subscriber = rospy.Subscriber(pluto_add_namespace(self.is_simulation, "/elevator_controller/state"), JointType, self.elevator_result)
        self.base_subscriber = rospy.Subscriber(pluto_add_namespace(self.is_simulation, "/base_rotation_controller/state"), JointType, self.base_result)
        self.shoulder_subscriber = rospy.Subscriber(pluto_add_namespace(self.is_simulation, "/shoulder_controller/state"), JointType, self.shoulder_result)
        self.elbow1_subscriber = rospy.Subscriber(pluto_add_namespace(self.is_simulation, "/elbow1_controller/state"), JointType, self.elbow1_result)
        self.elbow2_subscriber = rospy.Subscriber(pluto_add_namespace(self.is_simulation, "/elbow2_controller/state"), JointType, self.elbow2_result)
        self.wrist_subscriber = rospy.Subscriber(pluto_add_namespace(self.is_simulation, "/wrist_controller/state"), JointType, self.wrist_result)
        self.lfinger_subscriber = rospy.Subscriber(pluto_add_namespace(self.is_simulation, "/left_finger_controller/state"), JointType, self.lfinger_result)
        self.rfinger_subscriber = rospy.Subscriber(pluto_add_namespace(self.is_simulation, "/right_finger_controller/state"), JointType, self.rfinger_result)
    
    def shutdown_subscribers(self):
        self.elevator_subscriber.unregister()
        self.base_subscriber.unregister()
        self.shoulder_subscriber.unregister()
        self.elbow1_subscriber.unregister()
        self.elbow2_subscriber.unregister()
        self.wrist_subscriber.unregister()

    def check_fingers(self):
        if self.is_accomplished_lfinger and self.is_accomplished_rfinger:
            rospy.loginfo("Arm movement: fingers at position")
            self.lfinger_subscriber.unregister()
            self.rfinger_subscriber.unregister()

    def check_command_executed(self):
        if not self.response_sent:
            check = (self.is_accomplished_elevator and \
                self.is_accomplished_base and \
                self.is_accomplished_shoulder and \
                self.is_accomplished_elbow1 and \
                self.is_accomplished_elbow2 and \
                self.is_accomplished_wrist)
            if check:
                self.shutdown_subscribers()
                self.movement_result_publisher.publish("ARM_INIT_DONE")
                rospy.loginfo("Arm movement: initialization complete")
                self.response_sent = True


if __name__ == "__main__":
  try:
    rospy.init_node("arm_movement")
    arm_movement = ArmMovement()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
