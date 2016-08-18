#!/usr/bin/python
import roslib
import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from common import *
from tf.transformations import euler_from_quaternion
from threading import Lock
import thread

class RobotMovement:
	movement_subscriber = None
	movement_publisher = None
	movement_message = None
	movement_command = ""
	state_machine_publisher = None
	fine_movement = False
	odometry_subscriber = None
	last_orientation = None
	update_reference_odometry = False
	searching_for_ball = False
	prepared_to_stop = False
	angular_offset = 0.75
	odometry_lock = None

	def __init__(self):
		init_arguments(self)
		self.movement_publisher = rospy.Publisher(adjust_namespace(self.is_simulation, "/diff_driver/command"), Twist, queue_size = 100)
		self.movement_subscriber = rospy.Subscriber("/jupiter/robot_movement/command", String, self.process_command)
		self.state_machine_publisher = rospy.Publisher("/jupiter/robot_movement/result", String, queue_size = 10)
		self.odometry_subscriber = rospy.Subscriber(adjust_namespace(self.is_simulation, "/diff_driver/odometry"), Odometry, self.odometry_updated)
		self.movement_message = Twist()
		self.odometry_lock = Lock()

	# Odometry messages are  at a rate of ~50Hz, so publish a movement message for each odometry message received.
	# This is vital when dealing with the real robot, since it requires a constant stream of movement messages in order to not ignore them.
	def odometry_updated(self, message):
		self.odometry_lock.acquire()
		if self.update_reference_odometry:
			self.last_orientation = self.odometry_get_angle(message)
			self.update_reference_odometry = False
			time.sleep(6)
		elif self.searching_for_ball:
			if abs(self.last_orientation - self.odometry_get_angle(message)) >= 5 and not self.fine_movement:
				self.fine_movement = True # The wheels overcame the friction from the floor. Reduce the turn rate to improve CV functionality.
				rospy.loginfo("Robot movement: fine movement")
			if abs(self.last_orientation - self.odometry_get_angle(message)) >= self.angular_offset * 2 and not self.prepared_to_stop:
				self.prepared_to_stop = True
			elif self.prepared_to_stop and abs(self.last_orientation - self.odometry_get_angle(message)) <= self.angular_offset:
				self.stop_in_place("NO_BALL")
		self.odometry_lock.release()
		self.message_republisher()


	def odometry_get_angle(self, message):
		q = message.pose.pose.orientation
		roll, pitch, yaw = euler_from_quaternion([q.w, q.x, q.y, q.z])
		return (roll * 180 / math.pi) + 180

	def message_republisher(self):
		if self.movement_command != "":
			if self.movement_command == "RIGHT":
				self.move_robot("RIGHT", 0, 0, 0, 0, 0, -self.get_anguler_speed())
			elif self.movement_command == "LEFT":
				self.move_robot("LEFT", 0, 0, 0, 0, 0, self.get_anguler_speed())
			elif self.movement_command == "FORWARD":
				self.move_robot("FORWARD", self.get_linear_speed(), 0, 0, 0, 0, 0)
			elif self.movement_command == "BACKWARD":
				self.move_robot("BACKWARD", -self.get_linear_speed(), 0, 0, 0, 0, 0)
			self.movement_publisher.publish(self.movement_message)

	def stop_in_place(self, cause):
		self.searching_for_ball = False
		self.move_robot("STOP", 0, 0, 0, 0, 0, 0)
		self.state_machine_publisher.publish(cause)
		rospy.loginfo("Robot movement: stop in place, cause: %s", cause)

	def get_anguler_speed(self):
		if self.fine_movement:
			return 0.08 # rad/s
		else:
			return 0.2 # rad/s

	def get_linear_speed(self):
		if self.fine_movement:
			return 0.03 # m/s
		else:
			return 0.05 # m/s

	def move_robot(self, direction, lx, ly, lz, ax, ay, az):
		self.movement_message.linear.x = lx
		self.movement_message.linear.y = ly
		self.movement_message.linear.z = lz
		self.movement_message.angular.x = ax
		self.movement_message.angular.y = ay
		self.movement_message.angular.z = az
		self.movement_command = direction

	def process_command(self, command):
		self.odometry_lock.acquire()
		if command.data == "SEARCH_BALL":
			self.update_reference_odometry = True
			self.searching_for_ball = True
			self.prepared_to_stop = False
			self.fine_movement = False
			self.move_robot("RIGHT", 0, 0, 0, 0, 0, -self.get_anguler_speed())
		elif command.data == "STOP-BALL_FOUND":
			self.stop_in_place("BALL_FOUND")
		elif command.data == "STOP-BALL_AT_BOTTOM_OF_FRAME":
			self.stop_in_place("BALL_AT_BOTTOM_OF_FRAME")
			self.fine_movement = True
		elif command.data == "STOP-READY_TO_GRAB":
			self.stop_in_place("READY_TO_GRAB")
			self.searching_for_ball = False
		elif command.data == "LEFT":
			self.move_robot(command.data, 0, 0, 0, 0, 0, self.get_anguler_speed())
		elif command.data == "RIGHT":
			self.move_robot(command.data, 0, 0, 0, 0, 0, -self.get_anguler_speed())
		elif command.data == "FORWARD":
			self.move_robot(command.data, self.get_linear_speed(), 0, 0, 0, 0, 0)
		elif command.data == "FORWARD-LEFT":
			self.move_robot(command.data, self.get_linear_speed(), 0, 0, 0, 0, self.get_anguler_speed() * 3.0)
		elif command.data == "FORWARD-RIGHT":
			self.move_robot(command.data, self.get_linear_speed(), 0, 0, 0, 0, -self.get_anguler_speed() * 3.0)
		else: # BACKWARD
			self.move_robot(command.data, -self.get_linear_speed(), 0, 0, 0, 0, 0)
		rospy.loginfo("Robot movement: move %s", self.movement_command)
		self.odometry_lock.release()

if __name__ == "__main__":
	rospy.init_node("robot_movement")
	robot_movement = RobotMovement()
	rospy.spin()
