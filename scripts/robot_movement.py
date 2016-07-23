#!/usr/bin/python
import roslib
import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from pluto_common import *
from tf.transformations import euler_from_quaternion

class RobotMovement:

	movement_subscriber = None
	movement_publisher = None
	movement_message = None
	state_machine_publisher = None
	fine_movement = False
	odometry_subscriber = None
	last_orientation = None
	update_reference_odometry = False
	searching_for_ball = False
	prepared_to_stop = False
	angular_offset = 0.75

	def __init__(self):
		init_arguments(self)
		self.movement_subscriber = rospy.Subscriber("pluto/robot_movement/command", String, self.process_command)
		self.movement_publisher = rospy.Publisher("/diff_driver/command", Twist, queue_size = 10)
		self.state_machine_publisher = rospy.Publisher("/pluto/robot_movement/result", String, queue_size = 10)
		self.odometry_subscriber = rospy.Subscriber(pluto_add_namespace(self.is_simulation, "/diff_driver/odometry"), Odometry, self.odometry_updated)
		self.movement_message = Twist()

	def odometry_updated(self, message):
		if self.update_reference_odometry:
			self.last_orientation = self.odometry_get_angle(message)
			self.update_reference_odometry = False
			time.sleep(5)
		elif self.searching_for_ball:
			if abs(self.last_orientation - self.odometry_get_angle(message)) >= self.angular_offset * 2:
				self.prepared_to_stop = True
			elif self.prepared_to_stop and abs(self.last_orientation - self.odometry_get_angle(message)) <= self.angular_offset:
				self.stop_turning("NO_BALL")

	def odometry_get_angle(self, message):
		q = message.pose.pose.orientation
		roll, pitch, yaw = euler_from_quaternion([q.w, q.x, q.y, q.z])
		return (roll * 180 / math.pi) + 180

	def stop_turning(self, cause):
		self.searching_for_ball = False
		self.movement_message = Twist()
		self.movement_publisher.publish(self.movement_message)
		self.state_machine_publisher.publish(cause)
		if cause == "BALL_FOUND":
			rospy.loginfo("Robot movement: ball is centered, wheels stopped")
		else:
			rospy.loginfo("Robot movement: no ball detected")

	def get_anguler_speed(self):
		if self.fine_movement:
			if self.is_simulation:
				return 0.2
			else:
				return 0.75
		else:
			if self.is_simulation:
				return 0.5
			else:
				return 0.85

	def process_command(self, command):
		if command.data == "SEARCH_BALL":
			self.movement_message.linear.x = 0
			self.movement_message.linear.y = 0
			self.movement_message.linear.z = 0
			self.movement_message.angular.x = 0
			self.movement_message.angular.y = 0
			self.movement_message.angular.z = -self.get_anguler_speed()
			rospy.loginfo("Robot movement: move RIGHT")
			self.update_reference_odometry = True
			self.searching_for_ball = True
			self.movement_publisher.publish(self.movement_message)
		elif command.data == "STOP-BALL_FOUND":
			self.stop_turning("BALL_FOUND")
		elif command.data == "LEFT":
			return
		elif command.data == "RIGHT":
			self.movement_message.linear.x = 0
			self.movement_message.linear.y = 0
			self.movement_message.linear.z = 0
			self.movement_message.angular.x = 0
			self.movement_message.angular.y = 0
			self.movement_message.angular.z = -self.get_anguler_speed()
			rospy.loginfo("Robot movement: move RIGHT")
			self.movement_publisher.publish(self.movement_message)
		elif command.data == "FORWARD":
			return
		else: # BACKWARD
			return

if __name__ == "__main__":
	rospy.init_node("robot_movement")
	robot_movement = RobotMovement()
	rospy.spin()