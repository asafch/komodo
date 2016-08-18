#!/usr/bin/python
# Adapted from code by Bohan Lou Email: loubohan@gmail.com
import rospy
from jupiter.srv import Fingers
from std_msgs.msg import String, Float64
from dynamixel_msgs.msg import JointState as dxl_JointState
from common import *

def left_finger_result(data):
	global msg
	msg[0] = data.load * -1
	msg[2] = data.current_pos
 
def right_finger_result(data):
	global msg
	msg[1] = data.load
	msg[3] = data.current_pos
 
def handle_request(request):
	global msg
	global left_finger_publisher
	global right_finger_publisher
	msg = [0,0,0,0]
	if request.Action == "CLOSE_FINGERS":
		while msg[1] < request.Value or msg[0] < request.Value:
			msg[2] += 0.05
			msg[3] -= 0.05
			lf = Float64(msg[2]) 	
			rf = Float64(msg[3])
			rospy.loginfo('left_finger_'+str(lf))
			rospy.loginfo('right_finger_'+str(rf))
			rospy.loginfo('left_load_'+str(msg[0]))
			rospy.loginfo('right_load_'+str(msg[1]))
			left_finger_publisher.publish(lf)
			right_finger_publisher.publish(rf)
			rospy.sleep(0.3)
		return msg[1]
	elif request.Action == "OPEN_FINGERS":
		lf = Float64(request.Value) 	
		rf = Float64(request.Value * -1)
		rospy.loginfo('left_finger_'+str(lf))
		rospy.loginfo('right_finger_'+str(rf))
		left_finger_publisher.publish(lf)
		right_finger_publisher.publish(rf)
		rospy.sleep(0.5)
		return msg[2]
	else:
		raise exception("Finger movement: illegal action")
 
if __name__ == '__main__':
	is_simulation = False
	global msg
	msg = [0,0,0,0]
	rospy.init_node("finger_movement")
	global left_finger_publisher
	global right_finger_publisher
	left_finger_publisher = rospy.Publisher(adjust_namespace(is_simulation,'/left_finger_controller/command'), Float64, queue_size = 10)
	right_finger_publisher = rospy.Publisher(adjust_namespace(is_simulation,'/right_finger_controller/command'), Float64, queue_size = 10)
	rospy.Subscriber(adjust_namespace(is_simulation,'/left_finger_controller/state'), dxl_JointState, left_finger_result)
	rospy.Subscriber(adjust_namespace(is_simulation,'/right_finger_controller/state'), dxl_JointState, right_finger_result)
	s = rospy.Service("fingers", Fingers, handle_request)
	rospy.spin()