#!/usr/bin/python
#Made by Bohan Lou Email: loubohan@gmail.com
import rospy
from pluto.srv import Grip
from std_msgs.msg import Float64
from std_msgs.msg import String
from dynamixel_msgs.msg import JointState as dxl_JointState
from pluto_common import *
#from control_msgs.msg import JointControllerState as dxl_JointState

def lf_callback(data):
	global msg 
	#msg[0] = data.i_clamp * -1
	#msg[2] = data.current_pos
	msg[0] = data.load * -1
	msg[2] = data.current_pos
 
def rf_callback(data):
	global msg
	msg[1] = data.load
	msg[3] = data.current_pos
 
def handle_gripper(req):
	global msg
	global pub_left_finger
	global pub_right_finger
	msg = [0,0,0,0]
 
 
	if req.Option == "Close":
		while msg[1] < req.Value or msg[0] < req.Value:
			msg[2] += 0.05
			msg[3] -= 0.05
			lf = Float64(msg[2]) 	
			rf = Float64(msg[3])
			
			rospy.loginfo('left_finger_'+str(lf))
			rospy.loginfo('right_finger_'+str(rf))
			rospy.loginfo('left_load_'+str(msg[0]))
			rospy.loginfo('right_load_'+str(msg[1]))
			
			pub_left_finger.publish(lf)
			pub_right_finger.publish(rf)
			rospy.sleep(0.3)
 
		return msg[1]
 
	elif req.Option == "Open":
 
		lf = Float64(req.Value) 	
		rf = Float64(req.Value * -1)
		rospy.loginfo('left_finger_'+str(lf))
		rospy.loginfo('right_finger_'+str(rf))
		pub_left_finger.publish(lf)
		pub_right_finger.publish(rf)
		rospy.sleep(0.5)
 
		return msg[2]
 
def gripper_server():
		is_simulation = False
		#print "Options: Open Close \n Angle Values: -1.20~0.8 \n Load Values: 0.00 ~ 0.15"
		global msg
		msg = [0,0,0,0]
		rospy.init_node('gripper_server')
		global pub_left_finger
		global pub_right_finger
		pub_left_finger = rospy.Publisher(pluto_add_namespace( is_simulation,'/left_finger_controller/command'), Float64, queue_size=10)
		pub_right_finger = rospy.Publisher(pluto_add_namespace( is_simulation,'/right_finger_controller/command'), Float64, queue_size=10)
		#rospy.Subscriber('/komodo_1/left_finger_controller/state', Float64, lf_callback)
		#rospy.Subscriber('/komodo_1/right_finger_controller/state', Float64, rf_callback)
		rospy.Subscriber(pluto_add_namespace( is_simulation,'/left_finger_controller/state'), dxl_JointState, lf_callback)
		rospy.Subscriber(pluto_add_namespace( is_simulation,'/right_finger_controller/state'), dxl_JointState, rf_callback)
		s = rospy.Service('gripper', Grip, handle_gripper)
		rospy.spin()
 
 
if __name__ == '__main__':
	gripper_server()