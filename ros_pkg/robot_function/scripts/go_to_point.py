#! /usr/bin/env python3
# import roslib
# roslib.load_manifest('motion_plan')
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math
import matplotlib.pyplot as plt
import numpy as np

active_ = False

current_position_ = Point()
yaw_ = 0
state_ = 0
desired_position_ = None
desired_position_ = Point()

# desired_position_.x = rospy.get_param('des_pos_x')
# desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.x = 0
desired_position_.y = -0.5
desired_position_.z = 0
yaw_precision_ = math.pi/90
distance_precision_ = 0.05 #0.1
pub = None


def main():
	global pub, active_, current_plan_state
	rospy.init_node('go_to_goal')
	pub = rospy.Publisher('sim_ros_interface/cmd_vel', Twist, queue_size=1)
	
	odomSub = rospy.Subscriber('sim_ros_interface/odom', Odometry, odom_callback)
	desiredPoseSub = rospy.Subscriber('sim_ros_interface/desired_pose', Point, desired_pose_callback)

	srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if not active_:
			continue
		if desired_position_ == None:
			continue
		else:
			print(state_)
			if state_ == 0:
				correct_yaw(desired_position_)
			elif state_ == 1:
				correct_linear(desired_position_)
			elif state_ == 2:
				reached()
				pass
			else:
				rospy.logerr('Unknown state')
				pass
		rate.sleep()

def go_to_point_switch(req):
	global active_
	active_ = req.data
	res = SetBoolResponse()
	res.success = True
	res.message  = 'Done'
	return res

def correct_yaw(des_position):
	global yaw_, pub, state_, yaw_precision_
	required_yaw = math.atan2(des_position.y-current_position_.y, des_position.x-current_position_.x)
	error_yaw = required_yaw-yaw_
	print(required_yaw, yaw_, error_yaw)

	twist = Twist()
	mode = None # 0 - clockwise,	1 - anticlockwise

	if math.fabs(error_yaw)<=math.pi:
		# check opposite shorter path
		if yaw_<0:
			mode = 1
		else:
			mode = 0
	else:
		if yaw_<0:
			mode = 0
		else:
			mode = 1
	while(math.fabs(required_yaw-yaw_)>yaw_precision_):
		if mode==1:
			twist.angular.z = 2.0
		elif mode==0:
			twist.angular.z = -2.0
		pub.publish(twist)

		if (math.fabs(required_yaw-yaw_)<=yaw_precision_):
			twist.angular.z = 0
			print ('Error in yaw: %s' % (required_yaw-yaw_))
			update_state(1)
		
		

def correct_linear(des_position):
	global yaw_, pub, state_, yaw_precision_
	required_yaw = math.atan2(des_position.y-current_position_.y, des_position.x-current_position_.x)
	error_yaw = required_yaw-yaw_
	error_pos = math.sqrt(pow(des_position.y-current_position_.y,2) + pow(des_position.x-current_position_.x,2))

	if error_pos > distance_precision_:
		twist = Twist()
		twist.linear.x = 2 #1
		pub.publish(twist)
	else:
		print ('Error in position: %s' % error_pos)
		update_state(2)

	if math.fabs(error_yaw) > yaw_precision_:
		print ('Error in yaw: %s' % error_yaw)
		update_state(0)

def reached():
	twist = Twist()
	twist.linear.x = 0
	twist.angular.z = 0
	pub.publish(twist)

def update_state(state):
	global state_
	state_ = state
	print ('State changed to: %s' % state_)

def odom_callback(msg):
	global current_position_, yaw_
	current_position_ = msg.pose.pose.position
	# print(current_position_)
	quaternion_data = (msg.pose.pose.orientation.x,
				   msg.pose.pose.orientation.y,
				   msg.pose.pose.orientation.z,
				   msg.pose.pose.orientation.w)
	euler_data = transformations.euler_from_quaternion(quaternion_data)
	yaw_ = euler_data[2]
	# print(yaw_)

def desired_pose_callback(msg):
	global desired_position_
	desired_position_ = msg
	print(desired_position_)

if __name__=='__main__':
		main()