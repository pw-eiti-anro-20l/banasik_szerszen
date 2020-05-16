#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import sqrt, atan2, acos


def callback(data):
	global theta_now
	global pub

	x = data.pose.position.x
	y = data.pose.position.y
	z = data.pose.position.z
	r2 = x**2 + y**2 + z**2
	r = sqrt(r2)

	beta = abs(atan2( z,sqrt(x**2 + y**2 ) ))

	theta1_list = []
	theta2_list = []
	theta3_list = []

	theta1_list += [atan2(y,x)]
	#theta1_list += [atan2(y,x) + 3.14]

	theta2_list += [abs(acos( (r2+3)/(4*r) )) - beta]
	theta2_list += [-abs(acos( (r2+3)/(4*r) )) - beta]
	#theta2_list += [-abs(acos( (r2+3)/(4*r) )) + beta + 3.14]
	#theta2_list += [abs(acos( (r2+3)/(4*r) )) + beta + 3.14]

	theta3_list += [acos( (r2-5)/4 )]
	theta3_list += [-acos( (r2-5)/4 )]

	flag1 = True
	flag2 = False

	#if abs(theta1_list[0] - theta_now[0]) < abs(theta1_list[1] - theta_now[0]):
	#	theta1 = theta1_list[0]
	#	print("normal")
	#	flag1 = True
	#else:
	#	theta1 = theta1_list[1]
	#	print("over 90")
	#	flag1 = False

	theta1 = theta1_list[0]

	if flag1:
		if abs(theta2_list[0] - theta_now[1] + 0.01) < abs(theta2_list[1] - theta_now[1]):
			theta2 = theta2_list[0]
			flag2 = True
		else:
			theta2 = theta2_list[1]
			flag2 = False
	else:
		if abs(theta2_list[2] - theta_now[1]) < abs(theta2_list[3] - theta_now[1]):
			theta2 = theta2_list[2]
			flag2 = False
		else:
			theta2 = theta2_list[3]
			flag2 = True

	if flag2:
		theta3 = theta3_list[1]
	else:
		theta3 = theta3_list[0]


	#if abs(x) < 0.01 and abs(y) < 0.01:
	#	theta1 = theta_now[0]
	#	theta2 = theta_now[1]
	#	theta3 = theta_now[2]

	theta_now = [theta1, theta2, theta3]

	msg = JointState()
	msg.name = ['joint1', 'joint2', 'joint3']
	msg.header.stamp = rospy.get_rostime()
	msg.position = theta_now
	pub.publish(msg)


def ikin():
	global theta_now
	global pub
	
	theta_now = [0.0]*3
	pub = rospy.Publisher('/joint_states', JointState, queue_size = 1)
	rospy.init_node('ikin')
	rospy.Subscriber('/oint_rviz_pose', PoseStamped, callback)
	print "Ready for inversing zee kinemakicks problym"

	rospy.spin()


if __name__ == "__main__":
	ikin()
