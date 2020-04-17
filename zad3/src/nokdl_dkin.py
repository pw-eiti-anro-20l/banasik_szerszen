#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from numpy import array, append, dot
from math import sin, cos, sqrt

def Vec(x,y,z):
	return array([[x],[y],[z]])

def Rpy(roll, pitch, yaw):
	return array([[cos(yaw)*cos(pitch),-sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll),sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll)],
[sin(yaw)*cos(pitch),cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll),-cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll)],[-sin(pitch),cos(pitch)*sin(roll),cos(pitch)*cos(roll)]])

def T(rot, vec):
	t = append(rot,vec, axis=1)
	t = append(t, array([[0,0,0,1]]), axis=0)
	return t

def Quat(rot):
	w = sqrt(1+rot[0,0]+rot[1,1]+rot[2,2])/2
	x = (rot[2,1]-rot[1,2])/(4*w)
	y = (rot[0,2]-rot[2,0])/(4*w)
	z = (rot[1,0]-rot[0,1])/(4*w)
	return [x, y, z, w]

def callback(data, args):
	x = args[0]
	y = args[1]
	z = args[2]
	roll = args[3]
	pitch = args[4]
	yaw = args[5]

	rot1 = Rpy(roll[0], pitch[0], yaw[0])
	rot2 = Rpy(roll[1], pitch[1], yaw[1])
	rot3 = Rpy(roll[2], pitch[2], yaw[2])

	vec1 = Vec(x[0],y[0],z[0])
	vec2 = Vec(x[1],y[1],z[1])
	vec3 = Vec(x[2],y[2],z[2])

	rotZ1 = Rpy(0,0,data.position[0])
	rotZ2 = Rpy(0,0,data.position[1])
	rotZ3 = Rpy(0,0,data.position[2])

	rot1 = dot(rot1, rotZ1)
	rot2 = dot(rot2, rotZ2)
	rot3 = dot(rot3, rotZ3)

	T01 = T(rot1, vec1)
	T12 = T(rot2, vec2)
	T23 = T(rot3, vec3)

	T02 = dot(T01, T12)
	T03 = dot(T02, T23)

	msg = PoseStamped()
	msg.header.stamp = data.header.stamp
	msg.header.frame_id = "/base_link"

	msg.pose.position.x = T01[0,3]
	msg.pose.position.y = T01[1,3]
	msg.pose.position.z = T01[2,3]

	msg.pose.orientation.x = Quat(T01)[0]
	msg.pose.orientation.y = Quat(T01)[1]
	msg.pose.orientation.z = Quat(T01)[2]
	msg.pose.orientation.w = Quat(T01)[3]

	pub1.publish(msg)

	msg.pose.position.x = T02[0,3]
	msg.pose.position.y = T02[1,3]
	msg.pose.position.z = T02[2,3]

	msg.pose.orientation.x = Quat(T02)[0]
	msg.pose.orientation.y = Quat(T02)[1]
	msg.pose.orientation.z = Quat(T02)[2]
	msg.pose.orientation.w = Quat(T02)[3]

	pub2.publish(msg)

	msg.pose.position.x = T03[0,3]
	msg.pose.position.y = T03[1,3]
	msg.pose.position.z = T03[2,3]

	msg.pose.orientation.x = Quat(T03)[0]
	msg.pose.orientation.y = Quat(T03)[1]
	msg.pose.orientation.z = Quat(T03)[2]
	msg.pose.orientation.w = Quat(T03)[3]

	pub3.publish(msg)

def nokdl_dkin():
	
	global pub1
	global pub2
	global pub3

	pub1 = rospy.Publisher('nokdl_rviz1', PoseStamped, queue_size = 1)
	pub2 = rospy.Publisher('nokdl_rviz2', PoseStamped, queue_size = 1)
	pub3 = rospy.Publisher('nokdl_rviz3', PoseStamped, queue_size = 1)

	rospy.init_node('NOKDL_DKIN', anonymous=True)

	x = []
	y = []
	z = []
	roll = []
	pitch = []
	yaw = []

	for i in range(1,4):
		if rospy.has_param('/x%d' % i ):
			x.append( rospy.get_param('/x%d' % i ))
		else:
			print >> sys.stderr, "Failed reading parameters!"
			sys.exit(1)

	for i in range(1,4):
		if rospy.has_param('/y%d' % i ):
			y.append( rospy.get_param('/y%d' % i ))
		else:
			print >> sys.stderr, "Failed reading parameters!"
			sys.exit(1)

	for i in range(1,4):
		if rospy.has_param('/z%d' % i ):
			z.append( rospy.get_param('/z%d' % i ))
		else:
			print >> sys.stderr, "Failed reading parameters!"
			sys.exit(1)

	for i in range(1,4):
		if rospy.has_param('/roll%d' % i ):
			roll.append( rospy.get_param('/roll%d' % i ))
		else:
			print >> sys.stderr, "Failed reading parameters!"
			sys.exit(1)

	for i in range(1,4):
		if rospy.has_param('/pitch%d' % i ):
			pitch.append( rospy.get_param('/pitch%d' % i ))
		else:
			print >> sys.stderr, "Failed reading parameters!"
			sys.exit(1)

	for i in range(1,4):
		if rospy.has_param('/yaw%d' % i ):
			yaw.append( rospy.get_param('/yaw%d' % i ))
		else:
			print >> sys.stderr, "Failed reading parameters!"
			sys.exit(1)

	rospy.Subscriber('/joint_states', JointState, callback, (x, y, z, roll, pitch, yaw ))
	
	rospy.spin()
	

if __name__ == '__main__':
	try:
		nokdl_dkin()
	except rospy.ROSInterruptException:
		pass