#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from PyKDL import Rotation, Vector, Frame


def callback(data, args):
	x = args[0]
	y = args[1]
	z = args[2]
	roll = args[3]
	pitch = args[4]
	yaw = args[5]

	rot1 = Rotation.RPY(roll[0], pitch[0], yaw[0])
	rot2 = Rotation.RPY(roll[1], pitch[1], yaw[1])
	rot3 = Rotation.RPY(roll[2], pitch[2], yaw[2])

	vec1 = Vector( x[0], y[0], z[0] )
	vec2 = Vector( x[1], y[1], z[1] )
	vec3 = Vector( x[2], y[2], z[2] )

	rot1.DoRotZ(data.position[0])
	rot2.DoRotZ(data.position[1])
	rot3.DoRotZ(data.position[2])

	T01 = Frame( rot1, vec1 )
	T12 = Frame( rot2, vec2 )
	T23 = Frame( rot3, vec3 )

	T02 = T01*T12
	T03 = T02*T23
		
	msg = PoseStamped()
	msg.header.stamp = data.header.stamp
	msg.header.frame_id = "/base_link"

	msg.pose.position.x = T01.p.x()
	msg.pose.position.y = T01.p.y()
	msg.pose.position.z = T01.p.z()

	msg.pose.orientation.x = T01.M.GetQuaternion()[0]
	msg.pose.orientation.y = T01.M.GetQuaternion()[1]
	msg.pose.orientation.z = T01.M.GetQuaternion()[2]
	msg.pose.orientation.w = T01.M.GetQuaternion()[3]

	pub1.publish(msg)

	msg.pose.position.x = T02.p.x()
	msg.pose.position.y = T02.p.y()
	msg.pose.position.z = T02.p.z()

	msg.pose.orientation.x = T02.M.GetQuaternion()[0]
	msg.pose.orientation.y = T02.M.GetQuaternion()[1]
	msg.pose.orientation.z = T02.M.GetQuaternion()[2]
	msg.pose.orientation.w = T02.M.GetQuaternion()[3]

	pub2.publish(msg)

	msg.pose.position.x = T03.p.x()
	msg.pose.position.y = T03.p.y()
	msg.pose.position.z = T03.p.z()

	msg.pose.orientation.x = T03.M.GetQuaternion()[0]
	msg.pose.orientation.y = T03.M.GetQuaternion()[1]
	msg.pose.orientation.z = T03.M.GetQuaternion()[2]
	msg.pose.orientation.w = T03.M.GetQuaternion()[3]

	pub3.publish(msg)

def kdl_dkin():
	
	global pub1
	global pub2
	global pub3

	pub1 = rospy.Publisher('kdl_rviz1', PoseStamped, queue_size = 1)
	pub2 = rospy.Publisher('kdl_rviz2', PoseStamped, queue_size = 1)
	pub3 = rospy.Publisher('kdl_rviz3', PoseStamped, queue_size = 1)

	rospy.init_node('KDL_DKIN', anonymous=True)

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
		kdl_dkin()
	except rospy.ROSInterruptException:
		pass
