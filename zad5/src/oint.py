#!/usr/bin/env python

from time import sleep
from sys import stderr
from zad5.srv import OintControl, OintControlResponse
from zad5.srv import OintCyclic, OintCyclicResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from PyKDL import Rotation
from math import sin, cos, atan2, sqrt
import rospy
import copy



class Point2D:
	def __init__(self, new_x, new_y):
		self.x = new_x
		self.y = new_y


def linear_interpolation(t, x0, x1, time_period):
	return x0 + (x1-x0)*t/time_period



def polynomial_interpolation(t, x0, x1, time_period):
	a = -2*(x1-x0)/(time_period**3)
	b = 3*(x1-x0)/(time_period**2)

	return x0 + b*(t**2) + a*(t**3)



def oint_control(req):
	global pose_now
	global pub_pose
	global pub_path

	if req.mode == 'linear':
		fun = linear_interpolation
	elif req.mode == 'polynomial':
		fun = polynomial_interpolation
	else:
		print >> stderr, "Received undefined type of interpolation: %s" % req.mode.capitalize()
		return "Undefined interpolation: %s. Service interrupted! " % req.mode.capitalize()


	if not req.time > 0.0:
		print >> stderr, "Received invalid time value: %s" % req.time
		return "Passed invalid time value: %s. Expected positive float64. Service interrupted!" % req.time

	msg = PoseStamped()
	msg.header.frame_id = 'base_link'

	path = Path()
	path.header.frame_id = 'base_link'

	t = 0
	rate = rospy.Rate(20)
	while t < req.time:

		msg.header.stamp = rospy.get_rostime()

		msg.pose.position.x = fun( t, pose_now[0], req.x, req.time )
		msg.pose.position.y = fun( t, pose_now[1], req.y, req.time )
		msg.pose.position.z = fun( t, pose_now[2], req.z, req.time )

		roll  = fun( t, pose_now[3], req.roll , req.time )
		pitch = fun( t, pose_now[4], req.pitch, req.time )
		yaw   = fun( t, pose_now[5], req.yaw  , req.time )

		rot = Rotation.RPY(roll,pitch,yaw)
		quat = rot.GetQuaternion()

		msg.pose.orientation.x = quat[0]
		msg.pose.orientation.y = quat[1]
		msg.pose.orientation.z = quat[2]
		msg.pose.orientation.w = quat[3]

		pub_pose.publish(msg)

		path.header.stamp = msg.header.stamp
		path.poses.append(copy.deepcopy(msg))

		pub_path.publish(path)

		if req.time - t < 0.05:
			t = req.time
		else:
			t += 0.05
		rate.sleep()

	msg.header.stamp = rospy.get_rostime()

	msg.pose.position.x = fun( t, pose_now[0], req.x, req.time )
	msg.pose.position.y = fun( t, pose_now[1], req.y, req.time )
	msg.pose.position.z = fun( t, pose_now[2], req.z, req.time )

	roll  = fun( t, pose_now[3], req.roll , req.time )
	pitch = fun( t, pose_now[4], req.pitch, req.time )
	yaw   = fun( t, pose_now[5], req.yaw  , req.time )

	rot = Rotation.RPY(roll,pitch,yaw)
	quat = rot.GetQuaternion()

	msg.pose.orientation.x = quat[0]
	msg.pose.orientation.y = quat[1]
	msg.pose.orientation.z = quat[2]
	msg.pose.orientation.w = quat[3]

	pub_pose.publish(msg)

	path.header.stamp = msg.header.stamp
	path.poses.append(copy.deepcopy(msg))

	pub_path.publish(path)

	pose_now[0] = msg.pose.position.x
	pose_now[1] = msg.pose.position.y
	pose_now[2] = msg.pose.position.z
	pose_now[3] = roll
	pose_now[4] = pitch
	pose_now[5] = yaw

	return "%s interpolation completed" % req.mode.capitalize()



def oint_control_handle(req):
	global lock

	while lock == True:
		sleep(1)
	lock = True

	msg = oint_control(req)

	lock = False

	return msg



def move_to_elipse(a, b, z):
	global pose_now

	t_start = atan2( pose_now[1] * a, pose_now[0] * b )

	x = a * cos(t_start)
	y = b * sin(t_start)

	req = OintControl()

	req.mode = 'linear'
	req.x = x
	req.y = y
	req.z = z
	req.roll = pose_now[3]
	req.pitch = pose_now[4]
	req.yaw = pose_now[5]
	req.time = 3.0

	oint_control(req)

	return t_start



def eliptic_cycle(a, b, t_start, t, time_period):
	x = a*cos(t_start + 6.28*(t/time_period))
	y = b*sin(t_start + 6.28*(t/time_period))
	return [x,y]


def distance(point):
	global pose_now

	x_now = pose_now[0]
	y_now = pose_now[1]

	return sqrt( (point.x-x_now)**2 + (point.y-y_now)**2 )



def oint_cyclic(req):
	global pose_now
	global pub_pose
	global pub_path

	if not req.cycle_time > 0.0:
		print >> stderr, "Received invalid cycle time value: %s" % req.cycle_time
		return "Passed invalid cycle time value: %s. Expected positive float64. Service interrupted!" % req.cycle_time


	if req.mode == 'eliptic':
		msg = PoseStamped()
		msg.header.frame_id = 'base_link'

		path = Path()
		path.header.frame_id = 'base_link'

		t_start = move_to_elipse(req.a, req.b, req.z)

		rate = rospy.Rate(20)

		i = 0
		while i < req.cycle_count:
			t = 0
			while t < req.cycle_time:

				msg.header.stamp = rospy.get_rostime()

				position = eliptic_cycle(req.a, req.b, t_start, t, req.cycle_time)

				msg.pose.position.x = position[0]
				msg.pose.position.y = position[1]
				msg.pose.position.z = req.z

				rot = Rotation.RPY(pose_now[3],pose_now[4],pose_now[5])
				quat = rot.GetQuaternion()

				msg.pose.orientation.x = quat[0]
				msg.pose.orientation.y = quat[1]
				msg.pose.orientation.z = quat[2]
				msg.pose.orientation.w = quat[3]

				pub_pose.publish(msg)

				path.header.stamp = msg.header.stamp
				path.poses.append(copy.deepcopy(msg))

				pub_path.publish(path)

				if req.cycle_time - t < 0.05:
					t = req.cycle_time
				else:
					t += 0.05
				rate.sleep()

			msg.header.stamp = rospy.get_rostime()

			position = eliptic_cycle(req.a, req.b, t_start, t, req.cycle_time)

			msg.pose.position.x = position[0]
			msg.pose.position.y = position[1]
			msg.pose.position.z = req.z

			rot = Rotation.RPY(pose_now[3],pose_now[4],pose_now[5])
			quat = rot.GetQuaternion()

			msg.pose.orientation.x = quat[0]
			msg.pose.orientation.y = quat[1]
			msg.pose.orientation.z = quat[2]
			msg.pose.orientation.w = quat[3]

			pose_now[0] = msg.pose.position.x
			pose_now[1] = msg.pose.position.y
			pose_now[2] = msg.pose.position.z

			pub_pose.publish(msg)

			path.header.stamp = msg.header.stamp
			path.poses.append(copy.deepcopy(msg))

			pub_path.publish(path)

			i = i+1


	elif req.mode == 'rectangular':
		msg = PoseStamped()
		msg.header.frame_id = 'base_link'

		path = Path()
		path.header.frame_id = 'base_link' 

		vertex = [Point2D(req.a/2,req.b/2),Point2D(-req.a/2,req.b/2),Point2D(-req.a/2,-req.b/2),Point2D(req.a/2,-req.b/2),]

		distances = []
		for point in vertex:
			distances.append(distance(point))

		k_start = distances.index(min(distances))

		time_A = req.cycle_time*(req.a/(2*req.a+2*req.b))
		time_B = req.cycle_time*(req.b/(2*req.a+2*req.b))

		rate = rospy.Rate(20)

		request = OintControl()
		request.mode = 'linear'
		request.x = vertex[k_start].x
		request.y = vertex[k_start].y
		request.z = req.z
		request.roll = pose_now[3]
		request.pitch = pose_now[4]
		request.yaw = pose_now[5]
		request.time = 3

		oint_control(request)

		k = (k_start+1)%4

		i = 0
		while i < req.cycle_count:
			request = OintControl()
			request.mode = 'linear'
			request.x = vertex[k].x
			request.y = vertex[k].y
			request.z = req.z
			request.roll = pose_now[3]
			request.pitch = pose_now[4]
			request.yaw = pose_now[5]
			
			if k % 2 == 0:
				request.time = time_B
			else:
				request.time = time_A

			oint_control(request)

			if k == k_start:
				i = i+1

			k = (k+1)%4



	else:
		print >> stderr, "Received undefined type of cycle: %s" % req.mode.capitalize()
		return "Undefined cycle: %s. Service interrupted! " % req.mode.capitalize()

	return "%s cycle completed" % req.mode.capitalize()



def oint_cyclic_handle(req):
	global lock

	while lock == True:
		sleep(1)
	lock = True

	msg = oint_cyclic(req)

	lock = False

	return msg



def oint():
	global lock
	global pose_now
	global pub_pose
	global pub_path

	lock = True
	pose_now = [3.0] + [0.0]*5
	pub_pose = rospy.Publisher('/oint_rviz_pose', PoseStamped, queue_size = 1)
	pub_path = rospy.Publisher('/oint_rviz_path', Path, queue_size = 10)
	rospy.init_node('oint')
	s1 = rospy.Service('oint_control_srv', OintControl, oint_control_handle)
	s2 = rospy.Service('oint_cyclic_srv', OintCyclic, oint_cyclic_handle)
	print "Ready to interpolate."
	lock = False

	rospy.spin()

if __name__ == "__main__":
	oint()
