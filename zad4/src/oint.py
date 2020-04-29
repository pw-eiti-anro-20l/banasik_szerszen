#!/usr/bin/env python

from time import sleep
from sys import stderr
from zad4.srv import OintControl, OintControlResponse
from geometry_msgs.msg import PoseStamped
from PyKDL import Rotation
import rospy

def linear_interpolation(t, x0, x1, time_period):
    return x0 + (x1-x0)*t/time_period

def polynomial_interpolation(t, x0, x1, time_period):
    a = -2*(x1-x0)/(time_period**3)
    b = 3*(x1-x0)/(time_period**2)

    return x0 + b*(t**2) + a*(t**3)

def handle(req):
    global lock
    global pose_now
    global pub

    while lock == True:
        sleep(1)
    lock = True

    if req.mode == 'linear':
        fun = linear_interpolation
    elif req.mode == 'polynomial':
        fun = polynomial_interpolation
    else:
        print >> stderr, "Received undefined type of interpolation: %s" % req.mode.capitalize()
        lock = False
        return "Undefined interpolation: %s. Service interrupted! " % req.mode.capitalize()


    if not req.time > 0.0:
        print >> stderr, "Received invalid time value: %s" % req.time
        lock = False
        return "Passed invalid time value: %s. Expected positive float64. Service interrupted!" % req.time

    msg = PoseStamped()
    msg.header.frame_id = 'map'

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

        pub.publish(msg)

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

    pub.publish(msg)

    pose_now[0] = msg.pose.position.x
    pose_now[1] = msg.pose.position.y
    pose_now[2] = msg.pose.position.z
    pose_now[3] = roll
    pose_now[4] = pitch
    pose_now[5] = yaw

    lock = False
    return "%s interpolation completed" % req.mode.capitalize()

def jint():
    global lock
    global pose_now
    global pub

    lock = False
    pose_now = [0.0]*6
    pub = rospy.Publisher('/oint_rviz', PoseStamped, queue_size = 1)
    rospy.init_node('oint')
    s = rospy.Service('oint_control_srv', OintControl, handle)
    print "Ready to interpolate."

    rospy.spin()

if __name__ == "__main__":
    jint()