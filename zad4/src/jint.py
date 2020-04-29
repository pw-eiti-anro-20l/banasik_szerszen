#!/usr/bin/env python

from time import sleep
from sys import stderr
from zad4.srv import JintControl, JintControlResponse
from sensor_msgs.msg import JointState
import rospy

def cut_theta(theta):
    out = [0.0]*3
    
    if theta[0] > 3.14:
        out[0] = 3.14
        print >> stderr, "Theta 1 exceeded upper bounds. Value reduced to 3.14."
    elif theta[0] < -3.14:
        out[0] = -3.14
        print >> stderr, "Theta 1 exceeded lower bounds. Value increased to -3.14."
    else:
        out[0] = theta[0]

    if theta[1] > 0:
        out[1] = 0
        print >> stderr, "Theta 2 exceeded upper bounds. Value reduced to 0."
    elif theta[1] < -3.14:
        out[1] = -3.14
        print >> stderr, "Theta 2 exceeded lower bounds. Value increased to -3.14."
    else:
        out[1] = theta[1]

    if theta[2] > 2.35:
        out[2] = 2.35
        print >> stderr, "Theta 3 exceeded upper bounds. Value reduced to 2.35."
    elif theta[2] < -2.35:
        out[2] = -2.35
        print >> stderr, "Theta 3 exceeded lower bounds. Value increased to -2.35."
    else:
        out[2] = theta[2]

    return out

def linear_interpolation(t, x0, x1, time_period):
    return x0 + (x1-x0)*t/time_period

def polynomial_interpolation(t, x0, x1, time_period):
    a = -2*(x1-x0)/(time_period**3)
    b = 3*(x1-x0)/(time_period**2)

    return x0 + b*(t**2) + a*(t**3)

def handle(req):
    global lock
    global theta_now
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

    theta_trg = cut_theta([req.theta1, req.theta2, req.theta3])

    if not req.time > 0.0:
        print >> stderr, "Received invalid time value: %s" % req.time
        lock = False
        return "Passed invalid time value: %s. Expected positive float64. Service interrupted!" % req.time

    msg = JointState()

    msg.name = ['joint1', 'joint2', 'joint3']

    t = 0
    rate = rospy.Rate(20)
    while t < req.time:

        msg.header.stamp = rospy.get_rostime()
        theta = [0.0]*3

        theta[0] = fun(t, theta_now[0], theta_trg[0], req.time)
        theta[1] = fun(t, theta_now[1], theta_trg[1], req.time)
        theta[2] = fun(t, theta_now[2], theta_trg[2], req.time)

        msg.position = theta

        pub.publish(msg)

        if req.time - t < 0.05:
            t = req.time
        else:
            t += 0.05
        rate.sleep()

    msg.header.stamp = rospy.get_rostime()
    theta = [0.0]*3

    theta[0] = fun(t, theta_now[0], theta_trg[0], req.time)
    theta[1] = fun(t, theta_now[1], theta_trg[1], req.time)
    theta[2] = fun(t, theta_now[2], theta_trg[2], req.time)

    msg.position = theta

    pub.publish(msg)

    theta_now = theta

    lock = False
    return "%s interpolation completed" % req.mode.capitalize()

def jint():
    global lock
    global theta_now
    global pub

    lock = False
    theta_now = [0.0]*3
    pub = rospy.Publisher('/joint_states', JointState, queue_size = 1)
    rospy.init_node('jint')
    s = rospy.Service('jint_control_srv', JintControl, handle)
    print "Ready to interpolate."

    sleep(1)
    msg = JointState()
    msg.header.stamp = rospy.get_rostime()
    msg.name = ['joint1', 'joint2', 'joint3']
    msg.position = [0.0]*3
    pub.publish(msg)

    rospy.spin()

if __name__ == "__main__":
    jint()