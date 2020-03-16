#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Function getKey() is blocking function that returns char from stdin input steam.
# Taken inputs are not buffered with ENTER.
def getKey(): 
	# configure terminal to read raw inputs
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	# read key input
	key = sys.stdin.read(1)
	# reset terminal settings
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def turtle_custom_teleop()
	# create default settings parameters
	rospy.set_param("keybinds/forward", 'w')
	rospy.set_param("keybinds/backward", 's')
	rospy.set_param("keybinds/left", 'a')
	rospy.set_param("keybinds/right", 'd')

	# initialize communication and node
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
	rospy.init_node('turtle_custom_teleop1', anonymous=True)

	# confirm ready state
	print("Key control is ready! (default keys: WSAD)")
	print("Press Ctrl-C to exit.")
	print("------------------------------------------")

	while not rospy.is_shutdown():
		# read settings
		forward = rospy.get_param("keybinds/forward")
		backward = rospy.get_param("keybinds/backward")
		left = rospy.get_param("keybinds/left")
		right = rospy.get_param("keybinds/right")

		# read input
		key = getKey()

		# initialize default message
		twist = Twist()
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0

		# adjust message to input
		if key == forward:
			twist.linear.x = 2
		if key == backward:
			twist.linear.x = -2
		if key == left:
			twist.angular.z = 2
		if key == right:
			twist.angular.z = -2

		# publish message
		pub.publish(twist)

		# identify exit code
		if ord(key) == 3:
			print("^C") # illusion 100
			break

# 
if __name__=='__main__':
	try:
		settings = termios.tcgetattr(sys.stdin)
		turtle_custom_teleop()
	except rospy.ROSInterruptException:
		pass

	finally:
		# reset terminal settings
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
