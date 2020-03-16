#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def turtle_custom_teleop():
	rospy.set_param("keybinds/forward", 'w')
	rospy.set_param("keybinds/backward", 's')
	rospy.set_param("keybinds/left", 'a')
	rospy.set_param("keybinds/right", 'd')
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
	rospy.init_node('my_node_name', anonymous=True)
	print("Key control is ready! (default keys: WSAD)")
	print("Press Ctrl-C to exit.")
	print("------------------------------------------")

	while not rospy.is_shutdown():
		forward = rospy.get_param("keybinds/forward")
		backward = rospy.get_param("keybinds/backward")
		left = rospy.get_param("keybinds/left")
		right = rospy.get_param("keybinds/right")

		key = getKey()

		twist = Twist()
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0

		if key == forward:
			twist.linear.x = 2
		if key == backward:
			twist.linear.x = -2
		if key == left:
			twist.angular.z = 2
		if key == right:
			twist.angular.z = -2

		pub.publish(twist)

		if ord(key) == 3:
			print("^C") # illusion 100
			break

if __name__=='__main__':
	try:
		settings = termios.tcgetattr(sys.stdin)
		turtle_custom_teleop()
	except rospy.ROSInterruptException:
		pass

	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
