#!/usr/bin/env python

import rospy
from race.msg import control_variables

rospy.init_node('Tester', anonymous=True)
pub = rospy.Publisher('control_variables', control_variables, queue_size=1)

while True :
	#p_steering = input('p_steering : ')
	p_steering = -0.3
	p_steering_curve = 100
	#p_steering_curve = input('p_steering_curve : ')
	test_speed = input('test_speed : ')
	msg = control_variables()
	msg.p_steering = float(p_steering)
	msg.p_steering_curve = float(p_steering_curve)
	msg.test_speed = test_speed
	pub.publish(msg)
