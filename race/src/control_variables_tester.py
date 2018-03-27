#!/usr/bin/env python

import rospy
from race.msg import control_variables

rospy.init_node('Tester', anonymous=True)
pub = rospy.Publisher('control_variables', control_variables, queue_size=1)

while True :
	p_steering = input('p_steering : ')
	msg = control_variables()
	msg.p_steering = float(p_steering)
	pub.publish(msg)
