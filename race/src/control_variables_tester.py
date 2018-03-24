#!/usr/bin/env python

import rospy
from race.msg import control_variables

rospy.init_node('Tester', anonymous=True)
pub = rospy.Publisher('control_variables', control_variables, queue_size=1)

while True :
	p_slope = input('p_slope : ')
	p_position = input('p_position : ')
	msg = control_variables()
	msg.p_slope = float(p_slope)
	msg.p_position = float(p_position)
	pub.publish(msg)
