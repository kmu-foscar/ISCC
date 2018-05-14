#!/usr/bin/env python

import rospy
from race.msg import drive_values

rospy.init_node('Tester', anonymous = True)
pub = rospy.Publisher('Control', drive_values, queue_size = 1)

while True :
    throttle = input('throttle : ')
    steering = input('steering : ')
    msg = control_variables()
    msg.throttle = throttle
    msg.steering = float(steering)
    pub.publish(msg)
