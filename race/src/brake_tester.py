#!/usr/bin/env python

import rospy
from race.msg import drive_values

rospy.init_node('Tester', anonymous = True)
pub = rospy.Publisher('Control', drive_values, queue_size = 1)

while True :
    throttle = input('throttle : ')
    steering = input('steering : ')
    msg = drive_values()
    msg.throttle = throttle
    msg.steering = steering
    pub.publish(msg)
