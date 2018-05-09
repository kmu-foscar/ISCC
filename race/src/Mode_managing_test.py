#!/usr/bin/env python


import rospy
from race.msg import sign_classes

rospy.init_node('Mode_tester', anonymous=True)
pub = rospy.Publisher('sign_classes', sign_classes, queue_size=1)

while True :
    mode = input("mode : ")
    msg = sign_classes()
    msg.sign_class = mode
    pub.publish(msg)