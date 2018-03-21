#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
import curses
import time

stdscr = curses.initscr()

curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
pub = rospy.Publisher('throttle', Bool, queue_size=1)
pub2 = rospy.Publisher('Controller', UInt16, queue_size=1)

stdscr.refresh()
key = ''
start = False

while key != ord('q'):
	key = stdscr.getch()
	stdscr.refresh()
	if key == curses.KEY_UP: 
		stdscr.addstr(2, 20, "start")
		start = True
	elif key == curses.KEY_DOWN :
		stdscr.addstr(2, 20, "stop!")
		start = False
	msg = Bool()
	msg.data = start
	pub.publish(msg)
curses.endwin()
