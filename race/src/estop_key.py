#!/usr/bin/env python

import rospy
from std_msgs import Bool
import curses
import time

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('e_Stopper', anonymous=True)
pub = rospy.Publisher('e_Stop', Bool, queue_size=1)

stdscr.refresh()
key = ''
eStop = True

while key != ord('q'):
	key = stdscr.getch()
	stdscr.refresh()
	if key == curses.KEY_UP: 
		stdscr.addstr(2, 20, "Start")
        eStop = False
	elif key == curses.KEY_DOWN :
		stdscr.addstr(2, 20, "eStop")
        eStop = True
	msg = Bool()
	msg.data = eStop
	pub.publish(msg)
curses.endwin()
