#!/usr/bin/env python

import rospy
from race.msg import drive_values
import curses
import time

stdscr = curses.initscr()

curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
pub = rospy.Publisher('Control', drive_values, queue_size=1)

stdscr.refresh()
key = ''
throttle = 0
steering = 0

while key != ord('q'):
	key = stdscr.getch()
	stdscr.refresh()
	if key == curses.KEY_UP: 
		stdscr.addstr(2, 20, "Accel")
		throttle += 1
		if throttle > 1 :
			throttle = 1
	elif key == curses.KEY_DOWN :
		stdscr.addstr(2, 20, "Brake")
		throttle -= 1
		if throttle < -1 :
			throttle = -1
	elif key == curses.KEY_LEFT :
		steering -= 10
		if steering < -100 : 
			steering = -100
	elif key == curses.KEY_RIGHT :
		steering += 10
		if steering > 100 :
			steering = 100
	elif key == curses.KEY_DC :
		steering = 0
	if throttle == 0 :
		stdscr.addstr(2, 20, "Stop!")
	msg = drive_values()
	msg.throttle = throttle
	msg.steering = steering + 100
	pub.publish(msg)
curses.endwin()
