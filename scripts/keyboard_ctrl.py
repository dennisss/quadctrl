#!/usr/bin/python2

# Keyboard control of the robot platform

import rospy
import curses

from geometry_msgs.msg import * # Vector3 and twist

rospy.init_node('robot_ctrl', anonymous=True)
pub = rospy.Publisher('/setpoint', Twist, queue_size=1)


stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0,10,"Hit 'q' to quit")
stdscr.refresh()

throttle = 0

key = ''
while True:
    key = stdscr.getch()

    if key == ord('q'):
        break

    dirty = False

    if key == ord('z'):
        throttle = 0
        dirty = True

    stdscr.refresh()
    if key == curses.KEY_UP:
        throttle = throttle + 0.01
        dirty = True
    elif key == curses.KEY_DOWN:
        throttle = throttle - 0.01
        dirty = True
    #elif key == curses.KEY_RIGHT:
    #    v.angular.z = -0.2
    #    dirty = True
    #elif key == curses.KEY_LEFT:
    #    v.angular.z = 0.2
    #    dirty = True
    #elif key == ord('s'):
    #    dirty = True # Stop

    v = Twist(
        Vector3(0,0,throttle),
        Vector3(0,0,0)
    )

    stdscr.addstr(1, 10, str(throttle))

    if dirty:
        pub.publish(v)
        dirty = False

curses.endwin()
