#!/usr/bin/python2

# Keyboard control of the robot platform

import rospy
import curses

from geometry_msgs.msg import * # Vector3 and twist

rospy.init_node('robot_ctrl', anonymous=True)
pub = rospy.Publisher('/setpoint', Twist, queue_size=1)


rospy.set_param('gains/p', [10, 10, 10])
rospy.set_param('gains/i', [4, 4, 0])
rospy.set_param('gains/d', [0, 0, 0])



stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0,10,"Hit 'q' to quit")
stdscr.refresh()

throttle = 0

rotation = [0,0,0] # roll, pitch, yaw

key = ''
while True:
    key = stdscr.getch()

    if key == ord('q'):
        break

    dirty = False

    if key == ord('z'):
        throttle = 0
        rotation = [0,0,0]
        dirty = True

    stdscr.refresh()
    if key == curses.KEY_UP:
        throttle = throttle + 0.4
        dirty = True
    elif key == curses.KEY_DOWN:
        throttle = throttle - 0.4
        dirty = True
    elif key == ord('w'):
        rotation[1] = rotation[1] + 0.05
        dirty = True
    elif key == ord('s'):
        rotation[1] = rotation[1] - 0.05
        dirty = True
    elif key == ord('d'):
        rotation[0] = rotation[0] + 0.05
        dirty = True
    elif key == ord('a'):
        rotation[0] = rotation[0] - 0.05
        dirty = True
    #elif key == curses.KEY_RIGHT:
    #    v.angular.z = -0.2
    #    dirty = True
    #elif key == curses.KEY_LEFT:
    #    v.angular.z = 0.2
    #    dirty = True
    #elif key == ord('s'):
    #    dirty = True # Stop

    for i in range(0, len(rotation)):
        rotation[i] = round(rotation[i], 4)
    throttle = round(throttle, 4)

    v = Twist(
        Vector3(0,0,throttle),
        Vector3(rotation[0],rotation[1],rotation[2])
    )

    stdscr.addstr(1, 10, str(throttle) + "   ")
    stdscr.addstr(3, 10, "roll:  " + str(rotation[0]) + "   ")
    stdscr.addstr(4, 10, "pitch: " + str(rotation[1]) + "   ")
    stdscr.addstr(5, 10, "yaw:   " + str(rotation[2]) + "   ")

    if dirty:
        pub.publish(v)
        dirty = False

curses.endwin()
