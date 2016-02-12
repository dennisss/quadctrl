#!/usr/bin/python2

# Keyboard control of the robot platform

import rospy
import curses
from std_srvs.srv import *

from geometry_msgs.msg import * # Vector3 and twist

rospy.init_node('robot_ctrl', anonymous=True)
pub = rospy.Publisher('/setpoint', Twist, queue_size=1)

# Errors will be between -1 and 1 (for -pi to pi)
# Single axis torque limit of 1.56 (two axis limit of ~0.7)
rospy.set_param('gains/p', [0.83, 0.83, 0.12]) # < 10
rospy.set_param('gains/i', [0.1, 0.1, 0])
rospy.set_param('gains/d', [0.11, 0.11, 0]) # definately <0.8

# 0.95
# 0
# 0.16


# Tell the
rospy.wait_for_service('/configure')
configure = rospy.ServiceProxy('/configure', Empty)
try:
    configure()
except rospy.ServiceException as exc:
    print("Service did not process request: " + str(exc))



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
        throttle = throttle + 0.1 #0.1
        dirty = True
    elif key == curses.KEY_DOWN:
        throttle = throttle - 0.1
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
