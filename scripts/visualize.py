#!/usr/bin/python2

import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import *

import time


def quat2mat(q):
    qr, qi, qj, qk = q
    return np.matrix([
        [(1-2*qj*qj-2*qk*qk), 2*(qi*qj-qk*qr), 2*(qi*qk+qj*qr)],
        [2*(qi*qj+qk*qr), (1-2*qi*qi-2*qk*qk), 2*(qj*qk-qi*qr)],
        [2*(qi*qk-qj*qr), 2*(qj*qk+qi*qr), (1-2*qi*qi-2*qj*qj)]
    ])





fig = plt.figure()
ax = fig.add_subplot(111, projection='3d', aspect=1)
plt.ion()

ps =[
    np.matrix([[1],[0],[0]]),
    np.matrix([[0],[1],[0]]),
    np.matrix([[0],[0],[1]])
]
pcolors = ['r', 'g', 'b']

cam_pts = np.matrix([
    [0,0,0],
    [-1,-1,-1],
    [-1,1,-1],
    [0,0,0],
    [-1,1,-1],
    [1,1,-1],
    [0,0,0],
    [1,1,-1],
    [1,-1,-1],
    [0,0,0],
    [1,-1,-1],
    [-1,-1,-1]
])*0.5

motor_pos = [
    [2, 2, 0],
    [2, -2, 0],
    [-2, -2, 0],
    [-2, 2, 0]
]


def draw(q,t): # [w, x, y, z] quaternion

    Rt = quat2mat(q)

    ax.clear()

    # Draw axes
    for j in range(0, len(ps)):
        p = ps[j]
        px = np.dot(Rt, p)
        ax.plot([0, px[0,0]], [0, px[1,0]], [0, px[2,0]], pcolors[j] + '-')

    print(t)

    # Draw motor/throttle vectors
    for j in range(0, len(motor_pos)):
        t = [x for x in t]
        if t[j] < 0:
            t[j] = 0
        if t[j] > 1:
            t[j] = 1

        pt = motor_pos[j]
        m = np.matrix([
            pt,
            [pt[0], pt[1], pt[2] + 2*t[j]]
        ])
        vec = np.dot(Rt, m.transpose())
        ax.plot(vec[0,:].tolist()[0], vec[1,:].tolist()[0], vec[2,:].tolist()[0], 'b-' if j < 2 else 'r-')


    ax.plot([0,0], [0,0], [-5, 5], 'k-')

    # Draw quad
    #ax.plot([-2, 2], [-2, 2], [0, 0], 'k-')
    #ax.plot([-2, 2], [2, -2], [0, 0], 'k-')

    # Draw camera
    #cam = np.dot(Rt, cam_pts.transpose())
    #ax.plot(cam[0,:].tolist()[0], cam[1,:].tolist()[0], cam[2,:].tolist()[0], 'b-')


    #fig.canvas.draw()

    ax.set_xlabel('X')
    ax.set_xlim(-3, 3)
    ax.set_ylabel('Y')
    ax.set_ylim(-3, 3)
    ax.set_zlabel('Z')
    ax.set_zlim(-3, 3)

    #plt.draw()
    plt.pause(0.05)




latest_q = [1,0,0,0]
latest_t = [-1,-1,-1,-1]

def pose_callback(ps):
    global latest_q
    orient = ps.pose.orientation
    latest_q = [orient.w, orient.x, orient.y, orient.z]


def motor_callback(arr):
    global latest_t
    latest_t = arr.data




rospy.init_node('visualize_quad', anonymous=True)
rospy.Subscriber("/pose", PoseStamped, pose_callback)
rospy.Subscriber("/motors", Float32MultiArray, motor_callback)

#rospy.spin()
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    draw(latest_q, latest_t)
    r.sleep()




# http://stackoverflow.com/questions/4098131/how-to-update-a-plot-in-matplotlib
