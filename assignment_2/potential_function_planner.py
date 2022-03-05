#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from math import atan2, pi, cos, sin
from helper_functions import *
# import matplotlib.pyplot as plt


rospy.init_node('test', anonymous=True)

# Initialize
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

# Read Input File
f = open("catkin_ws/src/sc627_assignments/assignment_2/input.txt", "r")
l = f.readline()
start = [float(l[0]), float(l[2])]    # start = [0, 0]
l = f.readline()
goal = [float(l[0]), float(l[2])]     # goal = [5, 3]
l = f.readline()
step = float(l)                       # step = 0.1
l = f.readline()
l = f.readline()
p1 = [float(l[0]), float(l[2])]      # p1 = [1, 2]
l = f.readline()
p2 = [float(l[0]), float(l[2])]      # p2 = [1, 0]
l = f.readline()
p3 = [float(l[0]), float(l[2])]      # p3 = [3, 0]
l = f.readline()
l = f.readline()
p4 = [float(l[0]), float(l[2])]      # p4 = [2, 3]
l = f.readline()
p5 = [float(l[0]), float(l[2])]      # p5 = [4, 1]
l = f.readline()
p6 = [float(l[0]), float(l[2])]      # p6 = [5, 2]
f.close()

P1 = [p1, p2, p3]
P2 = [p4, p5, p6]

k = 1
h = [[0, 0], [0, 0]]


# Open Output File
out = open("catkin_ws/src/sc627_assignments/assignment_2/output.txt", "a")

# Setting Initial Condition
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0  # orientation in radians (0 to 2pi)
wp = MoveXYGoal()


# Potential Functions Algorithm

while PointToPointDist(h[k], goal) > step:
    print("Start")
    [Ax, Ay] = attPotential(h[k], goal)
    [R1x, R1y] = repPotential(h[k], P1)
    [R2x, R2y] = repPotential(h[k], P2)

    [Vx, Vy, theta] = normalise([Ax+R1x+R2x, Ay+R1y+R2y])

    newx = float(h[k][0]+step*Vx)
    newy = float(h[k][1]+step*Vy)
    # print(Ax, Ay)

    wp.pose_dest.x = newx
    wp.pose_dest.y = newy
    wp.pose_dest.theta = theta
    client.send_goal(wp)
    print("msg sent")

    client.wait_for_result()
    print("msg received")
    result = client.get_result()
    h.append([result.pose_final.x, result.pose_final.y])
    out.write("\n% s," % result.pose_final.x)
    out.write("%s" % result.pose_final.y)
    k += 1
    if ((Ax**2)+(Ay**2)**0.5) < 0.2:
        break

print(h)

out.close()
