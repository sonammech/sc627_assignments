#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from math import atan2, pi, cos, sin
from helper import *
# import matplotlib.pyplot as plt

rospy.init_node('test', anonymous=True)

# Initialize
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

# Read Input File txt
f = open("catkin_ws/src/sc627_assignments/assignment_1/input.txt", "r")
l = f.readline()
start = [float(l[0]), float(l[2])]   # start = [0, 0]
l = f.readline()
goal = [float(l[0]), float(l[2])]    # goal = [5, 3]
l = f.readline()
step = float(l)                      # step = 0.1
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

# Open Output File txt
out = open("catkin_ws/src/sc627_assignments/assignment_1/output_base.txt", "a")

# Initial Location/start
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0      # orientation in radians (0 to 2pi)
wp = MoveXYGoal()


# bug_base Algorithm
while PointToPointDist(h[k], goal) > step:
    print("Start")
    [Vx, Vy, theta] = VectorToPoint(h[k], goal)
    newx = float(h[k][0]+step*Vx)
    newy = float(h[k][1]+step*Vy)

    a = PointToPolygonDist(h[k], P1)[0]
    b = PointToPolygonDist(h[k], P2)[0]
    if a < b:
        Obst = P1
    else:
        Obst = P2

    # If Obstacle Detected
    if PointToPolygonDist([newx, newy], Obst)[0] < step:
        print("Failure: There is an obstacle lying between the start and goal")
        out.write(
            "\nFailure: There is an obstacle lying between the start and goal")
        break
    else:
        # Append h
        print("moving")
        wp.pose_dest.x = newx
        wp.pose_dest.y = newy
        wp.pose_dest.theta = theta
        client.send_goal(wp)
        print("msg sent")

        client.wait_for_result()
        print("msg received")
        result = client.get_result()
        # h.append([newx, newy])
        h.append([result.pose_final.x, result.pose_final.y])
        out.write("\n% s," % result.pose_final.x)
        out.write("%s" % result.pose_final.y)

        k += 1

if PointToPointDist(h[k], goal) < step:
    print("Success: Reached the goal")
    out.write("\nSuccess: Reached the goal")
print(h)

out.close()
