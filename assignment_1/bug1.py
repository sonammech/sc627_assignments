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
start = [float(l[0]), float(l[2])]  # start = [0, 0]
l = f.readline()
goal = [float(l[0]), float(l[2])]   # goal = [5, 3]
l = f.readline()
step = float(l)                    # step = 0.1
l = f.readline()
l = f.readline()
p1 = [float(l[0]), float(l[2])]    # p1 = [1, 2]
l = f.readline()
p2 = [float(l[0]), float(l[2])]    # p2 = [1, 0]
l = f.readline()
p3 = [float(l[0]), float(l[2])]    # p3 = [3, 0]
l = f.readline()
l = f.readline()
p4 = [float(l[0]), float(l[2])]    # p4 = [2, 3]
l = f.readline()
p5 = [float(l[0]), float(l[2])]    # p5 = [4, 1]
l = f.readline()
p6 = [float(l[0]), float(l[2])]    # p6 = [5, 2]
f.close()


P1 = [p1, p2, p3]
P2 = [p4, p5, p6]


k = 1
h = [[0, 0], [0, 0]]


# Open Output File txt
out = open("catkin_ws/src/sc627_assignments/assignment_1/output_bug1.txt", "a")

# Initial Location/start
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0  # in radians (0 to 2pi)
wp = MoveXYGoal()


# Bug1 Algorithm
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
        [V1, V2, theta] = (TangentVectorToPolygon(h[k], Obst))
        newx1 = float(h[k][0]+step*V1)
        newy1 = float(h[k][1]+step*V2)

        # Append h
        wp.pose_dest.x = newx1
        wp.pose_dest.y = newy1
        wp.pose_dest.theta = theta
        client.send_goal(wp)

        client.wait_for_result()
        result = client.get_result()
        # h.append([newx1, newy1])
        h.append([result.pose_final.x, result.pose_final.y])
        out.write("\n% s," % result.pose_final.x)
        out.write("%s" % result.pose_final.y)

        k = k+1
        hit = k
        print("h hit", h[hit])
        [V1, V2, theta] = (TangentVectorToPolygon(h[k], Obst))
        newx1 = float(h[k][0]+step*V1)
        newy1 = float(h[k][1]+step*V2)

        # Append h
        # print("hk1", h[k], k)
        wp.pose_dest.x = newx1
        wp.pose_dest.y = newy1
        wp.pose_dest.theta = theta
        client.send_goal(wp)

        client.wait_for_result()
        result = client.get_result()
        # h.append([newx1, newy1])
        h.append([result.pose_final.x, result.pose_final.y])
        out.write("\n% s," % result.pose_final.x)
        out.write("%s" % result.pose_final.y)

        k = k+1
        [V1, V2, theta] = (TangentVectorToPolygon(h[k], Obst))
        newx1 = float(h[k][0]+step*V1)
        newy1 = float(h[k][1]+step*V2)

        # Append h
        # print("hk1", h[k], k)
        wp.pose_dest.x = newx1
        wp.pose_dest.y = newy1
        wp.pose_dest.theta = theta
        client.send_goal(wp)

        client.wait_for_result()
        result = client.get_result()
        # h.append([newx1, newy1])
        h.append([result.pose_final.x, result.pose_final.y])
        out.write("\n% s," % result.pose_final.x)
        out.write("%s" % result.pose_final.y)

        k = k+1

        # Original Hit Point not achieved
        while not PointToPointDist(h[k], h[hit]) < step:
            if PointToPolygonDist(h[k], Obst)[0] > 2*step:
                T = PointToPolygonDist(h[k], Obst)[2]
                [V1, V2, theta] = VectorToPoint(h[k], T)
                # print(h[k])
                # print("T is ",T)
            else:
                [V1, V2, theta] = (TangentVectorToPolygon(h[k], Obst))
                T = PointToPolygonDist(h[k], Obst)[2]
                # print("T is ",T)
            # print("v",V1,V2)
            newx1 = float(h[k][0]+step*V1)
            newy1 = float(h[k][1]+step*V2)

            # Append h
            wp.pose_dest.x = newx1
            wp.pose_dest.y = newy1
            wp.pose_dest.theta = theta
            client.send_goal(wp)
            print("send1")

            client.wait_for_result()
            print("rec1")
            result = client.get_result()
            # h.append([newx1, newy1])
            h.append([result.pose_final.x, result.pose_final.y])
            out.write("\n% s," % result.pose_final.x)
            out.write("%s" % result.pose_final.y)

            k = k+1

        # print(h)
        print("out", h[k])
        i = hit+2
        minx = h[hit][0]
        miny = h[hit][1]
        print(h)
        while i < k:
            if PointToPointDist(h[i], goal) < PointToPointDist([minx, miny], goal):
                minx = h[i][0]
                miny = h[i][1]

            i = i+1
        print("min", minx, miny)
        pleave = [minx, miny]
        while not PointToPointDist(h[k], pleave) < step:
            # print("in")
            if PointToPolygonDist(h[k], Obst)[0] > 2*step:
                T = PointToPolygonDist(h[k], Obst)[2]
                [V1, V2, theta] = VectorToPoint(h[k], T)
                # print(h[k])
                # print("T is",T)
            else:
                [V1, V2, theta] = (TangentVectorToPolygon(h[k], Obst))
                T = PointToPolygonDist(h[k], Obst)[2]
            newx1 = float(h[k][0]+step*V1)
            newy1 = float(h[k][1]+step*V2)

            # Append h
            wp.pose_dest.x = newx1
            wp.pose_dest.y = newy1
            wp.pose_dest.theta = theta
            client.send_goal(wp)

            client.wait_for_result()
            result = client.get_result()
            # h.append([newx1, newy1])
            h.append([result.pose_final.x, result.pose_final.y])
            out.write("\n% s," % result.pose_final.x)
            out.write("%s" % result.pose_final.y)

            k = k+1
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
        # h.append([newx1, newy1])
        h.append([result.pose_final.x, result.pose_final.y])
        out.write("\n% s," % result.pose_final.x)
        out.write("%s" % result.pose_final.y)

        k += 1

print(h)

out.close()
