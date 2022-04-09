#!/usr/bin/env python

import rospy
from sc627_helper.msg import ObsData
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from math import sin, cos, atan2, pi
import numpy as np
import time
import matplotlib.pyplot as plt

ANG_MAX = math.pi/18
VEL_MAX = 0.15
Obs_data = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
odm = [0, 0, 0, 0, 0]


def col_check(candidate_vel, bot_pose, obs_pose):
    a = 0.2
    A = max(((bot_pose[0]-obs_pose[1])**2 +
            (bot_pose[1]-obs_pose[2])**2)**0.5, 0.3)

    alpha = np.arcsin(a/A)
    theta1 = atan2(-(bot_pose[1]-obs_pose[2]), -(bot_pose[0]-obs_pose[1]))
    beta = atan2(candidate_vel[1]-obs_pose[4], candidate_vel[0]-obs_pose[3])

    if beta < theta1+alpha and beta > theta1-alpha:
        return 1
    else:
        return 0


def vector_search(bot_pose, all_obs, goal):
    ind = 0
    optimum = [0, 0]  # [maginitude, angle]
    mag = []
    ang = []
    ang_goal = atan2(goal[1]-bot_pose[1], goal[0]-bot_pose[0])
    for i in range(15):
        mag.append(0.01+i*0.01)

    for i in range(21):
        ang.append((-10+i)*pi/180)

    for i in range(len(mag)):
        for j in range(len(ang)):
            can_x = mag[i]*cos(ang[j]+bot_pose[2])
            can_y = mag[i]*sin(ang[j]+bot_pose[2])

            col_obs = 0
            for k in range(3):
                col_obs = col_obs + \
                    col_check([can_x, can_y], bot_pose, all_obs[k])
            if col_obs == 0:
                if ind == 0:
                    optimum = [mag[i], ang[j]]
                    ind = 1
                elif abs((ang[j]+bot_pose[2])-goal[1]) <= abs((optimum[1]+bot_pose[2])-goal[1]):
                    optimum = [mag[i], ang[j]]   # print("ang_cond")
                elif ang[j] == optimum[1] and optimum[i] > optimum[0]:
                    optimum = [mag[i], ang[j]]   # print(" mag_cond")

    vel_x = optimum[0]*cos(optimum[1]+bot_pose[2])
    vel_y = optimum[0]*sin(optimum[1]+bot_pose[2])

    return [vel_x, vel_y]


def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 1  # modify if necessary
    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi

    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin = min(max(math.cos(ang_err) * math.sqrt(vel_x **
                2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang


def callback_obs(data):
    '''
    Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
    '''
    global Obs_data

    for i, j in zip(data.obstacles, range(len(data.obstacles))):
        Obs_data[j][0] = i.obs
        Obs_data[j][1] = i.pose_x
        Obs_data[j][2] = i.pose_y
        Obs_data[j][3] = i.vel_x
        Obs_data[j][4] = i.vel_y
    return Obs_data


def callback_odom(data):
    '''
    Get robot data
    '''
    global odm
    odm[0] = data.pose.pose.position.x
    odm[1] = data.pose.pose.position.y
    ori = data.pose.pose.orientation
    (roll, pitch, th) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    odm[2] = th  # data.odom.twist.twist.linear.x
    odm[3] = data.twist.twist.linear.x
    odm[4] = data.twist.twist.angular.z


rospy.init_node('assign3_skeleton', anonymous=True)
rospy.Subscriber('/obs_data', ObsData, callback_obs)  # topic name fixed
rospy.Subscriber('/bot_1/odom', Odometry, callback_odom)  # topic name fixed

publish_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size=10)
r = rospy.Rate(30)
goal = [5, 0]
x = 0
y = 0
f = 0

output = open(
    "/home/user/catkin_ws/src/sc627_assignments/assignment_3/output.txt", "a")


while ((x-goal[0])**2+(y-goal[1])**2)**0.5 > 0.4:  # replace with destination reached?
    # calculate collision cone below
    x = odm[0]
    y = odm[1]
    th = odm[2]
    # calculate v_x, v_y as per either TG, MV, or ST strategy
    [vel_x, vel_y] = vector_search(odm, Obs_data, goal)

    # convert velocity vector to linear and angular velocties using velocity_convert function given above
    [v_lin, v_ang] = velocity_convert(x, y, th % (2*pi), vel_x, vel_y)
    # publish the velocities below

    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = v_ang
    publish_vel.publish(vel_msg)

    # store robot path with time stamps (data available in odom topic)
    output.write("\n %s," % x)
    output.write("%s" % y)

    r.sleep()

vel_msg.linear.x = 0
vel_msg.angular.z = 0
publish_vel.publish(vel_msg)
output.close()
