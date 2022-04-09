#!/usr/bin/env python

import rospy
from sc627_helper.msg import ObsData
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import time
import matplotlib.pyplot as plt

ANG_MAX = math.pi/18
VEL_MAX = 0.15


class Equidistant_Robots:

    def __init__(self):
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.r = rospy.Rate(30)
        self.left_odom = Odometry()
        self.right_odom = Odometry()
        self.odom = Odometry()

    def velocity_convert(self, x, y, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        gain_ang = 0.5  # modify if necessary

        ang = math.atan2(vel_y, vel_x)
        ang += 2 * math.pi

        ang_err = ang - theta
        if ang_err < -pi:
            ang_err += 2 * math.pi
        elif ang_err > pi:
            ang_err -= 2*pi

        ang_err = self.calc_ang_err(ang_err)

        v_lin = min(max(math.cos(ang_err) * math.sqrt(vel_x **
                    2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
        v_ang = gain_ang * math.sin(ang_err)
        return v_lin, v_ang

    def calc_ang_err(self, ang_err):
        if ang_err > pi/2:
            ang_err = max(ang_err, pi-ANG_MAX)
        elif ang_err < -pi/2:
            ang_err = min(ang_err, -pi+ANG_MAX)
        else:
            ang_err = min(max(ang_err, -ANG_MAX), ANG_MAX)
        return ang_err

    def callback_odom(self, data):
        '''
        Get robot data
        '''
        self.odom = data

    def callback_left_odom(self, data):
        '''
        Get left robot data
        '''

        self.left_odom = data

    def callback_right_odom(self, data):
        '''
        Get right robot data
        '''
        # print(data)
        self.right_odom = data


if __name__ == '__main__':
    rospy.init_node('assign4_skeleton', anonymous=True)
    q = Equidistant_Robots()
    rospy.Subscriber('/odom', Odometry, q.callback_odom)
    rospy.Subscriber('/left_odom', Odometry,
                     q.callback_left_odom)
    rospy.Subscriber('/right_odom', Odometry,
                     q.callback_right_odom)

name = str(rospy.get_namespace())
if name == "/bot_2/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/output_2.txt", "a")
elif name == "/bot_3/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/output_3.txt", "a")
elif name == "/bot_4/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/output_4.txt", "a")
elif name == "/bot_5/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/output_5.txt", "a")
elif name == "/bot_6/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/output_6.txt", "a")
elif name == "/bot_7/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/output_7.txt", "a")


i = 1
k = 1
vel_x = 1
vel_y = 1
left_v = 1
right_v = 1


while i < 200 or abs(vel_x) > 0.01 or abs(left_v) > 0.01 or abs(right_v) > 0.01:

    x = q.odom.pose.pose.position.x
    y = q.odom.pose.pose.position.y
    ori = q.odom.pose.pose.orientation
    (roll, pitch, th) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    right_x = q.right_odom.pose.pose.position.x
    right_y = q.right_odom.pose.pose.position.y
    right_v = q.right_odom.twist.twist.linear.x
    left_x = q.left_odom.pose.pose.position.x
    left_y = q.left_odom.pose.pose.position.y
    left_v = q.left_odom.twist.twist.linear.x

    # calculate vel_x, vel_y
    vel_x = k*((right_x-x)+(left_x-x))
    vel_y = k*((right_y-y)+(left_y-y))

    # converts velocities
    [vel_lin, vel_ang] = q.velocity_convert(x, y, th % (2*pi), vel_x, vel_y)

    # publish the velocities
    vel_msg = Twist()
    vel_msg.linear.x = vel_lin
    vel_msg.angular.z = vel_ang
    q.pub_vel.publish(vel_msg)

    # store robot path with time stamps (create file data)
    out.write(str(i))
    out.write("\n")
    out.write(str(x))
    out.write("\n")
    out.write(str(y))
    out.write("\n")

    i = i+1
    q.r.sleep()

vel_msg.linear.x = 0
vel_msg.angular.z = 0
q.pub_vel.publish(vel_msg)
out.close()
