#!/usr/bin/env python

from math import atan2, cos, sin, pi
import math


def PointToPointDist(p1, p2):
    Distance = round((((p1[1]-p2[1])**2)+((p1[0]-p2[0])**2))**0.5, 4)
    return Distance


def VectorToPoint(q, G):
    theta = atan2(G[1]-q[1], G[0]-q[0])
    Vx = round(cos(theta), 4)
    Vy = round(sin(theta), 4)
    if theta < 0:
        theta += 2*pi
    return Vx, Vy, theta


def PointToPolygonDist(q, P):
    l = len(P)
    d = []
    d.append(PointToSegmentDist(q, P[l-1], P[0]))
    for i in range(l-1):
        d.append(PointToSegmentDist(q, P[i], P[i+1]))
    return (min(d))


def LineThroughTwoPoints(p1, p2):   # Let line be ax+by+c=0
    x1 = p1[0]
    x2 = p2[0]
    y1 = p1[1]
    y2 = p2[1]
    a = y2-y1
    b = x1-x2
    c = -(a*x1+b*y1)
    s = (a**2+b**2)**0.5  # Length of line ax+by+c=0
    return a/s, b/s, c/s  # Unit Length


def PointToLineDist(q, p1, p2):
    [a, b, c] = LineThroughTwoPoints(p1, p2)
    Distance = abs(a*q[0] + b*q[1] + c)
    return Distance


def PointToSegmentDist(q, p1, p2):
    [a, b, c] = LineThroughTwoPoints(p1, p2)
    x1 = round((b**2)*q[0]-a*b*q[1]-a*c, 4)
    if b == 0:
        y1 = q[1]
    else:
        y1 = -(1/b)*(a*x1+c)

    d1 = PointToPointDist([x1, y1], p1)  # perpendicular point to point 1
    d2 = PointToPointDist([x1, y1], p2)  # perpendicular point to point 2
    d3 = PointToPointDist(p1, p2)        # distance between p1 and p2

    if d1+d2 > d3+0.0001:
        if d1 > d2:
            w = 2
            dist = PointToPointDist(q, p2)
            T = p2
        else:
            w = 1
            dist = PointToPointDist(q, p1)
            T = p1
    else:
        w = 0
        dist = PointToLineDist(q, p1, p2)
        T = [x1, y1]
    return dist, w, T


def TangentVectorToPolygon(q, P):
    [dis, w, T] = PointToPolygonDist(q, P)
    theta = atan2(q[0]-T[0], T[1]-q[1])
    Vx = round(cos(theta), 4)
    Vy = round(sin(theta), 4)
    if theta < 0:
        theta += 2*pi
    return Vx, Vy, theta
