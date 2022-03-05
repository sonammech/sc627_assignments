#!/usr/bin/env python

from math import atan2, pi, cos, sin


def PointToPointDist(p1, p2):
    Distance = round((((p1[1]-p2[1])**2)+((p1[0]-p2[0])**2))**0.5, 4)
    return Distance


def LineThroughTwoPoints(p1, p2):
    x1 = p1[0]
    x2 = p2[0]
    y1 = p1[1]
    y2 = p2[1]
    a = y2-y1
    b = x1-x2
    c = -(a*x1+b*y1)
    s = (a**2+b**2)**0.5
    return a/s, b/s, c/s


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


def PointToPolygonDist(q, P):
    l = len(P)
    d = []
    d.append(PointToSegmentDist(q, P[l-1], P[0]))
    for i in range(l-1):
        d.append(PointToSegmentDist(q, P[i], P[i+1]))
    return (min(d))


def TangentVectorToPolygon(q, P):
    [dis, w, T] = PointToPolygonDist(q, P)
    theta = atan2(q[0]-T[0], T[1]-q[1])
    Vx = round(cos(theta), 4)
    Vy = round(sin(theta), 4)
    if theta < 0:
        theta += 2*pi
    return Vx, Vy, theta


def VectorToPoint(q, T):
    theta = atan2(T[1]-q[1], T[0]-q[0])
    Vx = round(cos(theta), 4)
    Vy = round(sin(theta), 4)
    if theta < 0:
        theta += 2*pi
    return Vx, Vy, theta


def attPotential(q, g):
    D = PointToPointDist(q, g)
    if D > 2:
        Vx = 1.6*(g[0]-q[0])/D
        Vy = 1.6*(g[1]-q[1])/D
    else:
        Vx = 0.8*(g[0]-q[0])
        Vy = 0.8*(g[1]-q[1])
    return Vx, Vy


def repPotential(q, P):
    [D, w, [a, b]] = PointToPolygonDist(q, P)
    if D > 2:
        Vx = 0
        Vy = 0
    else:
        Vx = 0.8*(0.5-(1/D))*(a-q[0])/(D**3)
        Vy = 0.8*(0.5-(1/D))*(b-q[1])/(D**3)
    return Vx, Vy


def normalise(q):
    s = (q[0]**2)+(q[1]**2)
    if s == 0:
        Vx = 0
        Vy = 0
    else:
        Vx = q[0]/(s**0.5)
        Vy = q[1]/(s**0.5)
    theta = atan2(Vy, Vx)
    return Vx, Vy, theta
