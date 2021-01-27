#!/usr/bin/env python
import roslib
roslib.load_manifest('lab2')
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import numpy as np
import random
from numpy import ones, vstack
from numpy.linalg import lstsq
import math


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y


def distance(m, c, p, q):
    d = abs((m * p) + (-1 * q) + c)
    d = d / math.sqrt(m * m + 1)
    return d


def proj(ranges):
    x = []
    y = []
    angle = -1.57079
    for i in range(len(ranges)):
        r = ranges[i]
        theta = angle
        angle = angle + 0.0087266
        x2, y2 = pol2cart(r, theta)
        # for any points that has the distance of lower than 1.5
        if r < 2.7:
            x.append(x2)
            y.append(y2)
    return np.array(x), np.array(y)


def best_points(x, y):
    n = len(x)
    m0, m1, m2 = 0,0,0
    for i in range(0, n - 1):
        for j in range(0, n - 1):
            dist = math.hypot(x[i] - x[j], y[i] - y[j])
            if dist > m0:
                m0, m1, m2 = dist, i, j
    return [[x[m1], y[m1]], [x[m2], y[m2]]]


def ransac(x, y):
    n = len(x)
    max0 = 0
    iner_x = []
    iner_y = []
    saved_index = []
    for j in range(40):  # 20 iterations
        r1 = random.randint(0, n - 1)
        r2 = random.randint(0, n - 1)
        if r1 != r2:
            points = [(x[r1], y[r1]), (x[r2], y[r2])]
            x_coords, y_coords = zip(*points)
            A = vstack([x_coords, ones(len(x_coords))]).T
            a, b = lstsq(A, y_coords)[0]
            # Line Solution: y = ax + b
            ilter = 0
            x_temp = []
            y_temp = []
            for i in range(0, n - 1):
                dist = distance(a, b, x[i], y[i])
                if dist < 0.3:
                    ilter = ilter + 1
                    x_temp.append(x[i])
                    y_temp.append(y[i])
                    saved_index.append(i)
            if ilter > max0:
                max0 = ilter
                iner_x = x_temp
                iner_y = y_temp
    if r1 != r2 and len(iner_x) > 20:
        iner_pts = best_points(iner_x, iner_y)
        pt = lambda c: Point(c[0], c[1], 0)
        points = [pt(i) for i in iner_pts]
        if len(points) == 2:
            return iner_pts, points
    return 0, []


def callback(data):
    # moving
    laser_scan = np.array(data.ranges)
    length = len(laser_scan)
    left, right = laser_scan[0:int(length/2)], laser_scan[int(length/2):]

    # set direction
    if len(left[left<0.5]) > len(right[right<0.5]):
        direction = 1
    else:
        direction = -1

    # stop if any point is less than 0.9
    laser_scan = list(laser_scan)
    laser_scan.sort()
    if laser_scan[0] < 0.9:
        move.linear.x = 0
        move.angular.z = direction*1.3
    else:
        move.linear.x = 10
        move.angular.z = 0.0
    pub.publish(move)

    # visualization_marker
    pub2 = rospy.Publisher('/Marker', Marker, queue_size=100)
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lines"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.color.g = 1.0
    marker.color.a = 1.0
    points = []
    x, y = proj(data.ranges)
    if len(x) != 0:
        remove_inex = []
        lim = 0
        while remove_inex != 0 and lim < 2:
            lim += 1
            x, y = np.delete(x, remove_inex), np.delete(y, remove_inex)
            remove_inex, target = ransac(x, y)
            if len(target) == 2:
                for i in target:
                    points.append(i)

    marker.id = 1
    marker.points = points
    pub2.publish(marker)


rospy.init_node('perception', anonymous=True)
rate = rospy.Rate(10)
sub = rospy.Subscriber('/base_scan', LaserScan, callback)
pub = rospy.Publisher('cmd_vel', Twist)
move = Twist()

rospy.spin()
