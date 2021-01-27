#!/usr/bin/env python
import rospy
import numpy as np
import roslib
roslib.load_manifest('lab2')
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math


def get_goalseek_angluar(goal_angle, face_wall, toward_goal_line):
	if toward_goal_line:
		return min(goal_angle, 1)
	elif face_wall:
		return 0
	else:
		return min(goal_angle, 1)


def get_wallfollow_angluar(left_obstacle,face_wall):
	# if there is a obstacle on the left, no need of angular velocity
	if face_wall:
		return 0.5
	if left_obstacle:
		return 0.15
	else:
		return -0.4


def check_point_toward_goal_line(pts, pos):
	A = pts[0, :]
	B = pts[1, :]
	C = np.array([pos.x, pos.y])
	area = abs((A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0)
	threshold = 0.6
	if area < threshold:
		return True
	else:
		return False


def callback(data):
	global laser_scans
	global face_wall, left_obstacle
	laser_scan = np.array(data.ranges)
	# if there is any point in laser scan that the distance is smaller than 1 within angle 40 to 140
	# than the robot is facing the wall
	if min(laser_scan) < 0.6:
		face_wall = True
	else:
		face_wall = False

	# if there is over 10 points on the left are close to the robot
	# than the there is a obstacle on the left
	left_scan = laser_scan[0:80]
	count = len(left_scan[left_scan < 1])
	if count > 10:
		left_obstacle = True
	else:
		left_obstacle = False


def callbacktruth(data):
	global orientation
	global pos
	# get current location and detail
	pos = data.pose.pose.position
	orientation = data.pose.pose.orientation


def print_info(states,dist_to_goal,pos):
	print("Robot Status:  ", states)
	print("Distance From Goal: ", dist_to_goal)
	print("Location: ", pos, '\n')

rospy.init_node('bug2', anonymous=True)
rospy.Subscriber("/base_scan", LaserScan, callback)

# setting parameters
points = []
pos, orientation = 0, 0
face_wall, left_obstacle, toward_goal_line, arrive = False, False, False, False
pubcmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
sub_truth = rospy.Subscriber("/base_pose_ground_truth", Odometry, callbacktruth)
rate = rospy.Rate(10)
points = np.array([[-8, -2], [4.5, 9.0]])
dist_thre = 0.35
states = "GOAL_SEEK"

while not arrive:
	# print pos, orientation
	if orientation != 0:
		robot_angle = 2 * np.arcsin(orientation.z)
		# the distance between the current location and goal
		dist_to_goal = math.sqrt((points[1, 0] - pos.x) ** 2 + (points[1, 1] - pos.y) ** 2)
		goal_angle = math.atan((points[1, 1] - pos.y) / (points[1, 0] - pos.x)) - robot_angle
		toward_goal_line = check_point_toward_goal_line(points, pos)
		twist = Twist()
		if dist_to_goal < dist_thre:
			# stop when arrive at goal position
			twist.linear.x, twist.angular.z = 0, 0
			states = "Arrive!!!"
			print_info(states, round(dist_to_goal,4), [round(pos.x,4),round(pos.y,4)])
			break
		else:
			if face_wall:
				# stop when facing a wall
				vel = 0
			else:
				# go straight by following the line toward the goal
				vel = 0.6
			twist.linear.x = vel
			if states == "GOAL_SEEK":
				twist.angular.z = get_goalseek_angluar(goal_angle, face_wall, toward_goal_line)
				if face_wall:
					states = "WALL_FOLLOW"
			else:
				twist.angular.z = -1 * get_wallfollow_angluar(left_obstacle, face_wall)
				if toward_goal_line and not face_wall:
					states = "GOAL_SEEK"
			print_info(states, round(dist_to_goal,4), [round(pos.x,4),round(pos.y,4)])
		pubcmd_vel.publish(twist)
		rate.sleep()


