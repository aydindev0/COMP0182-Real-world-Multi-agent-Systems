#!/usr/bin/env python3

import rospy
import math
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

"""
goal_pose.py
------------
Simple ArUco-based point-to-point controller.

This script:
- Waits for the robot pose from /id100/aruco_single/pose
- Waits for the goal pose from /id502/aruco_single/pose
- Computes heading and distance errors
- Publishes /cmd_vel until the robot enters a 2cm tolerance region

"""

rospy.init_node('goal_pose')

# --- Goal reach check --------------------------------------------------------
def check_goal_reached(init_pose, goal_pose, bias):
    """Return True if init_pose is within (+/-)bias of goal_pose."""
    if(init_pose.pose.position.x > goal_pose.pose.position.x - bias and init_pose.pose.position.x < goal_pose.pose.position.x + bias\
        and init_pose.pose.position.y > goal_pose.pose.position.y - bias and init_pose.pose.position.y < goal_pose.pose.position.y + bias):
        return True
    else:
        return False

# --- Publishers & initial pose acquisition -----------------------------------
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Wait for robot pose
init_pose = rospy.wait_for_message('/id100/aruco_single/pose', PoseStamped)

# Wait for goal pose (ArUco ID 502)
goal_pose = rospy.wait_for_message('/id502/aruco_single/pose', PoseStamped)

# Build lists of goals and starting poses
goals = []
goals.append(goal_pose)

twist = Twist()
inits = []
inits.append(init_pose)
inits.append(goal_pose)

# --- Main control loop -------------------------------------------------------
for goal_pose in goals:
	for init_pose in inits:
		while not check_goal_reached(init_pose, goal_pose, 0.02):

			# Refresh robot pose
			init_pose = rospy.wait_for_message('/id100/aruco_single/pose', PoseStamped)

			# Extract yaw
			orientation_q = init_pose.pose.orientation
			orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
			(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
			Orientation = yaw

			# Position differences
			dx = goal_pose.pose.position.x - init_pose.pose.position.x
			dy = goal_pose.pose.position.y - init_pose.pose.position.y
			distance = math.dist([init_pose.pose.position.x, init_pose.pose.position.y],
			                     [goal_pose.pose.position.x, goal_pose.pose.position.y])
			goal_direct = math.atan2(dy, dx)

			# Debug prints
			print("init_pose", [init_pose.pose.position.x, init_pose.pose.position.y])
			print("goal_pose", [goal_pose.pose.position.x, goal_pose.pose.position.y])
			print("Orientation", Orientation)
			print("goal_direct", goal_direct)

			# Normalise angles to [0, 2π)
			if(Orientation < 0):
				Orientation = Orientation + 2 * math.pi
			if(goal_direct < 0):
				goal_direct = goal_direct + 2 * math.pi

			# Angle error
			theta = goal_direct - Orientation

			# Shortest-angle wrap
			if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
				theta = theta + 2 * math.pi
			elif theta > 0 and abs(theta - 2 * math.pi) < theta:
				theta = theta - 2 * math.pi

			print("theta:", theta)

			# Control
			k2 = 2
			linear = 0.5
			angular = k2 * theta

			twist.linear.x = linear * distance * math.cos(theta)
			twist.angular.z = -angular

			# Publish velocity
			cmd_pub.publish(twist)
