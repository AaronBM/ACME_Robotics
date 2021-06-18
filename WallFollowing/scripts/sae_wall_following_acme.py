#!/usr/bin/env python3

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import pdb

# Vehicle parameters
ANGLE_RANGE = 270 				# Hokuyo 10LX has 270 degrees scan
DISTANCE_RIGHT_THRESHOLD = 0.5 	# (m)
VELOCITY = 0.1 					# meters per second

# Controller parameters
kp = 0.7
kd = 

# Other global variables
error = 0.0
prev_error = 0.0

def control(error):
	global kp
	global kd
	global VELOCITY

	# TO-DO: Implement controller
	# ---
  # Aaron will code this.  See the return values of follow_center()
	# ---

	# Set maximum thresholds for steering angles
	if steering_angle > 0.5:
		steering_angle = 0.5
	elif steering_angle < -0.5:
		steering_angle = -0.5

	print "Steering Angle is = %f" % steering_angle 

	# TO-DO: Publish the message

def steering_cmd(centerline_error, steering_angle )

	Kpgain =1.0
	Kigain =1.0

##  These two statements work the same.  I'm unsure on the signing of the centerline_error.
##  Incorporating a Proportional gain is straightforwad. Kp * the error.
##  Incorporating a Integral gain requires tracking error over the past X timesteps.  I'm not sure how to do that cleanly here.
	if centerline_error > 0:
		STEERING_ANGLE = centerline_error * Kpgain #+
	if centerline_error < 0:
		sSTEERING_ANGLE = centerline_error * Kpgain #+


    msg.drive.steering_angle = STEERING_ANGLE
    pub.publish(msg)



	# ---
  # Aaron will code this.  Note that he will need to implement a publisher down at the bottom (we think?)
	# ---

def get_index(angle, data):
	# 	# TO-DO: For a given angle, return the corresponding index for the data.ranges array
	# ---
  # Rewrite our original get_index function so that it returns a single number for the index.  We can then use this number to extract further angle and mid-angle points.
  # Erika is going to adapt that code.
  # This should return a single value index (an int value)
  
  return index
	# ---

def distance(angle_right, angle_lookahead, data):
	global ANGLE_RANGE
	global DISTANCE_RIGHT_THRESHOLD

	# TO-DO: Find index of the two rays, and calculate a, b, alpha and theta. Find the actual distance from the right wall.
	# ---
  # CJ will code this
	# ---

	print "Distance from right wall : %f" % distance_r

	# Calculate error
	error = 

	return error, distance


def follow_center(angle_right,angle_lookahead_right, data):

	angle_lookahead_left = 180 + angle_right
	angle_left = 180 - angle_lookahead_right 

	er, dr = distance(angle_right, angle_lookahead_right, data)
	el, dl = distance(angle_left, angle_lookahead_left, data)

	# Find Centerline error
	# ---
  # CJ will code this
	# ---

	print "Centerline error = %f " % centerline_error

	return centerline_error

def callback(data):

	# Pick two rays at two angles
	angle_right = 90
	angle_lookahead = 60

	# To follow right wall
	#er, dr = distance(angle_right,angle_lookahead, data)

	# To follow the centerline
	ec = follow_center(angle_right,angle_lookahead, data)

	control(ec)

if __name__ == '__main__':
	print("Wall following started")
	rospy.init_node('wall_following',anonymous = True)

	# TO-DO: Implement the publishers and subscribers
	# ---
	# This code needs to be checked to make sure its right.  CJ will take a look at completing this.  What is AckemannDriveStamped?
  sub = rospy.Subscriber("/scan", LaserScan, callback) # This shoud be correct?  Check!
	# ---

	rospy.spin()
