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
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degree scan.
DISTANCE_THRESHOLD = 3  # Distance threshold before collision (m)
VELOCITY = 0.5  # Maximum Velocity of the vehicle
TIME_THRESHOLD = 1  # Time threshold before collision (s)
STEERING_ANGLE = 0  # Steering angle is uncontrolled

# P-Controller Parameters
kp_dist = 1.0
kp_ttc = 1.0

dist_error = 0.0
time_error = 0.0

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)


def dist_control(distance):
    global kp_dist
    global VELOCITY
    global DISTANCE_THRESHOLD
    global STEERING_ANGLE

    # TO-DO: Calculate Distance to Collision Error
    # ---
    error = distance - DISTANCE_THRESHOLD
    if VELOCITY < error * kp_dist:
        velocity = VELOCITY
    elif (0 < error * kp_dist) and (error * kp_dist) < VELOCITY:
        velocity = error * kp_dist
    else:
        velocity = 0
    # ---

    print("Distance before collision is = ", distance)
    print("Vehicle velocity= ", velocity)

    msg = AckermannDriveStamped()
    msg.drive.speed = velocity
    msg.drive.steering_angle = STEERING_ANGLE
    pub.publish(msg)


def TTC_control(distance):
    global kp_ttc
    global TIME_THRESHOLD
    global VELOCITY
    global STEERING_ANGLE

    # TO-DO: Calculate Time To Collision Error
    # ---
    # We didn't do this mode yet
    # ---

    print("Time to collision in seconds is = ", time)
    print("Vehicle velocity = ", velocity)

    msg = AckermannDriveStamped()
    msg.drive.speed = velocity
    msg.drive.steering_angle = STEERING_ANGLE
    pub.publish(msg)


def get_index(angle, data):
    # TO-DO: For a given angle, return the corresponding index for the data.ranges array
    # ---
    angle_limit = 2
    angle_inc = data.angle_increment * 180 / np.pi
    angle_min = data.angle_min * 180 / np.pi
    angle_max = data.angle_max * 180 / np.pi

    all_angles = np.arange(angle_min, angle_max, angle_inc)

    index = (angle - angle_limit < all_angles) & (all_angles < angle + angle_limit)

    return index



# Use this function to find the average distance of a range of points directly in front of the vehicle.
def get_distance(data):
    global ANGLE_RANGE

    angle_front = 0  # Range of angle in the front of the vehicle we want to observe
    # Get the corresponding list of indices for given range of angles
    index = get_index(angle_front, data)

    # TO-DO: Find the avg range distance
    # ---
    ranges = np.array(data.ranges)
    front_scan = ranges[index]  				# Pull angles from front cone

    print(front_scan[0])

    clean_front_scan = front_scan[np.isfinite(front_scan)]  # Remove infinite values

    avg_dist = np.mean(clean_front_scan)
    # ---

    print("Average Distance = ", avg_dist)

    return avg_dist


def callback(data):
    # TO-DO: Complete the Callback. Get the distance and input it into the controller
    curr_distance = get_distance(data)

    dist_control(curr_distance)


# ---

if __name__ == '__main__':
    print("AEB started")
    rospy.init_node('aeb', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()
