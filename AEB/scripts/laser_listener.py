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

class LaserListener():

  def __init__(self):
    # Start listening to the laser scan
    rospy.init_node('laser_listener',anonymous = True)
    self.laser_subscriber = rospy.Subscriber('/scan',LaserScan,self.callback)   

    self.laserscan = LaserScan()
    
    rospy.spin()
    
    
  def callback(self, data):
    # Listen to the laser scan and print the minimum range
    self.laserscan = data
    self.angle_min = data.angle_min * 180 / np.pi
    self.angle_max = data.angle_max * 180 / np.pi
    self.angle_increment = data.angle_increment * 180 / np.pi

    self.all_angles = np.arange(self.angle_min, self.angle_max, self.angle_increment)
    
    self.ranges = np.array(self.laserscan.ranges)

    self.frontal_scan = self.ranges[ (-5 < self.all_angles) & (self.all_angles < 5)]
      
    self.ave_frontal_distance = self.frontal_scan[np.isfinite(self.frontal_scan)]
    
    rospy.loginfo('The average frontal range is currently %s', str(self.ave_frontal_distance))
    
    
if __name__ == '__main__':
  print("Listening to laser scanner")
  x = LaserListener()
