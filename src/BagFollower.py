#!/usr/bin/env python

import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped

BAG_TOPIC = # Name of the topic that should be extracted from the bag
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
PUB_RATE = # The rate at which messages should be published

# Loads a bag file, reads the msgs from the specified topic, and republishes them
def follow_bag(bag_path, follow_backwards=False):
	pass

if __name__ == '__main__':
	bag_path = None # The file path to the bag file
	follow_backwards = False # Whether or not the path should be followed backwards
	
	rospy.init_node('bag_follower', anonymous=True)
	
	# Populate param(s) with value(s) passed by launch file
	
	follow_bag(bag_path, follow_backwards)
