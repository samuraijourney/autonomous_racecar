#!/usr/bin/env python

import rospy
import rosbag
import sys
import time
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped

#PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
#BAG_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/teleop'
BAG_TOPIC = '/sim_car_pose/pose'
PUB_TOPIC = '/sim_car_pose/pose'
PUB_RATE_SLEEP = 1.0 / 30.0 # The rate at which messages should be published in seconds

# Loads a bag file, reads the msgs from the specified topic, and republishes them
def follow_bag(bag_path, follow_backwards=False):
    bag = rosbag.Bag(bag_path)
    pub = rospy.Publisher(PUB_TOPIC, PoseStamped, queue_size=1)
    
    if follow_backwards is True:
	msgs = []
        for topic, msg, t in bag.read_messages(topics=[BAG_TOPIC]):
	    msgs.append(msg)

	for msg in reversed(msgs):
            pub.publish(msg)
            time.sleep(PUB_RATE_SLEEP)

    else:
        for topic, msg, t in bag.read_messages(topics=[BAG_TOPIC]):
            pub.publish(msg)
            time.sleep(PUB_RATE_SLEEP)

    bag.close()

if __name__ == '__main__':
    bag_path = None # The file path to the bag file
    follow_backwards = False # Whether or not the path should be followed backwards

    rospy.init_node('bag_follower', anonymous=True)
	
    if (len(sys.argv) - 1) < 2:
        rospy.loginfo("Insufficient argument count. Ros bag path and follow backwards parameters are needed. Exiting...")
        sys.exit()
    
    bag_path = sys.argv[1]
    follow_backwards = sys.argv[2].lower() == 'true'
    follow_bag(bag_path, follow_backwards)
