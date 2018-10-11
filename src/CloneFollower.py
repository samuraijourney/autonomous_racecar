#!/usr/bin/env python

import argparse
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Point
import Utils # <----------- LOOK AT THESE FUNCTIONS ***************************

SUB_TOPIC = '/sim_car_pose/pose' # The topic that provides the simulated car pose
PUB_TOPIC = '/clone_follower_pose/pose' # The topic that you should publish to
MAP_TOPIC = 'static_map' # The service topic that will provide the map

# Follows the simulated robot around
class CloneFollower:

    '''
    Initializes a CloneFollower object
    In:
        follow_offset: The required x offset between the robot and its clone follower
        force_in_bounds: Whether the clone should toggle between following in front
                        and behind when it goes out of bounds of the map
    '''
    def __init__(self, follow_offset, force_in_bounds):
        self.follow_offset = follow_offset # Store the input params in self
        self.force_in_bounds = force_in_bounds # Store the input params in self
        self.map_img, self.map_info = get_map(MAP_TOPIC) # Get and store the map for bounds checking
        
        # Setup publisher that publishes to PUB_TOPIC
        self.pub = rospy.Publisher(PUB_TOPIC, PoseStamped, queue_size=1)
        
        # Setup subscriber that subscribes to SUB_TOPIC and uses the self.update_pose
        # callback
        self.sub = rospy.Subsriber(SUB_TOPIC, PoseStamped, self.update_pose)
        
    '''
    Given the translation and rotation between the robot and map, computes the pose
    of the clone
    (This function is optional)
    In:
        trans: The translation between the robot and map
        rot: The rotation between the robot and map
    Out:
        The pose of the clone
    '''
    def compute_follow_pose(self, trans, rot):
        offset_vector = [self.follow_offset, 0]
        rot_matrix = rotation_matrix(rot)
        offset_vector = np.matmul(rot_matrix, offset_vector)
        final_position = trans + offset_vector
        return Pose(Point(final_position[0], final_position[1], 0), rot)
        
    '''
    Callback that runs each time a sim pose is received. Should publish an updated
    pose of the clone.
    In:
        msg: The pose of the simulated car. Should be a geometry_msgs/PoseStamped
    '''  
    def update_pose(self, msg):
        # Compute the pose of the clone
        # Note: To convert from a message quaternion to corresponding rotation matrix,
        #       look at the functions in Utils.py
        clone_pose = self.compute_follow_pose(msg.pose.position, -quaternion_to_angle(msg.pose.orientation))

        # Check bounds if required
        if self.force_in_bounds:
            pixel_location = world_to_map(clone_pose, self.map_info)
            if (self.map_img[pixel_location[1], pixel_location[0]] == 0):
                clone_pose = self.compute_follow_pose(msg.pose.position, quaternion_to_angle(msg.pose.orientation))
        
        # Setup the out going PoseStamped message
        clone_poseStamped = PoseStamped(msg.header, clone_pose)

        # Publish the clone's pose
        self.pub.publish(clone_poseStamped)
    
if __name__ == '__main__':
    follow_offset = 1.0 # The offset between the robot and clone
    force_in_bounds = False # Whether or not map bounds should be enforced

    rospy.init_node('clone_follower', anonymous=True) # Initialize the node

    parser = argparse.ArgumentParser(description='Process follow parameters')
    parser.add_argument('-o', '--follow_offset', help='The follow offset' type=float, dest='offset')
    parser.add_argument('-f', '--force_in_bounds', help='Force in bounds', type=bool, dest='force')
    args = parser.parse_args()

    # Populate params with values passed by launch file
    follow_offset = args.offset
    force_in_bounds = args.force

    cf = CloneFollower(follow_offset, force_in_bounds) # Create a clone follower
    rospy.spin() # Spin