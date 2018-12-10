#!/usr/bin/env python

import os
import rospy
import numpy as np
import time
import Utils
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance

CONTROLLER_DONE_TOPIC = '/controller/target_reached'
LOCALIZATION_TOPIC = '/pf/viz/inferred_pose'
MAP_TOPIC = 'static_map'
START_TOPIC = '/initialpose'
TARGET_TOPIC = 'move_base_simple/goal'

class OrchestratorNode (object):

  def __init__(self):
    _, map_info = Utils.get_map(MAP_TOPIC)

    self.curr_target = None
    self.start_pose = None
    self.waypoint_index = 0
    self.waypoints = np.array([[2600, map_info.height - 660, 0],
                               [1880, map_info.height - 440, 0],
                               [1435, map_info.height - 545, 0],
                               [1250, map_info.height - 460, 0],
                               [540, map_info.height - 835, 0]],
                               dtype='float64')

    self.start = np.array([[2500, map_info.height - 640, 0]], dtype='float64')

    Utils.map_to_world(self.waypoints, map_info)
    Utils.map_to_world(self.start, map_info)
    self.start = self.start[0,:]
    self.start[2] = np.deg2rad(-15)

    for i in range(1,len(self.waypoints)):
      theta = np.arctan2(self.waypoints[i, 1] - self.waypoints[i-1, 1], self.waypoints[i, 0] - self.waypoints[i-1, 0])
      self.waypoints[i-1, 2] = theta

    self.waypoints[0,2] = np.deg2rad(-15)
    self.waypoints[1,2] = np.deg2rad(180)
    self.waypoints[2,2] = np.deg2rad(135)
    self.waypoints[3,2] = np.deg2rad(170)
    self.waypoints[4,2] = np.deg2rad(-135)

    self.pose_cb(rospy.wait_for_message(LOCALIZATION_TOPIC, PoseStamped))

    self.pose_sub = rospy.Subscriber(LOCALIZATION_TOPIC, PoseStamped, self.pose_cb)
    self.start_pub = rospy.Publisher(START_TOPIC, PoseWithCovarianceStamped, queue_size=10)
    self.target_pub = rospy.Publisher(TARGET_TOPIC, PoseStamped, queue_size=10)
    self.target_reached_sub = rospy.Subscriber(CONTROLLER_DONE_TOPIC, PoseStamped, self.complete_wp_cb)

    rospy.sleep(1)
    start_msg = PoseWithCovarianceStamped()
    start_msg.header.frame_id = '/map'
    start_msg.header.stamp = rospy.Time.now()
    start_msg.pose = PoseWithCovariance()
    start_msg.pose.pose = Pose()
    start_msg.pose.pose.position.x = self.start[0]
    start_msg.pose.pose.position.y = self.start[1]
    start_msg.pose.pose.orientation = Utils.angle_to_quaternion(self.start[2])
    self.start_pub.publish(start_msg)
    rospy.sleep(2)

    self.plan_next_wp()

  def complete_wp_cb (self, msg):
    print("Path complete!")
    self.waypoint_index += 1
    if self.done():
      print "Done!"
    else:
      print("Setting up new path...")
      self.plan_next_wp()

  def done (self):
    return (self.waypoints.shape[0] == self.waypoint_index)

  def plan_next_wp (self):
    if not self.done():
      self.curr_target = self.waypoints[self.waypoint_index]

      print "Heading towards:"
      print self.curr_target

      start_msg = PoseWithCovarianceStamped()
      start_msg.header.frame_id = '/map'
      start_msg.header.stamp = rospy.Time.now()
      start_msg.pose = PoseWithCovariance()
      start_msg.pose.pose = Pose()
      start_msg.pose.pose.position.x = self.start_pose[0]
      start_msg.pose.pose.position.y = self.start_pose[1]
      start_msg.pose.pose.orientation = self.start_pose[2]
      self.start_pub.publish(start_msg)

      target_msg = PoseStamped()
      target_msg.header.frame_id = '/map'
      target_msg.header.stamp = rospy.Time.now()
      target_msg.pose = Pose()
      target_msg.pose.position.x = self.curr_target[0]
      target_msg.pose.position.y = self.curr_target[1]
      target_msg.pose.orientation = Utils.angle_to_quaternion(self.curr_target[2])
      self.target_pub.publish(target_msg)

  def pose_cb(self, msg):
    self.start_pose = np.array([msg.pose.position.x,
                                msg.pose.position.y,
                                msg.pose.orientation])

if __name__ == '__main__':
  rospy.init_node('orchestrator_node', anonymous=True)
  on = OrchestratorNode()
  rospy.spin()
