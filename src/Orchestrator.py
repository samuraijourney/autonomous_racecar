#!/usr/bin/env python

import os
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseStamped

PLAN_REQ_TOPIC = '/start_plan'
LOCALIZATION_TOPIC = '/pf/viz/inferred_pose'
CONTROLLER_DONE_TOPIC = '/controller/target_reached'

class OrchestratorNode (object):

  def __init__(self):
    self.curr_target = None
    self.waypoints = [[2600, 660], [1880, 440], [1435, 545], [1250, 460], [540, 835]]
    self.plan_req_pub = rospy.Publisher (PLAN_REQ_TOPIC, PoseArray, queue_size=10)

  def plan_next_wp (self, curr_pose):
    if self.waypoints:
      self.curr_target = self.waypoints[0]

      print "Heading towards:"
      print self.curr_target

      req_arr = PoseArray ()

      req_arr.header.frame_id = '/map'
      req_arr.header.stamp = rospy.Time.now()

      target_pose = Pose ()
      target_pose.position.x = self.curr_target[0]
      target_pose.position.y = self.curr_target[1]

      req_arr.poses.append (curr_pose)
      req_arr.poses.append (target_pose)

      self.plan_req_pub.publish (req_arr)

  def done (self):
    return (len (self.waypoints) == 0)

  def complete_wp (self, curr_pose):
    self.waypoints.pop (0)

if __name__ == '__main__':
  rospy.init_node ('orchestrator_node', anonymous=True)

  on = OrchestratorNode ()

  while not on.done ():
    start_pose = rospy.wait_for_message (LOCALIZATION_TOPIC, PoseStamped)
    on.plan_next_wp (start_pose.pose)
    finish_pose = rospy.wait_for_message (CONTROLLER_DONE_TOPIC, PoseStamped)
    on.complete_wp (finish_pose.pose)

  print "Done!"

  rospy.spin ()
