#!/usr/bin/env python

import os
import rospy
import numpy as np
import PRM
import Utils
import Astar
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker


'''
Modify input map file to accomodate for dimensions of robot by padding obstacles by provided radius
In:
    map: Occupancy grid for map to modify
    radius: Radius in pixels to pad obstacles by
Out:
    The modified Occupancy grid
'''
def fatten_map (map, radius):
  new_map = OccupancyGrid ()
  new_map.info = map.info
  new_map.header = map.header

  data = np.asarray (map.data)

  waypoints = [[1910, map.info.height - 340], [1500, map.info.height - 210],
    [1520, map.info.height - 435], [1130, map.info.height - 400], [670, map.info.height - 840]]

  for waypoint in waypoints:
    for x in range (waypoint[0] - radius, waypoint[0] + radius):
      for y in range (waypoint[1] - radius, waypoint[1] + radius):
        data[x + y * map.info.width] = 100

  for y in range (radius, map.info.height - radius):
    for x in range (radius, map.info.width - radius):
      if map.data[x + y * map.info.width] == 0:
        if (map.data[(x - 1) + y * map.info.width] > 0 and map.data[(x - 1) + y * map.info.width] != 92):
          for x2 in range (x, radius):
            data[x2 + y * map.info.width] = 92

        if (map.data[(x + 1) + y * map.info.width] > 0 and map.data[(x + 1) + y * map.info.width] != 92):
          for x2 in range (x - radius, x):
            data[x2 + y * map.info.width] = 92

        if (map.data[x + (y - 1) * map.info.width] > 0 and map.data[x + (y - 1) * map.info.width] != 92):
          for y2 in range (y, radius):
            data[x + y2 * map.info.width] = 92

        if (map.data[x + (y + 1) * map.info.width] > 0 and map.data[x + (y + 1) * map.info.width] != 92):
          for y2 in range (y - radius, y):
            data[x + y2 * map.info.width] = 92
      else:
        data[x + y * map.info.width] = 100

  new_map.data = data

  return new_map

class PlannerNode(object):

  def __init__(self):
    self.map_pub = rospy.Publisher ('/fat_map', OccupancyGrid, queue_size=10)
    self.plan_pub = rospy.Publisher ('/plan', PoseArray, queue_size=10)
    self.prm_pub = rospy.Publisher ('/prm', Marker, queue_size=10)

    map_service_name = rospy.get_param ("~static_map", "static_map")
    rospy.wait_for_service (map_service_name)

    self.map_msg = rospy.ServiceProxy (map_service_name, GetMap) ().map

    print("Getting map from service: " + map_service_name)

    self.map = fatten_map (self.map_msg, 30)
    self.map_pub.publish (self.map)

    print("Done beefening map")

    self.prm = PRM.PRM (self.map, 150, 600)

    print("Done generating PRM")

    # points = []

    # i = 1

    # marker = Marker ()
    # marker.action = marker.DELETEALL

    # self.prm_pub.publish (marker)

    # for edge in self.prm.graph.edges (data = "weight"):
    #   p1 = list (float (n) for n in edge[0])
    #   p2 = list (float (n) for n in edge[1])

    #   p1[1] = self.info.height - p1[1]
    #   p2[1] = self.map.info.height - p2[1]

    #   p1 = Utils.map_to_world (p1 + [0], self.map.info)
    #   p2 = Utils.map_to_world (p2 + [0], self.map.info)

    #   pf = Point ()
    #   pf.x = p1[0]
    #   pf.y = p1[1]
    #   pf.z = 0
    #   points.append (pf)

    #   pe = Point ()
    #   pe.x = p2[0]
    #   pe.y = p2[1]
    #   pe.z = 0
    #   points.append (pe)

    #   marker.id = i
    #   marker.header.frame_id = "/map"
    #   marker.type = marker.LINE_STRIP
    #   marker.action = marker.ADD
    #   marker.pose.orientation.w = 1
    #   marker.points = [pf, pe]
    #   marker.lifetime = rospy.Duration ()
    #   marker.scale.x = 0.03
    #   marker.scale.y = 0.03
    #   marker.scale.z = 0.03
    #   marker.color.a = 1.0
    #   marker.color.g = 1.0
    #   marker.color.b = 0.0

    #   i += 1

    #   self.prm_pub.publish (marker)

    # marker.id = 199991
    # marker.header.frame_id = "/map"
    # marker.type = marker.POINTS
    # marker.action = marker.ADD
    # marker.pose.orientation.w = 1
    # marker.points = points
    # marker.lifetime = rospy.Duration ()
    # marker.scale.x = 0.4
    # marker.scale.y = 0.4
    # marker.scale.z = 0.4
    # marker.color.a = 1.0
    # marker.color.r = 1.0
    # marker.color.g = 0.0

    # self.prm_pub.publish (marker)

    print 'Ready to plan'

    self.sub = rospy.Subscriber ('/start_plan', PoseArray, self.plan_cb)

  def plan_cb (self, req):
    print("Planner node cb")
    if len (req.poses) != 2:
      return

    source = np.array ([[req.poses[0].position.x, req.poses[0].position.y, req.poses[0].position.z]], dtype='float64')
    target = np.array ([req.poses[1].position.x, req.poses[1].position.y], dtype="int").reshape (2)

    Utils.world_to_map (source, self.map.info)
    source = np.array([source[0,0], source[0,1]], dtype='int64')

    plan = Astar.generate_plan (self.prm, source, target)

    if plan:
      print "Plan found"

      plan_arr = PoseArray ()

      plan_arr.header.frame_id = '/map'
      plan_arr.header.stamp = rospy.Time.now()

      for node in plan:
        node = np.array ([[node[0], node[1], 0]], dtype='float64')
        Utils.map_to_world (node, self.map.info)
        node = np.array([node[0,0], node[0,1]], dtype='float64')

        pose = Pose ()
        pose.position.x = node[0]
        pose.position.y = node[1]
        pose.position.z = 0

        if len (plan_arr.poses) > 1:
          theta = np.arctan2 (pose.position.y - plan_arr.poses[-1].position.y, pose.position.x - plan_arr.poses[-1].position.x)
          plan_arr.poses[-1].orientation = Utils.angle_to_quaternion (theta)

        plan_arr.poses.append (pose)

      self.plan_pub.publish (plan_arr)

if __name__ == '__main__':
  rospy.init_node ('planner_node', anonymous=True)

  pn = PlannerNode ()

  rospy.spin ()
