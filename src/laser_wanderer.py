#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

SCAN_TOPIC = '/scan' # The topic to subscribe to for laser scans
CMD_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0' # The topic to publish controls to
POSE_TOPIC = '/sim_car_pose/pose' # The topic to subscribe to for current pose of the car
                                  # NOTE THAT THIS IS ONLY NECESSARY FOR VIZUALIZATION
VIZ_TOPIC = '/laser_wanderer/rollouts' # The topic to publish to for vizualizing
                                       # the computed rollouts. Publish a PoseArray.

MAX_PENALTY = 10000 # The penalty to apply when a configuration in a rollout
                    # goes beyond the corresponding laser scan
                    

'''
Wanders around using minimum (steering angle) control effort while avoiding crashing
based off of laser scans. 
'''
class LaserWanderer:

  '''
  Initializes the LaserWanderer
    rollouts: An NxTx3 numpy array that contains N rolled out trajectories, each
              containing T poses. For each trajectory, the t-th element represents
              the [x,y,theta] pose of the car at time t+1
    deltas: An N dimensional array containing the possible steering angles. The n-th
            element of this array is the steering angle that would result in the 
            n-th trajectory in rollouts
    speed: The speed at which the car should travel
    compute_time: The amount of time (in seconds) we can spend computing the cost
    laser_offset: How much to shorten the laser measurements
  '''
  def __init__(self, rollouts, deltas, speed, compute_time, laser_offset):
    # Store the params for later
    self.rollouts = rollouts
    self.deltas = deltas
    self.speed = speed
    self.compute_time = compute_time
    self.laser_offset = laser_offset
    
    # YOUR CODE HERE
    self.cmd_pub = # Create a publisher for sending controls
    self.laser_sub = # Create a subscriber to laser scans that uses the self.wander_cb callback
    self.viz_pub = # Create a publisher for vizualizing trajectories. Will publish PoseArrays
    self.viz_sub = # Create a subscriber to the current position of the car
    # NOTE THAT THIS VIZUALIZATION WILL ONLY WORK IN SIMULATION. Why?
    
  '''
  Vizualize the rollouts. Transforms the rollouts to be in the frame of the world.
  Only display the last pose of each rollout to prevent lagginess
    msg: A PoseStamped representing the current pose of the car
  '''  
  def viz_sub(self, msg):
    # Create the PoseArray to publish. Will contain N poses, where the n-th pose
    # represents the last pose in the n-th trajectory
    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()
    

    # Transform the last pose of each trajectory to be w.r.t the world and insert into
    # the pose array
    # YOUR CODE HERE

    self.viz_pub.publish(pa)
    
  '''
  Compute the cost of one step in the trajectory. It should penalize the magnitude
  of the steering angle. It should also heavily penalize crashing into an object
  (as determined by the laser scans)
    delta: The steering angle that corresponds to this trajectory
    rollout_pose: The pose in the trajectory 
    laser_msg: The most recent laser scan
  '''  
  def compute_cost(self, delta, rollout_pose, laser_msg):
  
    # Initialize the cost to be the magnitude of delta
    # Consider the line that goes from the robot to the rollout pose
    # Compute the angle of this line with respect to the robot's x axis
    # Find the laser ray that corresponds to this angle
    # Add MAX_PENALTY to the cost if the distance from the robot to the rollout_pose 
    # is greater than the laser ray measurement - np.abs(self.laser_offset)
    # Return the resulting cost
    # Things to think about:
    #   What if the angle of the pose is less (or greater) than the angle of the
    #   minimum (or maximum) laser scan angle
    #   What if the corresponding laser measurement is NAN?
    # NOTE THAT NO COORDINATE TRANSFORMS ARE NECESSARY INSIDE OF THIS FUNCTION
    
    # YOUR CODE HERE
    pass
    
  '''
  Controls the steering angle in response to the received laser scan. Uses approximately
  self.compute_time amount of time to compute the control
    msg: A LaserScan
  '''
  def wander_cb(self, msg):
    start = rospy.Time.now().to_sec() # Get the time at which this function started
    
    # A N dimensional matrix that should be populated with the costs of each
    # trajectory up to time t <= T
    delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float) 
    traj_depth = 0
    
    # Evaluate the cost of each trajectory. Each iteration of the loop should calculate
    # the cost of each trajectory at time t = traj_depth and add those costs to delta_costs
    # as appropriate
    
    # Pseudo code
    # while(you haven't run out of time AND traj_depth < T):
    #   for each trajectory n:
    #       delta_costs[n] += cost of the t=traj_depth step of trajectory n
    #   traj_depth += 1 
    # YOUR CODE HERE


    # Find the delta that has the smallest cost and execute it by publishing
    # YOUR CODE HERE
    
'''
Apply the kinematic model to the passed pose and control
  pose: The current state of the robot [x, y, theta]
  control: The controls to be applied [v, delta, dt]
  car_length: The length of the car
Returns the resulting pose of the robot
'''
def kinematic_model_step(pose, control, car_length):
  # Apply the kinematic model
  # Make sure your resulting theta is between 0 and 2*pi
  # Consider the case where delta == 0.0
  
  # YOUR CODE HERE
  pass
    
'''
Repeatedly apply the kinematic model to produce a trajectory for the car
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''
def generate_rollout(init_pose, controls, car_length):
  # YOUR CODE HERE
  pass
   
'''
Helper function to generate a number of kinematic car rollouts
    speed: The speed at which the car should travel
    min_delta: The minimum allowed steering angle (radians)
    max_delta: The maximum allowed steering angle (radians)
    delta_incr: The difference (in radians) between subsequent possible steering angles
    dt: The amount of time to apply a control for
    T: The number of time steps to rollout for
    car_length: The length of the car
Returns a NxTx3 numpy array that contains N rolled out trajectories, each
containing T poses. For each trajectory, the t-th element represents the [x,y,theta]
pose of the car at time t+1
'''
def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

  deltas = np.arange(min_delta, max_delta, delta_incr)
  N = deltas.shape[0]
  
  init_pose = np.array([0.0,0.0,0.0], dtype=np.float)
  
  rollouts = np.zeros((N,T,3), dtype=np.float)
  for i in xrange(N):
    controls = np.zeros((T,3), dtype=np.float)
    controls[:,0] = speed
    controls[:,1] = deltas[i]
    controls[:,2] = dt
    rollouts[i,:,:] = generate_rollout(init_pose, controls, car_length)
    
  return rollouts, deltas

    

def main():

  rospy.init_node('laser_wanderer', anonymous=True)

  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LaserWanderer class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system  
  # YOUR CODE HERE
  speed = # Default val: 1.0
  min_delta = # Default val: -0.34
  max_delta = # Default val: 0.341
  delta_incr = # Starting val: 0.34/3 (consider changing the denominator) 
  dt = # Default val: 0.01
  T = # Starting val: 300
  compute_time = # Default val: 0.09
  laser_offset = # Starting val: 1.0
  
  # DO NOT ADD THIS TO YOUR LAUNCH FILE, car_length is already provided by teleop.launch
  car_length = rospy.get_param("car_kinematics/car_length", 0.33) 
  
  # Generate the rollouts
  rollouts, deltas = generate_mpc_rollouts(speed, min_delta, max_delta,
                                           delta_incr, dt, T, car_length)
  
  # Create the LaserWanderer                                         
  lw = LaserWanderer(rollouts, deltas, speed, compute_time, laser_offset)
  
  # Keep the node alive
  rospy.spin()
  

if __name__ == '__main__':
  main()
