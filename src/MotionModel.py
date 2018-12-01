#!/usr/bin/env python

import rospy
import numpy as np
import utils as Utils
import math
from std_msgs.msg import Float64
from threading import Lock
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped
import matplotlib.pyplot as plt

KM_V_NOISE = 0.03 # Kinematic car velocity noise std dev
KM_DELTA_NOISE = 0.1 # Kinematic car delta noise std dev
KM_X_FIX_NOISE = 0.01 # Kinematic car x position constant noise std dev
KM_Y_FIX_NOISE = 0.01 # Kinematic car y position constant noise std dev
KM_THETA_FIX_NOISE = 0.05 # Kinematic car theta constant noise std dev

'''
  Propagates the particles forward based on the velocity and steering angle of the car
'''
class KinematicMotionModel:

  '''
    Initializes the kinematic motion model
      motor_state_topic: The topic containing motor state information
      servo_state_topic: The topic containing servo state information    
      speed_to_erpm_offset: Offset conversion param from rpm to speed
      speed_to_erpm_gain: Gain conversion param from rpm to speed
      steering_angle_to_servo_offset: Offset conversion param from servo position to steering angle
      steering_angle_to_servo_gain: Gain conversion param from servo position to steering angle 
      car_length: The length of the car
      particles: The particles to propagate forward
      state_lock: Controls access to particles    
  '''
  def __init__(self, motor_state_topic, servo_state_topic, speed_to_erpm_offset, 
               speed_to_erpm_gain, steering_to_servo_offset,
               steering_to_servo_gain, car_length, particles, state_lock=None):
    self.last_servo_cmd = None # The most recent servo command
    self.last_vesc_stamp = None # The time stamp from the previous vesc state msg
    self.particles = particles
    self.SPEED_TO_ERPM_OFFSET = speed_to_erpm_offset # Offset conversion param from rpm to speed
    self.SPEED_TO_ERPM_GAIN   = speed_to_erpm_gain # Gain conversion param from rpm to speed
    self.STEERING_TO_SERVO_OFFSET = steering_to_servo_offset # Offset conversion param from servo position to steering angle
    self.STEERING_TO_SERVO_GAIN = steering_to_servo_gain # Gain conversion param from servo position to steering angle
    self.CAR_LENGTH = car_length # The length of the car
    
    # This just ensures that two different threads are not changing the particles
    # array at the same time. You should not have to deal with this.
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
      
    # This subscriber just caches the most recent servo position command
    self.servo_pos_sub  = rospy.Subscriber(servo_state_topic, Float64,
                                       self.servo_cb, queue_size=1)
    # Subscribe to the state of the vesc
    self.motion_sub = rospy.Subscriber(motor_state_topic, VescStateStamped, self.motion_cb, queue_size=1)                                       

  '''
    Caches the most recent servo command
      msg: A std_msgs/Float64 message
  '''
  def servo_cb(self, msg):
    self.last_servo_cmd = msg.data # Update servo command

  '''
    Converts messages to controls and applies the kinematic car model to the
    particles
      msg: a vesc_msgs/VescStateStamped message
  '''
  def motion_cb(self, msg):
    self.state_lock.acquire()
    if self.last_servo_cmd is None:
      self.state_lock.release()
      return

    if self.last_vesc_stamp is None:
      self.last_vesc_stamp = msg.header.stamp
      self.state_lock.release()
      return

    # Convert raw msgs to controls
    particle_count = self.particles.shape[0]
    v = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
    d = (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN

    # apply control noise
    v += np.random.normal(0, KM_V_NOISE, particle_count)
    d += np.random.normal(0, KM_DELTA_NOISE, particle_count)

    # Propagate particles forward in place
    dt = msg.header.stamp.to_sec() - self.last_vesc_stamp.to_sec()
    beta = np.arctan(0.5 * np.tan(d))
    theta = self.particles[:,2] + v * np.sin(2 * beta) * dt / float(self.CAR_LENGTH)
    x = self.particles[:,0] + self.CAR_LENGTH * (np.sin(theta) - np.sin(self.particles[:,2])) / np.sin(2 * beta)
    y = self.particles[:,1] + self.CAR_LENGTH * (-np.cos(theta) + np.cos(self.particles[:,2])) / np.sin(2 * beta)

    # apply model noise
    x += np.random.normal(0, KM_X_FIX_NOISE, particle_count)
    y += np.random.normal(0, KM_Y_FIX_NOISE, particle_count)
    theta += np.random.normal(0, KM_THETA_FIX_NOISE, particle_count)

    # Limits all angles from 0 -> 2*pi
    theta = np.mod(theta, 2*np.pi)
    # Limits all angles from -pi -> pi
    theta[theta > np.pi] -= 2*np.pi

    self.particles[:,0] = x
    self.particles[:,1] = y
    self.particles[:,2] = theta

    self.last_vesc_stamp = msg.header.stamp
    self.state_lock.release()

'''
  Code for testing motion model
'''

TEST_SPEED = 1.0 # meters/sec
TEST_STEERING_ANGLE = 0.34 # radians
TEST_DT = 1.0 # seconds

if __name__ == '__main__':
  MAX_PARTICLES = 1000
  
  rospy.init_node("odometry_model", anonymous=True) # Initialize the node
  particles = np.zeros((MAX_PARTICLES,3))

  # Load params
  motor_state_topic = rospy.get_param("~motor_state_topic", "/vesc/sensors/core") # The topic containing motor state information
  servo_state_topic = rospy.get_param("~servo_state_topic", "/vesc/sensors/servo_position_command") # The topic containing servo state information
  speed_to_erpm_offset = float(rospy.get_param("/vesc/speed_to_erpm_offset", 0.0)) # Offset conversion param from rpm to speed
  speed_to_erpm_gain = float(rospy.get_param("/vesc/speed_to_erpm_gain", 4350))   # Gain conversion param from rpm to speed
  steering_angle_to_servo_offset = float(rospy.get_param("/vesc/steering_angle_to_servo_offset", 0.5)) # Offset conversion param from servo position to steering angle
  steering_angle_to_servo_gain = float(rospy.get_param("/vesc/steering_angle_to_servo_gain", -1.2135)) # Gain conversion param from servo position to steering
  car_length = float(rospy.get_param("/car_kinematics/car_length", 0.33)) # The length of the car
    
  # Going to fake publish controls
  servo_pub = rospy.Publisher(servo_state_topic, Float64, queue_size=1)
  vesc_state_pub = rospy.Publisher(motor_state_topic, VescStateStamped, queue_size=1)
  
  kmm = KinematicMotionModel(motor_state_topic, servo_state_topic, speed_to_erpm_offset, 
                             speed_to_erpm_gain, steering_angle_to_servo_offset,
                             steering_angle_to_servo_gain, car_length,particles)
  
  # Give time to get setup
  rospy.sleep(1.0)
  
  # Send initial position and vesc state  
  servo_msg = Float64()
  servo_msg.data = steering_angle_to_servo_gain*TEST_STEERING_ANGLE+steering_angle_to_servo_offset
  
  servo_pub.publish(servo_msg)
  rospy.sleep(1.0)
  
  vesc_msg = VescStateStamped()
  vesc_msg.header.stamp = rospy.Time.now()
  vesc_msg.state.speed = speed_to_erpm_gain*TEST_SPEED+speed_to_erpm_offset  
  vesc_state_pub.publish(vesc_msg)
  
  rospy.sleep(TEST_DT)
  
  vesc_msg.header.stamp = rospy.Time.now()
  vesc_state_pub.publish(vesc_msg)
  
  rospy.sleep(1.0)
  
  kmm.state_lock.acquire()
  # Visualize particles
  plt.xlabel('x')
  plt.ylabel('y')
  plt.scatter([0],[0], c='r')
  plt.scatter(particles[:,0], particles[:,1], c='b')
  plt.show()
  kmm.state_lock.release()