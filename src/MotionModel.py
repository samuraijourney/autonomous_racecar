#!/usr/bin/env python

import rospy
import numpy as np
import Utils
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
KM_X_SCALE_NOISE = 0.0 # Kinematic car x position scale noise std dev
KM_Y_SCALE_NOISE = 0.0 # Kinematic car y position scale noise std dev

'''
  Propagates the particles forward based on the difference between the two most
  recent odometry readings
'''
class OdometryMotionModel:

  '''
    Initializes the OdometryMotionModel
    odometry_topic: The topic that contains odometry information
    particles: The particles to propagate forward
    state_lock: Used to control access to particles
  '''
  def __init__(self, odometry_topic, particles, state_lock=None):
    self.last_pose = None # The last pose that was received
    self.particles = particles
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock

    # Subscribe to the vesc odometry
    self.motion_sub = rospy.Subscriber(odometry_topic, Odometry, self.motion_cb, queue_size=1)

  '''
    Approximates the motion of the car based on the most recent and previous odometry
    msgs, then applies this motion and gaussian noise to the particles
      msg: A nav_msgs/Odometry message
  '''
  def motion_cb(self, msg):
    self.state_lock.acquire()

    # Form the current pose of the car w.r.t to the odom frame
    position = np.array([msg.pose.pose.position.x,
		         msg.pose.pose.position.y])
    orientation = Utils.quaternion_to_angle(msg.pose.pose.orientation)
    pose = np.array([position[0], position[1], orientation])

    if isinstance(self.last_pose, np.ndarray):
      # Comute the car's current offset w.r.t the last pose
      rot = Utils.rotation_matrix(-self.last_pose[2])
      delta = np.array([position - self.last_pose[0:2]]).transpose()
      local_delta = (rot*delta).transpose()

		  # Control is the current pose of the car w.r.t the last pose
      control = np.array([local_delta[0,0], local_delta[0,1], orientation - self.last_pose[2]])
      self.apply_motion_model(self.particles, control)

    self.last_pose = pose # Cache for next iteration
    self.state_lock.release()

  '''
    Propagate the particles forward (in-place) by the estimated motion and add sampled gaussian noise
      proposal_dist: The particles to propagate forward
      control: The estimated chage in the car's pose [dx, dy, dtheta]
  '''
  def apply_motion_model(self, proposal_dist, control):

    cosines = np.cos(proposal_dist[:,2])
    sines = np.sin(proposal_dist[:,2])

    # Rotate motion into frame of the world
    proposal_dist[:,0] += cosines*control[0] - sines*control[1]
    proposal_dist[:,1] += sines*control[0] + cosines*control[1]
    proposal_dist[:,2] += control[2]

    # Add motion and gaussian noise
    add_rand = 0.05
    proposal_dist[:,0] += np.random.normal(loc=0.0,scale=add_rand,size=proposal_dist.shape[0])
    proposal_dist[:,1] += np.random.normal(loc=0.0,scale=add_rand*0.5,size=proposal_dist.shape[0])
    proposal_dist[:,2] += np.random.normal(loc=0.0,scale=0.25,size=proposal_dist.shape[0])

    # Limit rotation to be between -pi and pi
    proposal_dist[proposal_dist[:,2] < -1*np.pi,2] += 2*np.pi
    proposal_dist[proposal_dist[:,2] > np.pi,2] -= 2*np.pi

'''
  Propagates the particles forward based the velocity and steering angle of the car
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
      print ("Vesc callback called for first time....")
      self.last_vesc_stamp = msg.header.stamp
      self.state_lock.release()
      return

    # Convert raw msgs to controls
    # Note that control = (raw_msg_val - offset_param) / gain_param
    curr_speed = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN

    curr_steering_angle = (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
    dt = (msg.header.stamp - self.last_vesc_stamp).to_sec()

    # Propagate particles forward in place
    self.apply_motion_model(proposal_dist=self.particles, control=[curr_speed, curr_steering_angle, dt])

    self.last_vesc_stamp = msg.header.stamp
    self.state_lock.release()

  '''
    Propagates particles forward (in-place) by applying the kinematic model and adding
    sampled gaussian noise
      proposal_dist: The particles to propagate
      control: List containing velocity, steering angle, and timer interval - [v,delta,dt]
  '''
  def apply_motion_model(self, proposal_dist, control):

    # Update the proposal distribution by applying the control to each particle

    v, delta, dt = control
    v_mag = np.abs(v)
    delta_mag = np.abs(delta)
    # Sample control noise and add to nominal control
    v += np.random.normal(loc=0.0, scale=KM_V_NOISE, size=proposal_dist.shape[0])
    delta += np.random.normal(loc=0.0, scale=KM_DELTA_NOISE, size=proposal_dist.shape[0])

    # Compute change in pose based on controls
    if delta_mag < 1e-2:
        dx = v * np.cos(proposal_dist[:, 2]) * dt
        dy = v * np.sin(proposal_dist[:, 2]) * dt
        dtheta = 0
    else:
        beta = np.arctan(0.5 * np.tan(delta))
        sin2beta = np.sin(2 * beta)
        dtheta = ((v / self.CAR_LENGTH) * sin2beta) * dt
        dx = (self.CAR_LENGTH/sin2beta)*(np.sin(proposal_dist[:,2]+dtheta)-np.sin(proposal_dist[:,2]))
        dy = (self.CAR_LENGTH/sin2beta)*(-1*np.cos(proposal_dist[:,2]+dtheta)+np.cos(proposal_dist[:,2]))

    # Propagate particles forward, and add sampled model noise
    proposal_dist[:, 0] += dx + np.random.normal(loc=0.0, scale=KM_X_FIX_NOISE+KM_X_SCALE_NOISE*v_mag, size=proposal_dist.shape[0])
    proposal_dist[:, 1] += dy + np.random.normal(loc=0.0, scale=KM_Y_FIX_NOISE+KM_Y_SCALE_NOISE*v_mag, size=proposal_dist.shape[0])
    proposal_dist[:, 2] += dtheta + np.random.normal(loc=0.0, scale=KM_THETA_FIX_NOISE, size=proposal_dist.shape[0])

    # Limit particle totation to be between -pi and pi
    proposal_dist[proposal_dist[:,2] < -1*np.pi,2] += 2*np.pi
    proposal_dist[proposal_dist[:,2] > np.pi,2] -= 2*np.pi

'''
  Code for testing motion model
'''

TEST_SPEED = 1.0
TEST_STEERING_ANGLE = 0.34
TEST_DT = 1.0

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
