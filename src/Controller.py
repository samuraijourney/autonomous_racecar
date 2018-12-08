#!/usr/bin/env python

import collections
import numpy as np
import rospy
import sys
import utils as Utils
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msg.msg import Image

# Constants
CAMERA_TOPIC = '/camera/color/image_raw'
CMD_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
PLAN_TOPIC = '/planner_node/car_plan'
POSE_TOPIC = '/sim_car_pose/pose'
ERROR_BUFF_LENGTH = 10

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class Controller:

    '''
    Initializes the line follower
        kp: The proportional PID parameter
        ki: The integral PID parameter
        kd: The derivative PID parameter
        speed: The speed at which the robot should travel
        plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                        then it should navigate towards the (i+plan_lookahead)-th pose in the plan
        translation_weight: How much the error in translation should be weighted in relation
                            to the error in rotation
        rotation_weight: How much the error in rotation should be weighted in relation
                        to the error in translation
    '''
    def __init__(self, kp, ki, kd, speed=1.0, plan_lookahead=1, translation_weight=1.0, rotation_weight=1.0):
        self.avoid_waypoints = []
        self.check_camera = False
        self.cv_bridge = CvBridge()
        self.error_buff = collections.deque(maxlen=ERROR_BUFF_LENGTH)
        self.error_buff.append((0.0, 0.0))
        self.errors = []
        self.kd = kd
        self.ki = ki
        self.kp = kp
        self.plan = []
        self.plan_lookahead = plan_lookahead
        self.rotation_weight = rotation_weight / (translation_weight + rotation_weight)
        self.speed = speed
        self.target_waypoints = []
        self.translation_weight = translation_weight / (translation_weight + rotation_weight)
        self.waypoint_reaction_distance = 2.0

        self.camera_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.__camera_cb)
        self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size=10)
        self.plan_sub = rospy.Subscriber(PLAN_TOPIC, PoseArray, self.__plan_cb)

    def __camera_cb(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    '''
    Checks if the check_pose is located behind from the current pose
        cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
        check_pose: The pose to check if behind, represented as a numpy array [x,y,theta]
    Returns: True if the check_pose is behind the cur_pose, otherwise returns False.
    '''
    def __check_if_pose_behind(self, cur_pose, check_pose):
        a, b = self.__define_line_points(cur_pose, np.pi/2.0)
        p = check_pose[0:2]

        return np.cross(p-a, b-a) > 0

    '''
    Checks if the check_pose is located to the right of the current pose
        cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
        check_pose: The pose to check if to the right, represented as a numpy array [x,y,theta]
    Returns: True if the check_pose is right of cur_pose, otherwise returns False.
    '''
    def __check_if_pose_right(self, cur_pose, check_pose):
        a, b = self.__define_line_points(cur_pose, 0.0)
        p = check_pose[0:2]

        return np.cross(p-a, b-a) > 0

    '''
    Computes the error based on the current pose of the car
        cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
    Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
            (True, E) - where E is the computed error
    '''
    def __compute_error(self, cur_pose):

        # First check if the waypoint error should override the path error
        finished, error = self.__compute_waypoint_error(cur_pose)
        if finished:
            return (False, 0.0)
        if not np.isnan(error):
            return (True, error)

        # Second check the path error since there is no waypoint override
        finished, error = self.__compute_path_error(cur_pose)
        if finished:
            return (False, 0.0)
        return (True, error)

    '''
    Computes the error based on the current pose of the car and the planned path
        cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
    Returns: (True, np.nan) if the end of the plan has been reached. Otherwise, returns
            (False, E) - where E is the computed error
    '''
    def __compute_path_error(self, cur_pose):

        # Find the first element of the plan that is in front of the robot, and remove
        # any elements that are behind the robot.
        while len(self.plan) > 0:
            if self.__check_if_pose_behind(cur_pose, self.plan[0]):
                self.plan.pop(0)
            else:
                break

        # Check if the plan is empty.
        if len(self.plan) == 0:
            return (True, np.nan)

        # At this point, we have removed configurations from the plan that are behind
        # the robot. Therefore, element 0 is the first configuration in the plan that is in
        # front of the robot. To allow the robot to have some amount of 'look ahead',
        # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
        # We call this index the goal_index
        goal_idx = min(self.plan_lookahead, len(self.plan)-1)

        # Compute the translation error between the robot and the configuration at goal_idx in the plan
        goal_pose = self.plan[goal_idx]
        pose_delta = goal_pose - cur_pose
        translation_error = np.linalg.norm(pose_delta[0:2])
        if self.__check_if_pose_right(cur_pose, goal_pose):
            translation_error *= -1

        # Compute the total error
        # Translation error was computed above
        # Rotation error is the difference in yaw between the robot and goal configuration
        q1 = Utils.angle_to_quaternion(goal_pose[2])
        q2 = Utils.angle_to_quaternion(cur_pose[2])
        rotation_error = Utils.angle_between_quaternions(q2, q1)
        error = self.translation_weight * translation_error + self.rotation_weight * rotation_error

        return (False, error)

    '''
    Computes the error based on the current pose of the car and the waypoint position on camera
        cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
    Returns: (True, np.nan) if the end of the plan has been reached. Otherwise, returns
            (False, E) - where E is the computed error, E can be np.nan
    '''
    def __compute_waypoint_error(self, cur_pose):
        if not self.check_camera:
            return (False, np.nan)

        waypoint_info = self.__find_waypoint()
        if waypoint_info is not None:
            if self.__is_waypoint_within_distance(self.waypoint_reaction_distance):
                # Compute error to get to visible waypoint and set error value here

        return error

    '''
    Uses a PID control policy to generate a steering angle from the passed error
        error: The current error
    Returns: The steering angle that should be executed
    '''
    def __compute_steering_angle(self, error):
        now = rospy.Time.now().to_sec() # Get the current time

        # Compute the derivative error using the passed error, the current time,
        # the most recent error stored in self.error_buff, and the most recent time
        # stored in self.error_buff
        last_error = self.error_buff.pop()
        deriv_error = (error - last_error[0]) / (now - last_error[1])

        # Add the current error to the buffer
        self.error_buff.append((error, now))

        # Compute the integral error by applying rectangular integration to the elements
        # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
        integ_error = (now - last_error[1]) * (error + last_error[0]) / 2.0

        # Compute the steering angle as the sum of the pid errors
        print("Error: " + str(error) + ", Integ Error: " + str(integ_error) + ", Deriv Error: " + str(deriv_error))
        return self.kp*error + self.ki*integ_error + self.kd*deriv_error

    '''
    Defines 2 points for a line based off the rotation offset from the current pose
        cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
        rotation_offset: The rotation offset from the current pose theta in radians
    Returns: (a, b) 2 points on the line
    '''
    def __define_line_points(self, cur_pose, rotation_offset=0.0):
        a = cur_pose[0:2]
        offset_vector = np.array([1, 0])
        rot_matrix = Utils.rotation_matrix(cur_pose[2] - rotation_offset)
        b = np.add(a, np.dot(rot_matrix, offset_vector))

        return a, b

    '''
    Searches the camera frame for a waypoint marker
    Returns: [location: (x,y), target: True/False] metadata info for waypoint or None if nothing was found
    '''
    def __find_waypoint(self):
        return None

    '''
    Matches waypoint_info to a waypoint based on closest cached avoid/target waypoints and determines if it is
    within the specified distance to the robot
    Returns: True/False
    '''
    def __is_waypoint_within_distance(self, waypoint_info, distance=2.0):
        return False

    '''
    Callback for the path plan
        msg: A PoseArray defining all the poses for the path plan
    '''
    def __plan_cb(self, msg):
        self.plan = []
        for pose in msg.poses:
            self.plan.append([msg.pose.position.x,
                              msg.pose.position.y,
                              Utils.quaternion_to_angle(msg.pose.orientation)])

        self.pose_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.__pose_cb)

    '''
    Callback for the current pose of the car
        msg: A PoseStamped representing the current pose of the car
    '''
    def __pose_cb(self, msg):
        cur_pose = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             Utils.quaternion_to_angle(msg.pose.orientation)])

        success, error = self.__compute_error(cur_pose)
        self.errors.append(error)
        if not success:
            self.pose_sub = None
            self.speed = 0.0

        delta = self.__compute_steering_angle(error)

        # Setup the control message
        ads = AckermannDriveStamped()
        ads.header.frame_id = '/map'
        ads.header.stamp = rospy.Time.now()
        ads.drive.steering_angle = delta
        ads.drive.speed = self.speed

        # Send the control message
        self.cmd_pub.publish(ads)

    def set_avoid_waypoints(self, waypoints):
        self.check_camera = True
        self.avoid_waypoints = waypoints

    def set_waypoint_reaction_distance(self, distance):
        self.waypoint_reaction_distance = distance

    def set_target_waypoints(self, waypoints):
        self.check_camera = True
        self.target_waypoints = waypoints

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True) # Initialize the node

    plan_lookahead = 1
    translation_weight = 1.0
    rotation_weight = 1.0
    kp = 2.0
    ki = 0.0
    kd = 0.0
    speed = 1.0

    controller = Controller(kp,
                            ki,
                            kd,
                            speed,
                            plan_lookahead,
                            translation_weight,
                            rotation_weight)

    rospy.spin() # Prevents node from shutting down