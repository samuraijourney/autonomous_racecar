#!/usr/bin/env python

import collections
import cv2
import numpy as np
import rospy
import sys
import Utils
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import Image

# Topics
CAMERA_AVOID_WAYPOINT_TOPIC = '/controller/camera/avoid'
CAMERA_TOPIC = '/camera/color/image_raw'
CAMERA_TARGET_WAYPOINT_TOPIC = '/controller/camera/target'
CMD_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
MAP_TOPIC = 'static_map'
PLAN_TOPIC = '/planner_node/car_plan'
POSE_TOPIC = '/pf/viz/inferred_pose'
TARGET_REACHED_TOPIC = '/controller/target_reached'

# Constants
AVOID_COLOR_LOWER_BOUND = np.array([-9,89,192])
AVOID_COLOR_UPPER_BOUND = np.array([11,109,272])
CAMERA_FOV = 69.4 # horizontal FOV in degrees
ERROR_BUFF_LENGTH = 10
PLAN_IGNORE_THRESHOLD = 1.0 # metres
TARGET_WAYPOINTS = np.array([[2600,660,0], [1880,440,0], [1435,545,0], [1250,460,0], [540,835,0]], dtype='float64') # pixels
TARGET_COLOR_LOWER_BOUND = np.array([95,130,135])
TARGET_COLOR_UPPER_BOUND = np.array([115,150,215])
TARGET_REACH_Y_GOAL_THRESHOLD = 0.90 # threshold for target Y position to consider a point reached
WAYPOINT_OFFSET_GAIN = 0.5
WAYPOINT_REACTION_DISTANCE = 1.0 # metres

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
        _, map_info = Utils.get_map(MAP_TOPIC)
        Utils.map_to_world(TARGET_WAYPOINTS, map_info)

        self.cur_pose = None
        self.cv_bridge = CvBridge()
        self.desired_speed = speed
        self.error_buff = collections.deque(maxlen=ERROR_BUFF_LENGTH)
        self.error_buff.append((0.0, 0.0))
        self.errors = []
        self.image_height = None
        self.image_width = None
        self.kd = kd
        self.ki = ki
        self.kp = kp
        self.plan = []
        self.plan_lookahead = plan_lookahead
        self.rotation_weight = rotation_weight / (translation_weight + rotation_weight)
        self.speed = 0.0
        self.target_waypoint_position = None
        self.target_waypoints = TARGET_WAYPOINTS[:,:2]
        self.translation_weight = translation_weight / (translation_weight + rotation_weight)

        self.camera_avoid_pub = rospy.Publisher(CAMERA_AVOID_WAYPOINT_TOPIC, Image, queue_size=10)
        self.camera_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.__camera_cb)
        self.camera_target_pub = rospy.Publisher(CAMERA_TARGET_WAYPOINT_TOPIC, Image, queue_size=10)
        self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size=10)
        self.plan_sub = rospy.Subscriber(PLAN_TOPIC, PoseArray, self.__plan_cb)
        self.pose_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.__pose_cb)
        self.target_reached_pub = rospy.Publisher(TARGET_REACHED_TOPIC, PoseStamped, queue_size=10)

    def __camera_cb(self, msg):
        self.image_height = msg.height
        self.image_width = msg.width

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        target_mask = cv2.inRange(hsv, TARGET_COLOR_LOWER_BOUND, TARGET_COLOR_UPPER_BOUND)
        #avoid_mask = cv2.inRange(hsv, AVOID_COLOR_LOWER_BOUND, AVOID_COLOR_UPPER_BOUND)
        target_image = cv2.bitwise_and(cv_image, cv_image, mask=target_mask)
        #avoid_image = cv2.bitwise_and(cv_image, cv_image, mask=avoid_mask)

        if target_mask[target_mask > 0].size > 0:
            m = cv2.moments(target_mask)
            x = int(m["m10"] / m["m00"])
            y = int(m["m01"] / m["m00"])
            self.target_waypoint_position = np.array([x,y])
            cv2.circle(target_image, (x,y), 5, (255, 255, 255), -1)
        else:
            self.target_waypoint_position = None

        # if avoid_mask[avoid_mask > 0].size > 0:
        #     m = cv2.moments(avoid_mask)
        #     x = int(m["m10"] / m["m00"])
        #     y = int(m["m01"] / m["m00"])
        #     self.avoid_waypoint_position = np.array([x,y])
        #     cv2.circle(avoid_image, (x,y), 5, (255, 255, 255), -1)
        # else:
        #     self.avoid_waypoint_position = None

        try:
            #avoid_image_msg = self.cv_bridge.cv2_to_imgmsg(avoid_image, "bgr8")
            target_image_msg = self.cv_bridge.cv2_to_imgmsg(target_image, "bgr8")
            #self.camera_avoid_pub.publish(avoid_image_msg)
            self.camera_target_pub.publish(target_image_msg)

        except CvBridgeError as e:
            print(e)

        #cv2.imshow("Image Window", cv_image)
        #cv2.imshow("Target Image window", target_image)
        #cv2.imshow("Avoid Image window", avoid_image)
        #cv2.waitKey(5)

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
    Returns: (True, np.nan) if the end of the plan has been reached. Otherwise, returns
            (False, E) - where E is the computed error
    '''
    def __compute_error(self, cur_pose):

        # First check if the waypoint error should override the path error
        finished, error = self.__compute_waypoint_error(cur_pose)
        if finished or (not np.isnan(error)):
            return (finished, error)

        # Second check the path error since there is no waypoint override
        return self.__compute_path_error(cur_pose)

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
        if self.target_waypoint_position is not None:
            dist, degree_offset, selected_waypoint = self.__get_most_likely_waypoint(self.target_waypoints,
                                                                                     cur_pose,
                                                                                     self.target_waypoint_position)

            #if (selected_waypoint is not None):
            #    if dist < WAYPOINT_REACTION_DISTANCE:

             # Consider the waypoint reached if the Y position on the screen exceeds the goal threshold
            if self.target_waypoint_position[1] > (TARGET_REACH_Y_GOAL_THRESHOLD * self.image_height):
                return (True, np.nan)
            else:
                return (False, -degree_offset * WAYPOINT_OFFSET_GAIN)
            #else:
            #    print("Suitable waypoint found but it is to far away, ignoring it")
            #else:
            #    print("No suitable waypoint could be found based on current position")

        return (False, np.nan)

    '''
    Uses a PID control policy to generate a steering angle from the passed error
        error: The current error
    Returns: The steering angle that should be executed
    '''
    def __compute_steering_angle(self, error):
        now = rospy.Time.now().to_sec() # Get the current time

        deriv_error = 0.0
        if len(self.error_buff) >= 1:
            deriv_error = (error - self.error_buff[-1][0])/(now-self.error_buff[-1][1])

        # Add the current error to the buffer
        self.error_buff.append((error, now))
        integ_error = 0.0
        for i in xrange(len(self.error_buff)-1):
            integ_error += 0.5*(self.error_buff[i][0]+self.error_buff[i+1][0])*(self.error_buff[i+1][1]-self.error_buff[i][1])

        # Compute the steering angle as the sum of the pid errors
        #print("Error: " + str(error) + ", Integ Error: " + str(integ_error) + ", Deriv Error: " + str(deriv_error))
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
    Finds the most likely waypoint based on known waypoint positions and the current pose of the robot
        waypoints: Waypoints to match against as multiple numpy arrays of waypoints [x, y]
        pose: Pose to filter points based on as a numpy array [x, y]
        screen_position: The screen position of the detected marker in pixels as a numpy array [x, y]
    Returns: (dist, degree_offset, waypoint) if a likely waypoint was found. Waypoint will be None if
             a suitable waypoint could not be determined.
    '''
    def __get_most_likely_waypoint(self, waypoints, pose, screen_position):

        # Get offset in degrees from robot to detected waypoint marker
        degrees_per_pixel = CAMERA_FOV / self.image_width
        degree_offset = (screen_position[0] - self.image_width / 2) * degrees_per_pixel

        # Define bounding cone which the waypoint position needs to be in based on current pose
        lower_bound = np.deg2rad(degree_offset - 10)
        upper_bound = np.deg2rad(degree_offset + 10)
        a_l, b_l = self.__define_line_points(pose, lower_bound)
        a_u, b_u = self.__define_line_points(pose, upper_bound)

        # Select closest waypoint which is within the bounding cone
        selected_waypoint = None
        min_dist = np.inf
        for waypoint in waypoints:
            to_right_of_lower = np.cross(waypoint-a_l, b_l-a_l) > 0
            to_left_of_upper = np.cross(waypoint-a_u, b_u-a_u) < 0
            if (to_right_of_lower and to_left_of_upper):
                dist = np.linalg.norm(waypoint - pose[0:2])
                if (dist < min_dist):
                    selected_waypoint = waypoint
                    min_dist = dist

        return min_dist, np.deg2rad(degree_offset), selected_waypoint

    '''
    Callback for the path plan
        msg: A PoseArray defining all the poses for the path plan
    '''
    def __plan_cb(self, msg):
        start_pose = [msg.poses[0].position.x,
                      msg.poses[0].position.y]

        # Ignore provided plan if uncertain about current location
        if self.cur_pose is None:
            print("Ignoring received plan, do not know current position")
            return

        # Ignore provided plan if current location is to far from start location
        pose_delta = start_pose - self.cur_pose[0:2]
        distance = np.linalg.norm(pose_delta)
        if distance > PLAN_IGNORE_THRESHOLD:
            print("Ignoring received plan, start position is to far from current")
            return

        self.plan = []
        for pose in msg.poses:
            self.plan.append([pose.position.x,
                              pose.position.y,
                              Utils.quaternion_to_angle(pose.orientation)])

        self.speed = self.desired_speed
        print("New received plan has been set")

    '''
    Callback for the current pose of the car
        msg: A PoseStamped representing the current pose of the car
    '''
    def __pose_cb(self, msg):
        cur_pose = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             Utils.quaternion_to_angle(msg.pose.orientation)])

        self.cur_pose = cur_pose
        success, error = self.__compute_error(cur_pose)
        if np.isnan(error):
            error = 0.0

        self.errors.append(error)
        if (self.speed is not 0.0) and (len(self.plan) != 0) and (success == True):
            ps = PoseStamped()
            ps.header = Utils.make_header("map")
            ps.pose.position.x = cur_pose[0]
            ps.pose.position.y = cur_pose[1]
            ps.pose.orientation = Utils.angle_to_quaternion(cur_pose[2])
            self.target_reached_pub.publish(ps)
            self.speed = 0.0
            print("Plan complete!")

        delta = self.__compute_steering_angle(error)

        # Setup the control message
        ads = AckermannDriveStamped()
        ads.header.frame_id = '/map'
        ads.header.stamp = rospy.Time.now()
        ads.drive.steering_angle = delta
        ads.drive.speed = self.speed

        # Send the control message
        self.cmd_pub.publish(ads)

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True) # Initialize the node

    plan_lookahead = 3
    translation_weight = 1.0
    rotation_weight = 1.0
    kp = 0.5
    ki = 0.0
    kd = 0.1
    speed = 1.0

    controller = Controller(kp,
                            ki,
                            kd,
                            speed,
                            plan_lookahead,
                            translation_weight,
                            rotation_weight)

    rospy.spin() # Prevents node from shutting down