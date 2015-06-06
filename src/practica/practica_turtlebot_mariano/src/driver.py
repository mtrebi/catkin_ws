#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy
import time
import numpy as np

# The velocity command message
from geometry_msgs.msg import Vector3, Twist, Quaternion, Pose, Point

# The laser scan message
from sensor_msgs.msg import LaserScan

# The odometry message
from nav_msgs.msg import Odometry

# We use a hyperbolic tangent as a transfer function
from math import tanh, radians, degrees
import math
import tf

def angle_wrap(a):
    '''
    Returns the angle a normalized between -pi and pi.
    Works with numbers and numpy arrays.
    '''
    a = a % (2 * np.pi)
    if (isinstance(a, int) or isinstance(a, float)) and (a > np.pi):
        a -= 2 * np.pi
    elif isinstance(a, np.ndarray): # arrays
        a[a > np.pi] -= 2 * np.pi
    return a

class Driver:
    def __init__(self, end_pose, rate=5):
      self.current_pose = Pose()
      self.end_pose = end_pose
      self.rate = rate
      self.obstacle = False
      self.obstacle_threshold = 1

      # Subscriber for the encoder data
      # When data of type LaserScal arrives from topic 'scan' call laser_callback function immediately
      self.sub_scan = rospy.Subscriber('scan', LaserScan, self.laser_callback) # self.sub_scan.unregister()
      self.sub_odom = rospy.Subscriber('odom', Odometry, self.odometry_callback) # self.sub_odom.unregister()

      # Publisher for movement commands
      # We publish data of type Twist in velocity topic
      self.cmd_vel = rospy.Publisher("/mobile_base/commands/velocity", Twist)

      # Let the world know we're ready
      rospy.loginfo('Driver initialized')

      # What function to call when you ctrl + c    
      rospy.on_shutdown(self.shutdown)

      time.sleep(1)

    ##################### NAVIGATION #####################

    def bug_0(self, accepted_error = 0.1):
      rospy.loginfo('Starting bug_0 algorithm')
      r = rospy.Rate(self.rate)
      self.head_toward_goal()
      while not self.is_goal(accepted_error):
        if not self.is_obstacle():
          self.go_forward(self.compute_linear_speed())
          self.correct_orientation()
        else:# TODO ESTE ELSE!!!
          while self.is_obstacle():
            self.turn()
            r.sleep()
          self.go_forward_distance(1)
        r.sleep()
      rospy.loginfo('Current position: x = {0}, y = {1}, z = {2}'.format(self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z)) 
      rospy.loginfo('Stopping bug_0 algorithm')

    # def bug_0(self, accepted_error = 0.1):
    #   rospy.loginfo('Starting bug_0 algorithm')
    #   r = rospy.Rate(self.rate)
    #   self.head_toward_goal()
    #   while not self.is_goal(accepted_error):
    #     while self.is_obstacle():
    #       self.turn()
        
    #     if not self.is_obstacle():
    #       self.go_forward(self.compute_linear_speed())
    #       self.correct_orientation()
    #     else:
    #       self.turn_degrees(5)
    #     r.sleep()
    #   rospy.loginfo('Stopping bug_0 algorithm')

    def go_no_obstacle(self):
      rospy.loginfo('Starting go_no_obstacle')
      r = rospy.Rate(self.rate)
      self.head_toward_goal()
      while not self.is_goal():
        # self.go_forward_distance(0.5)
        self.go_forward(self.compute_linear_speed())
        self.correct_orientation()
        r.sleep()

      rospy.loginfo('Current position: x = {0}, y = {1}, z = {2}'.format(self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z)) 
      rospy.loginfo('Stopping go_no_obstacle')
      self.stop()

    def turn_on_obstacle(self):
      rospy.loginfo('Starting turn_on_obstacle')
      r = rospy.Rate(self.rate)
      while not self.is_goal():
        if not self.is_obstacle():
          self.go_forward()
        else:
          self.turn()
        r.sleep()
      rospy.loginfo('Stopping turn_on_obstacle')
      self.stop()

    ##################### SPEED #####################

    def compute_linear_speed(self, max_linear_speed = 10):
      return (self.distance_to_goal()/max_linear_speed)


    def compute_angular_speed(self, max_angular_speed = 180):
      return (self.degrees_to_goal()/max_angular_speed)

    ##################### ORIENTATION #####################
    def turn(self, turn_speed = 45):
      # rospy.loginfo('Turning robot, Speed: {0} degrees/sec'.format(turn_speed))
      twist_turn = Twist()
      # let's go forward at turn_speed degrees/sec
      twist_turn.angular.z = radians(turn_speed)
      # publish the command
      self.cmd_vel.publish(twist_turn)

    def turn_degrees(self, degrees, iterations = 25):
      rospy.loginfo('Turning robot, Degrees: {0} '.format(degrees))
      r = rospy.Rate(self.rate)
      time_per_cicle = 1/float(self.rate)
      total_time = iterations * time_per_cicle
      turn_speed = degrees/total_time

      for i in range(0, iterations):
        self.turn(turn_speed)
        r.sleep()

      self.turn(0)
      time.sleep(1.25)

    ##################### MOVEMENT #####################

    # Move the robot in the forward direction
    def go_forward(self, speed = 2):
      rospy.loginfo('Moving forward, Speed: {0}'.format(speed))
      twist_forward = Twist()
      # let's go forward at speed m/s
      twist_forward.linear.x = speed
      # publish the command
      self.cmd_vel.publish(twist_forward)

    # Move the robot in the forward direction
    def go_forward_distance(self, distance, iterations = 10):
      rospy.loginfo('Moving forward, Distance: {0} '.format(distance))
      r = rospy.Rate(self.rate)
      time_per_cicle = 1/float(self.rate)
      total_time = iterations * time_per_cicle
      forward_speed = distance/total_time

      for i in range(0, iterations):
        self.go_forward(forward_speed)
        r.sleep()

      self.go_forward(0)
      time.sleep(1.25)

    ##################### OBJECTIVE #####################

    def distance_to_goal(self):
      distance = math.hypot(self.end_pose.position.x - self.current_pose.position.x, self.end_pose.position.y - self.current_pose.position.y)
      rospy.loginfo('Distance to goal: {0}'.format(distance))
      return distance;

    # Return true if the robot has reached the goal with the given accepted error. False otherwise.
    def is_goal(self, accepted_error = 0.05):
      return (self.distance_to_goal() < accepted_error);

    def degrees_to_goal(self):
      # Desired angle
      deltaX = self.end_pose.position.x - self.current_pose.position.x
      deltaY = self.end_pose.position.y - self.current_pose.position.y

      desired_angle_radians = math.atan2(deltaY, deltaX)

      # Current angle
      current_quat = self.current_pose.orientation
      current_euler = tf.transformations.euler_from_quaternion([current_quat.x,current_quat.y,current_quat.z,current_quat.w])
      current_position_theta = current_euler[2]

      distance_radians = angle_wrap((desired_angle_radians) - (current_position_theta))
      distance_degrees = degrees(distance_radians)

      # rospy.loginfo('Degrees to face goal theta = {0}'.format(distance_degrees))
      # rospy.loginfo('Degrees to face goal = {0}'.format(self.degrees_to_goal_odom()))

      return distance_degrees;   

    def is_facing_goal(self, accepted_error = 0.05):
      degrees = math.fabs(self.degrees_to_goal())
      rospy.loginfo('Is facing goal error: {0}'.format(degrees))
      return math.fabs(self.degrees_to_goal()) < accepted_error

    # Turn the robot facing the goal
    def head_toward_goal(self):
      self.turn_degrees(self.degrees_to_goal())

    def correct_orientation(self, accepted_error = 5):
      if not self.is_facing_goal(accepted_error):
        rospy.loginfo('Correcting TurtleBot orientation') 
        self.head_toward_goal()

    # Return true if there is a obstacle in the forward direction. False otherwise.
    def is_obstacle(self):
      if self.obstacle:
        rospy.loginfo('Obstacle found!!!')
      return self.obstacle

    ##################### ROS CALLBACKS #####################

    # Laser returns NaN if objects is too far or too near. We must take care!
    def laser_callback(self, scan):
      closest = min(scan.ranges)
      self.obstacle = self.obstacle_threshold >= closest #or (math.isnan(closest) and self.obstacle)

    def odometry_callback(self, odom):
      self.current_pose = odom.pose.pose
      # rospy.loginfo('Odometry data: {0}'.format(odom))
      # Read odometry params
      # rospy.loginfo('Odometry data:')
      # rospy.loginfo('Current position: x = {0}, y = {1}, z = {2}'.format(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)) 
      # rospy.loginfo('Current orientation: x = {0}, y = {1}, z = {2}, w = {3}'.format(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)) 
      # rospy.loginfo('Current linear speed: x = {0}, y = {1}, z = {2}'.format(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z)) 
      # rospy.loginfo('Current angular speed: x = {0}, y = {1}, z = {2}'.format(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z)) 

    ##################### ROS SYSTEM #####################
    def stop(self):
      rospy.loginfo('Stopping TurtleBot')
      twist_stop = Twist()
      self.cmd_vel.publish(twist_stop)

    def shutdown(self):
      rospy.loginfo("Shutting down TurtleBot")
      self.stop()
      rospy.sleep(1)
      rospy.signal_shutdown("Shutdown function has been called")
