#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy
import time
import math
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

class Driver(object):
    def __init__(self, rate=5):


        # Create end_pose
        self.end_pose = Pose()
        self.end_pose.position.x = rospy.get_param('x',0)
        rospy.loginfo('X: {0}'.format(self.end_pose.position.x ))
        self.end_pose.position.y = rospy.get_param('y',0)
        rospy.loginfo('Y: {0}'.format(self.end_pose.position.y))

        self.current_pose = Pose()
        self.rate = rate
        self.obstacle = False
        self.final_point = None


        #VARIABLES
        
        #this app use status that lets make a execution in realtime
        self.status=0 #0: Moving towards goal. 1: front obstacble, robot turns  2: forward to avoid obstable   5: finished

        #max error of precision of the goal accepted
        self.accepted_error = 0.05

        #distance (m) detection of obstacte when the robot stop and begin to turn 
        self.obstacle_threshold = 1
        #distance (m) obstacle_threshold+obstacle_threshold_add when the robot detects that the robot i more far starts to go fowrward to avoid it
        self.obstacle_threshold_add = 0.25
        #distance (m) in status 2 that the robot move to try avoid the obstracle and go to the goal after turn in front of obstracle
        self.distance_try_avoid_obstacle = 0.4

        #######GO FOWARD######### 
        #this linear variable * distance to goal lets to make a variable velocity
        self.linear_constant=0.5
        #max speed of the robot, if is very high is possible that the robot don't have enought time for stop whith obstabce
        self.max_speed=0.25

        #######GO FOWARD DISTANCE######### 
        self.accepted_error_try_avoid_obstable = 0.1

        #######TURN######### 
        #degrees/sec that turns the robot when detects obstable.
        self.turn_speed = 25 #velocitat ideal si fa mes no dona temps al laserscan a actuar

        ####### head_toward_goal #########  
        #this linear velocity variable * distance to goal lets to make a variable linear velocity
        self.head_toward_linear_constant=0.1
        #this angular velocity variable * distance to goal lets to make a variable angular velocity
        self.head_toward_angular_constant=0.2



        # Subscriber for the encoder data
        # When data of type LaserScal arrives from topic 'scan' call laser_callback function immediately
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.laser_callback) # self.sub_scan.unregister()
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odometry_callback) # self.sub_odom.unregister()

        # Publisher for movement commands
        # We publish data of type Twist in velocity topic
        self.cmd_vel = rospy.Publisher("/mobile_base/commands/velocity", Twist)

        # Let the world know we're ready
        rospy.loginfo('Driver initialized')
        rospy.loginfo('Starting bug_0 algorithm')

    ##################### NAVIGATION #####################

    def bug0(self):
        rospy.loginfo('status: {0}'.format( self.status))

        if self.status==0: #Move towards goal
            self.move_toward_goal()
        elif self.status==1:
            print "front obstacle we turn"
            self.turn()
        elif self.status==2:
            print "we forward 0.4 meten and try if  the object is here"
            self.go_forward_X_distance()



    ##################### ORIENTATION #####################
    def turn(self):
      # rospy.loginfo('Turning robot, Speed: {0} degrees/sec'.format(turn_speed))
      twist_turn = Twist()
      # let's go forward at turn_speed degrees/sec
      twist_turn.angular.z = radians(self.turn_speed)
      # publish the command
      self.cmd_vel.publish(twist_turn)


    ##################### MOVEMENT #####################

    # Move the robot in the forward direction
    #def go_forward(self):
    #  twist_forward = Twist()
    # let's go forward at speed m/s
    #  twist_forward.linear.x = self.distance_to_goal()* self.linear_constant
    #  rospy.loginfo('Moving forward, Speed: {0}'.format(twist_forward.linear.x))
    # publish the command
    #  self.cmd_vel.publish(twist_forward)

    # Move the robot in the forward direction adding le current position the definet final_point to avoid obstable
    def go_forward_X_distance(self):
      distance = math.hypot(self.final_point.x - self.current_pose.position.x, self.final_point.y - self.current_pose.position.y)
      if (distance> self.accepted_error_try_avoid_obstable):
          twist_forward = Twist()
          # let's go forward at speed m/s
          twist_forward.linear.x = distance* self.linear_constant
          rospy.loginfo('Moving forward X distance, Speed: {0}'.format(twist_forward.linear.x))
          # publish the command
          self.cmd_vel.publish(twist_forward)
      else:
          self.status=0


    ##################### OBJECTIVE #####################

    def move_toward_goal(self):
        if not self.is_goal():
            self.head_toward_goal()
        else:
            print "Finished"
            self.status=5 #Finished
    
    def distance_to_goal(self):
      distance = math.hypot(self.end_pose.position.x - self.current_pose.position.x, self.end_pose.position.y - self.current_pose.position.y)
      rospy.loginfo('Distance to goal: {0}'.format(distance))
      return distance;

    # Return true if the robot has reached the goal with the given accepted error. False otherwise.
    def is_goal(self):
      return (self.distance_to_goal() < self.accepted_error);

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

      rospy.loginfo('Degrees to face goal theta = {0}'.format(distance_degrees))
      # rospy.loginfo('Degrees to face goal = {0}'.format(self.degrees_to_goal_odom()))

      return distance_degrees;   


    # Turn the robot facing the goal
    def head_toward_goal(self):
        twist_turn = Twist()
        # let's go forward at speed m/s

        #si estem molt mal posats la divisio sera molt alta i anira amb velocitat linieal lenta i rectificara l0angle, si no comencaria a donar voltes
        twist_turn.linear.x = min(self.distance_to_goal()*self.head_toward_linear_constant/abs(self.degrees_to_goal()), self.max_speed)
        rospy.loginfo('Moving forward, Speed: {0}'.format(twist_turn.linear.x))
        # let's go forward at turn_speed degrees/sec

        twist_turn.angular.z = radians(self.degrees_to_goal()*self.head_toward_angular_constant)
        rospy.loginfo('Turning to goal, Speed: {0}'.format(twist_turn.angular.z))
        # publish the command
        self.cmd_vel.publish(twist_turn)


    ##################### ROS CALLBACKS #####################

    # Laser returns NaN if objects is too far or too near. We must take care!
    def laser_callback(self, scan):
      closest = min(scan.ranges)
      print "Real closest range is:", closest
      if np.isnan(closest):
        closest=999 #when closest is nan = very fast,  is not possible too near because the robot turn before
      print "Closest range is:", closest

      self.obstacle = self.obstacle_threshold >= closest

      if (self.status==0 or self.status==2) and self.obstacle:
        self.status=1 #turn object

      if self.status==1 and  (self.obstacle_threshold + self.obstacle_threshold_add) < closest  :
        self.status=2 #Following object
        #Compute next point to follow object
        current_quat=self.current_pose.orientation
        current_euler = tf.transformations.euler_from_quaternion([current_quat.x,current_quat.y,current_quat.z,current_quat.w])
        current_position_theta = current_euler[2]
        self.final_point= Point();
        self.final_point.x=self.current_pose.position.x+ self.distance_try_avoid_obstacle * (np.cos(current_position_theta))
        self.final_point.y=self.current_pose.position.y+ self.distance_try_avoid_obstacle * (np.sin(current_position_theta))

    def odometry_callback(self, odom):
      self.current_pose = odom.pose.pose


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
