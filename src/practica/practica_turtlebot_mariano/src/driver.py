#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan

# The odometry message
from nav_msgs.msg import Odometry

# We use a hyperbolic tangent as a transfer function
from math import tanh
import math

class Point:
    def __init__(self, x, y, z):
      self.x = x
      self.y = y
      self.z = z

class Driver:
    def __init__(self, start_position, end_position, rate=10):
      self.start_position = start_position
      self.current_position = start_position
      self.end_position = end_position
      self.rate = rate
      self.obstacle = False
      self.obstacle_threshold = 1

      # Subscriber for the encoder data
      # When data of type LaserScal arrives from topic 'scan' call laser_callback function immediately
      self.sub_scan = rospy.Subscriber('scan', LaserScan, self.laser_callback) # self.sub.unregister()
      self.sub_odom = rospy.Subscriber('odom', Odometry, self.odometry_callback) # self.sub.unregister()

      # Publisher for movement commands
      # We publish data of type Twist in velocity topic
      self.cmd_vel = rospy.Publisher("/mobile_base/commands/velocity", Twist)

      # Let the world know we're ready
      # rospy.loginfo('Driver initialized')
      # rospy.loginfo('Start position: '+ str(self.current_position))
      # rospy.loginfo('End position: '+ str(self.end_position))
      # What function to call when you ctrl + c    
      rospy.on_shutdown(self.shutdown)

    def bug_0(self, turn_orientation = 'Left', accepted_error = 1):
      rospy.loginfo('Starting bug_0 algorithm')
      r = rospy.Rate(self.rate)

      while not self.is_goal():
        self.head_toward_goal()
        if not self.is_obstacle():
          self.go_forward()
        else:
          self.turn()
          self.go_forward()
        r.sleep()
      rospy.loginfo('Congratulations!!! Goal reached')
      rospy.loginfo('Stopping bug_0 algorithm')

    # Turn the robot facing the goal
    def head_toward_goal(self):
      a = 1

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

    def stop_on_obstacle(self):
      rospy.loginfo('Starting stop_on_obstacle')
      r = rospy.Rate(self.rate)
      while not self.is_obstacle():
        self.go_forward()
        r.sleep()
      rospy.loginfo('Stopping stop_on_obstacle')
      self.stop()

    # Return true if there is a obstacle in the forward direction. False otherwise.
    def is_obstacle(self):
      if self.obstacle:
        rospy.loginfo('Obstacle found!!!')
      return self.obstacle

    # Laser returns NaN if objects is too far or too near. We must take care!
    def laser_callback(self, scan):
      closest = min(scan.ranges)
      self.obstacle = self.obstacle_threshold >= closest or (math.isnan(closest) and self.obstacle)
      rospy.loginfo('Laser data, Distance: {0}'.format(closest))

    
    def odometry_callback(self, odom):
      rospy.loginfo('Odometry data: {0}'.format(odom))

    # Move the robot in the forward direction
    def go_forward(self, speed = 0.5):
      rospy.loginfo('Moving forward, Speed: {0}'.format(speed))
      twist_forward = Twist()
      # let's go forward at 0.5 m/s
      twist_forward.linear.x = speed
      # publish the command
      self.cmd_vel.publish(twist_forward)

    # If turn_speed is to high we may detect a NaN as a near obstacle when there is a far distance
    def turn(self, turn_speed = 0.2):
      rospy.loginfo('Turning robot, Speed: {0}'.format(turn_speed))
      twist_turn = Twist()
      # let's go forward at 0.2 m/s
      twist_turn.angular.z = turn_speed
      # publish the command
      self.cmd_vel.publish(twist_turn)

    # Turn the robot with turn_orientation until he stops facing a obstacle 
    def avoid_obstacle(self, turn_orientation, turn_rate = 0.1):
      while self.is_obstacle():
        self.turn()

    # Return true if the robot has reached the goal with the given accepted error. False otherwise.
    def is_goal(self, accepted_error = 0.1):
      # Math.hypot(current position, end position)
      return False

    def stop(self):
      rospy.loginfo('Stopping')
      twist_stop = Twist()
      self.cmd_vel.publish(twist_stop)

    def shutdown(self):
      rospy.loginfo("Shutting down TurtleBot")
      self.stop()
      rospy.sleep(1)
      rospy.signal_shutdown("Shutdown function has been called")
