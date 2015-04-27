#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan

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
      self.obstacle_threshold = 0.5

      # Subscriber for the encoder data
      # When data of type LaserScal arrives from topic 'scan' call laser_callback function immediately
      self.sub = rospy.Subscriber('scan', LaserScan, self.laser_callback) # self.sub.unregister()

      # Publisher for movement commands
      # We publish data of type Twist in velocity topic
      self.cmd_vel = rospy.Publisher("/mobile_base/commands/velocity", Twist)

      # Let the world know we're ready
      # rospy.loginfo('Driver initialized')
      # rospy.loginfo('Start position: '+ str(self.current_position))
      # rospy.loginfo('End position: '+ str(self.end_position))
      # What function to call when you ctrl + c    
      rospy.on_shutdown(self.shutdown)

    # def bug_0(self, turn_orientation = 'Left', accepted_error = 1):
    #   rospy.loginfo('Starting bug_0 algorithm')
    #   rospy.loginfo('Turning orientation is ' + turn_orientation)
    #   rospy.loginfo('Rate is ' + self.rate)

    #   r = rospy.Rate(self.rate)

    #   goal_reached = false
    #   while not goal_reached:
    #     head_toward_goal()
    #     if not is_obstacle():
    #       go_forward()
    #     else:
    #       avoid_obstacle(turn_orientation)
    #       go_forward()
    #     goal_reached = is_goal(accepted_error)
    #     r.sleep()
    #   rospy.loginfo('Congratulations!!! Goal reached')

    # Turn the robot facing the goal
    #def head_toward_goal(self):

    #def turn_on_obstacle(self):

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

    def laser_callback(self, scan):
      closest = min(scan.ranges)
      self.obstacle = self.obstacle_threshold >= closest or (math.isnan(closest) and self.obstacle)
      rospy.loginfo('Reading data from scanner, Distance: {0}'.format(closest))

    # Move the robot in the forward direction
    def go_forward(self, speed = 0.5):
      rospy.loginfo('Moving forward at  Speed : {0}'.format(speed))
      twist_forward = Twist()
      # let's go forward at 0.5 m/s
      twist_forward.linear.x = speed
      # publish the command
      self.cmd_vel.publish(twist_forward)

    # def turn(self, turn_speed = 0.1):
    #   twist_turn = Twist()
    #   # let's go forward at 0.2 m/s
    #   twist_turn.angular.z = turn_speed
    #   # publish the command
    #   self.cmd_vel.publish(twist_forward)

    # Turn the robot with turn_orientation until he stops facing a obstacle 
    # def avoid_obstacle(self, turn_orientation, turn_rate = 0.1):
    #   while not is_obstacle():
    #     turn(turn_orientation, turn_rate)

    # Return true if the robot has reached the goal with the given accepted error. False otherwise.
    # def is_goal(self, accepted_error):
    #   # Math.hypot(current position, end position)

    def stop(self):
      rospy.loginfo('Stopping')
      twist_stop = Twist()
      # publish the command
      self.cmd_vel.publish(twist_stop)

    def shutdown(self):
      rospy.loginfo("Shutting down TurtleBot")
      self.stop()
      rospy.sleep(1)
      rospy.signal_shutdown("Shutdown function has been called")
