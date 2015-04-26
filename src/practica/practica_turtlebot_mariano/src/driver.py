#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

# We use a hyperbolic tangent as a transfer function
from math import tanh
import math

class Point:
    def __init__(self, x, y, z):
      self.x = x
      self.y = y
      self.z = z

class Driver:
    def __init__(self, start_position, end_position, max_speed=0.2, rate=10):
      self.start_position = start_position
      self.current_position = start_position
      self.end_position = end_position
      self.max_speed = max_speed
      self.rate = rate

      # Subscriber for the encoder data
      # When data of type LaserScal arrives from topic 'scan' call laser_callback function immediately
      # self.sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)

      # Publisher for movement commands
      # We publish data of type Twist in velocity topic
      self.cmd_vel = rospy.Publisher("/mobile_base/commands/velocity", Twist)
      # Set refresh rate

      # Let the world know we're ready
      # rospy.loginfo('Driver initialized')
      # rospy.loginfo('Start position: '+ str(self.current_position))
      # rospy.loginfo('End position: '+ str(self.end_position))
      # What function to call when you ctrl + c    
      rospy.on_shutdown(self.shutdown)

    def bug_0(self, turn_orientation):
      rospy.loginfo('Starting bug_0 algorithm')
      rospy.loginfo('Turning orientation is ' + turn_orientation)
      rospy.loginfo('Rate is ' + self.rate)

      r = rospy.Rate(self.rate)

      goal_reached = false
      while not goal_reached:
        head_toward_goal()
        if not is_obstacle():
          go_forward()
        else:
          avoid_obstacle(turn_orientation)
          go_forward()
        goal_reached = is_goal(current_position, end_position, accepted_error)
        r.sleep()
      rospy.loginfo('Congratulations!!! Goal reached')

    # Turn the robot facing the goal
    def head_toward_goal():

    # Return true if there is a obstacle in the forward direction. False otherwise.
    def is_obstacle():

    # Move the robot in the forward direction
    def go_forward():

    # Turn the robot with turn_orientation until he stops facing a obstacle 
    def avoid_obstacle(turn_orientation):

    # Return true if the robot has reached the goal with the given accepted error. False otherwise.
    def is_goal(turn_orientation, end_position, accepted_error):

    # def go_forward(self, speed = 0.2, rotation = 0):
    #   r = rospy.Rate(self.rate);

    #   move_cmd = Twist()
    #   # let's go forward at 0.2 m/s
    #   move_cmd.linear.x = speed
    #   # let's turn at 0 radians/s
    #   move_cmd.angular.z = rotation

    #   # as long as you haven't ctrl + c keeping doing...
    #   while not rospy.is_shutdown():
    #       # publish the velocity
    #       self.cmd_vel.publish(move_cmd)
    #       # wait for 0.1 seconds (10 HZ) and publish again
    #       r.sleep()

    def shutdown(self):
      # stop turtlebot
      rospy.loginfo("Stop TurtleBot")
      # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
      self.cmd_vel.publish(Twist())
      # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
      rospy.sleep(1)
