#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

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

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

    def go_forward(self, speed = 0.2, rotation = 0):
        r = rospy.Rate(self.rate);

        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = speed
        # let's turn at 0 radians/s
        move_cmd.angular.z = rotation

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            # publish the velocity
            self.cmd_vel.publish(move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    def go_forward_avoid_obstacles(self):
        #tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        #allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        #we'll send a goal to the robot to move 3 meters forward
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 3.0 #3 meters
        goal.target_pose.pose.orientation.w = 1.0 #go forward

        #start moving
        self.move_base.send_goal(goal)

        #allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 


        if not success:
                    self.move_base.cancel_goal()
                    rospy.loginfo("The base failed to move forward 3 meters for some reason")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved 3 meters forward")


