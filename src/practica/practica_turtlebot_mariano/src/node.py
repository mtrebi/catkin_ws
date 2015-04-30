#!/usr/bin/env python

# TODO is it necessary here?
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy

from driver import Driver
from geometry_msgs.msg import Pose

if __name__ == '__main__':
  try:
    # Starts a unique node with name driver
    rospy.init_node('driver')

    # Create start_pose
    start_pose = Pose()
    start_pose.position.x = 0
    start_pose.position.y = 0

    # Create end_pose
    end_pose = Pose()
    end_pose.position.x = 1
    end_pose.position.y = 0

    # Create driver
    driver = Driver(start_pose, end_pose)

    # Tell him what to do
    # driver.stop_on_obstacle()
    # driver.turn_on_obstacle()
    # driver.bug_0()
    driver.head_toward_goal()
    # Hand control over to ROS
    # This function will only exit when the user press Ctrl + C
    # Does not do anything. Only handles here the program     
  except rospy.ROSInterruptException:
    pass
