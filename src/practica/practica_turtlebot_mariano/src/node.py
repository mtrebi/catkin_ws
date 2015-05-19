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

    # Create end_pose
    end_pose = Pose()
    end_pose.position.x = 0
    end_pose.position.y = 3

    # Create driver
    driver = Driver(end_pose)

    # Tell him what to do
    # driver.stop_on_obstacle()
    # driver.turn_on_obstacle()
    # driver.bug_0()
    # driver.go_no_obstacle()
    # driver.go_forward_distance(1)
    driver.head_toward_goal()
    # driver.degrees_to_goal()
    # Hand control over to ROS
    # This function will only exit when the user press Ctrl + C
    # Does not do anything. Only handles here the program     
    # rospy.spin()
  except rospy.ROSInterruptException:
    pass
