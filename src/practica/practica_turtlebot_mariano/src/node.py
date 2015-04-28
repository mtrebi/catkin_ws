#!/usr/bin/env python

# TODO is it necessary here?
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy

from driver import Driver

if __name__ == '__main__':
  try:
    # Starts a unique node with name driver
    rospy.init_node('driver')

    # Create driver
    driver = Driver()

    # Tell him what to do
    # driver.stop_on_obstacle()
    driver.turn_on_obstacle()
    # driver.bug_0()

    # Hand control over to ROS
    # This function will only exit when the user press Ctrl + C
    # Does not do anything. Only handles here the program     
  except rospy.ROSInterruptException:
    pass
