#!/usr/bin/env python

# TODO is it necessary here?
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy

from driver import Driver
from driver import Point

if __name__ == '__main__':
    try:
        # Starts a unique node with name driver
        rospy.init_node('driver')

        # Get current position
        start_position = Point(0,0,0)
        end_position = Point(0,0,0)

        # Create driver
        driver = Driver(start_position, end_position)

        # Tell him what to do
        driver.go_forward_avoid_obstacles()


        # Hand control over to ROS
        # This function will only exit when the user press Ctrl + C
        # Does not do anything. Only handles here the program
        rospy.spin() 

            
    except rospy.ROSInterruptException:
        pass
