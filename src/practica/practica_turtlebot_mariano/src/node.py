#!/usr/bin/env python

# TODO is it necessary here?
import roslib; roslib.load_manifest('practica_turtlebot')
import rospy

from driver import driver

if __name__ == '__main__':
    try:
        # Starts a unique node with name driver
        rospy.init_node('driver')

        # Get stoper distance threshold from parameter server.  Default is 0.5
        stop_distance = rospy.get_param('distance', 0.5)

        # Set up the controller
        stopper = driver(stop_distance)

        # Hand control over to ROS
        # This function will only exit when the user press Ctrl + C
        # Does not do anything. Only handles here the program
        rospy.spin() 

            
    except rospy.ROSInterruptException:
        pass
