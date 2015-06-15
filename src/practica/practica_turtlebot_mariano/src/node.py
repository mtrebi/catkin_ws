#!/usr/bin/env python

# TODO is it necessary here?
import roslib; roslib.load_manifest('practica_turtlebot_mariano')
import rospy

from driver import Driver
from geometry_msgs.msg import Pose

if __name__ == '__main__':
  try:
    # Starts a unique node with name driver
    rospy.init_node('driver')



    driver = Driver()

    #bucle principal del programa cada 0.03 segons mira si ha rebut missatges.
    while not rospy.is_shutdown() and not driver.status==5:
        driver.bug0()
        rospy.sleep(0.03)     

  except rospy.ROSInterruptException:
    pass
