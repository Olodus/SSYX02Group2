#!/usr/bin/env python

from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from controller.srv import *
import sys
import math
#import scipy as sp
import rospy
from Helper import RobotServices
import Helper as h

if __name__ == '__main__':
    try:
        rospy.init_node("Controller")
        rospy.sleep(0.5)
        r0 = RobotServices(0)
        print "Controller setup done"

        while True:
            # Both robots go to their start points
            p0 = Point()
            p0.x = -3.0
            p0.y = 0.0
            r0.go_to_point(p0)
            rospy.sleep(0.5)
	    while not r0.is_ready().robot_ready:
		rospy.sleep(1.0)

            # Both robots aim at the other side of the intersections
            p1 = Point()
            p1.x = -3.0
            p1.y = 3.0
            r0.go_to_point(p1)
            rospy.sleep(0.5)
	    while not r0.is_ready().robot_ready:
		rospy.sleep(1.0)

            # Now both robots are ready to enter the intersection
	    p2 = Point()
            p2.x = 0.0
            p2.y = 3.0
            r0.go_to_point(p2)
            rospy.sleep(0.5)
	    while not r0.is_ready().robot_ready:
		rospy.sleep(1.0)

	    p3 = Point()
            p3.x = 0.0
            p3.y = 0.0
            r0.go_to_point(p3)
            rospy.sleep(0.5)
	    while not r0.is_ready().robot_ready:
		rospy.sleep(1.0)


    except rospy.ROSInterruptException:
        pass
