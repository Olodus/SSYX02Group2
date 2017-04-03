#!/usr/bin/env python

from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from controller.srv import *
import sys
import math
import rospy
from Helper import RobotServices

if __name__ == '__main__':
    try:
        rospy.init_node("Controller")
	    rospy.sleep(0.5)
        r0 = RobotServices(0)
        r1 = RobotServices(1)
        while True:
            # Both robots go to their start points
            p0 = Point()
            p0.x = -3.0
            p0.y = 0.0
            r0.go_to_point(p0)
            p1 = Point()
            p1.x = 0.0
            p1.y = -3.0
            r1.go_to_point(p1)
            while not r0.is_ready():
                rospy.sleep(1.0)
            while not r1.is_ready():
                rospy.sleep(1.0)

            # Both robots aim at the other side of the intersections
            p0 = Point()
            p0.x = 3.0
            p0.y = 0.0
            r0.aim_at_point(p0)
            p1 = Point()
            p1.x = 0.0
            p1.y = 3.0
            r1.aim_at_point(p1)
            while not r0.is_ready():
                rospy.sleep(1.0)
            while not r1.is_ready():
                rospy.sleep(1.0)

            # Now both robots are ready to enter the intersection
            # First set the speed they'll enter the problem with
            r0.set_speed(0.5)
            r1.set_speed(0.5)
            rospy.sleep(1.0)
            
            rospy.sleep(1.0)

    except rospy.ROSInterruptException:
        pass
