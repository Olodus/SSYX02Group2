#!/usr/bin/env python

from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from controller.srv import *
import sys
import math
import rospy
from Helper import RobotServices
import Helper as h

if __name__ == '__main__':
    try:
        rospy.init_node("Controller")
        rospy.sleep(0.5)
        r0 = RobotServices(0)
        r1 = RobotServices(1)
        print "Controller setup done"

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
            rospy.sleep(0.5)
            h.wait_til_both_ready(r0,r1)
            print "Both robots are now at starting points"

            # Both robots aim at the other side of the intersections
            p0 = Point()
            p0.x = 3.0
            p0.y = 0.0
            r0.aim_at_point(p0)
            p1 = Point()
            p1.x = 0.0
            p1.y = 3.0
            r1.aim_at_point(p1)
            rospy.sleep(0.5)
            h.wait_til_both_ready(r0, r1)
            print "Both robots are now aimed correctly"

            # Now both robots are ready to enter the intersection
            # First set the speed they'll enter the problem with
            r0.set_speed(0.5)
            r1.set_speed(0.5)
            rospy.sleep(1.5)
            print "Both robots are now at correct speed"

            # Calculate how long it will take for r0 to reach ip
            r0state = r0.get_state().state
            r0dist2ip = math.sqrt(math.pow(r0state.pose.pose.position.x,2)+math.pow(r0state.pose.pose.position.y,2))
            t = r0dist2ip/r0state.twist.twist.linear.x

            t = t + 2.0
            v = r1.get_state().state.twist.twist.linear.x
            resp = r1.set_acc(-0.09)

            while r0.get_state().state.pose.pose.position.x < 0.0:
                rospy.sleep(0.5)

            print "r0 passed mid"

            r1.set_acc(0.3)
            while r0.get_state().state.pose.pose.position.x < 2.0:
                rospy.sleep(0.5)

            print "r1 passed y=2.0"

            p0 = Point()
            p0.x = 3.0
            p0.y = 0.0
            r0.go_to_point(p0)
            p1 = Point()
            p1.x = 0.0
            p1.y = 3.0
            r1.go_to_point(p1)
            rospy.sleep(0.5)
            h.wait_til_both_ready(r0, r1)

            temp_r = r0
            r0 = r1
            r1 = temp_r


    except rospy.ROSInterruptException:
        pass
