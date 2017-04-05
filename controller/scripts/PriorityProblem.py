#!/usr/bin/env python

from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from controller.srv import *
import sys
import math
import rospy
from Helper import RobotServices
import Helper as h
import random

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
            p0.x = -5.0
            p0.y = 0.0
            r0.go_to_point(p0)
            p1 = Point()
            p1.x = 0.0
            p1.y = -5.0
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
	    r0.set_speed(0.3)
            r1.set_speed(0.3)
	    rospy.sleep(1.5)
	    '''
	    rand0 = random.uniform(0.25,0.4)
	    rand1 = random.uniform(0.25,0.4)
            r0.set_speed(rand0)
            r1.set_speed(rand1)
	    if rand0 < rand1:
	    	rospy.sleep(rand1*5)
	    else:
		rospy.sleep(rand0*5)
	    '''
            print "Both robots are now at correct speeds"
	    '''
	    print "speed set for robot0 is: "+str(rand0)
	    print "speed set for robot1 is: "+str(rand1)
	    '''

            # Calculate how long it will take for r0 to reach ip
            r0state = r0.get_state().state
	    r1state = r1.get_state().state
	    displacement = -1.0
            r0dist2ip = math.sqrt(math.pow(r0state.pose.pose.position.x,2)+math.pow(r0state.pose.pose.position.y,2))
	    r1dist2ip = math.sqrt(math.pow(r1state.pose.pose.position.x,2)+math.pow(displacement-r1state.pose.pose.position.y,2))
            t = r0dist2ip/r0state.twist.twist.linear.x

	    print "time to cross for robot0 is: "+str(t)
	    print "distance to intersection for robot0 is: "+str(r0dist2ip)
	    print "velocity of robot0 is: "+str(r0state.twist.twist.linear.x)
            #t = t + 20.0
            v = r1.get_state().state.twist.twist.linear.x
	    a = 2*(r1dist2ip-v*t)/t**2
	    '''
	    vfinal = v+a*t
	    if vfinal < 0:
		t = t - 10.0
	    	a = 2*(r1dist2ip-v*t)/t**2
		print "robot1 passes first now!"
	    '''
	    #a = -0.02
	    print "acceleration for robot1 is: "+str(a)
	    print "time to cross for robot1 is: "+str(t)
	    print "velocity for robot1 is: "+str(v)
	    print "distance to intersection for robot1 is: "+str(r1dist2ip)
            resp = r1.set_acc(a)

            while r0.get_state().state.pose.pose.position.x < 0.0:
                rospy.sleep(0.5)
		#print "velocity for robot1 is: "+str(r1.get_state().state.twist.twist.linear.x)

	    '''now = rospy.get_time()

	    while r1.get_state().state.pose.pose.position.x < 0.0:
	        rospy.sleep(0.1)

	    now2 = rospy.get_time()
	    print str(now2-now)
	    '''
            print "r0 passed mid"

            r1.set_acc(0.1)
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
