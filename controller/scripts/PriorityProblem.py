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
	numberOfCrossings = 0
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

	    r0intvel = [0.1,0.5]
	    r1intvel = [0.1,0.5]
	    rand0 = random.uniform(r0intvel[0],r1intvel[1])
	    rand1 = random.uniform(r0intvel[0],r1intvel[1])
	    '''
	    rand0 = 0.406497472315
	    rand1 = 0.314250457359
	    '''
            r0.set_speed(rand0)
            r1.set_speed(rand1)
	    if rand0 < rand1:
	    	rospy.sleep(rand1*6.0)
	    else:
		rospy.sleep(rand0*6.0)
            print "Both robots are now at correct speeds"
	    print "speed set for robot0 is: "+str(rand0)
	    print "speed set for robot1 is: "+str(rand1)

            # Calculate how long it will take for r0 to reach ip
	    
	    #dont think its a good idea to store state in variable
            #r0state = r0.get_state().state
	    #r1state = r1.get_state().state
	    if rand0 <= 0.25 or rand0 >= 0.4 or rand1 <= 0.25 or rand1 >= 0.4:
		displacement = -2.0
	    else:
	    	displacement = -1.0
            r0dist2ip = math.sqrt(math.pow(r0.get_state().state.pose.pose.position.x,2)+math.pow(r0.get_state().state.pose.pose.position.y,2))
	    r1dist2ip = math.sqrt(math.pow(r1.get_state().state.pose.pose.position.x,2)+math.pow(displacement-r1.get_state().state.pose.pose.position.y,2))
            t = r0dist2ip/r0.get_state().state.twist.twist.linear.x

	    print "--------------------------------------"
	    print "number of crossings is: "+str(numberOfCrossings)
	    print "time to cross [0,0] for robot0 is: "+str(t)
	    print "distance to intersection for robot0 is: "+str(r0dist2ip)
	    print "velocity of robot0 is: "+str(r0.get_state().state.twist.twist.linear.x)
            #t = t - 20.0
            v = r1.get_state().state.twist.twist.linear.x
	    a = 2*(r1dist2ip-v*t)/t**2
	    vfinal = v+a*t
	    if vfinal < 0 or math.fabs(a) < 0.005:
		#t = t - 10.0
		#1.0 gives possible collision with high velocities
		displacement = -displacement
		r1dist2ip = math.sqrt(math.pow(r1.get_state().state.pose.pose.position.x,2)+math.pow(displacement-r1.get_state().state.pose.pose.position.y,2))
		a = 2*(r1dist2ip-v*t)/t**2
		vfinal = v+a*t
		while vfinal < 0 or math.fabs(a) < 0.005:
			displacement = displacement + 0.1
			r1dist2ip = math.sqrt(math.pow(r1.get_state().state.pose.pose.position.x,2)+math.pow(displacement-r1.get_state().state.pose.pose.position.y,2))
		    	a = 2*(r1dist2ip-v*t)/t**2
			vfinal = v+a*t
		print "robot1 passes first now!"
	    if vfinal >= 1.0:
		a = -0.5
		print "No optimized solution found, emergency break"
	    print "time to cross [0,"+str(displacement)+"] for robot1 is: "+str(t)
	    print "distance to intersection for robot1 is: "+str(r1dist2ip)
	    print "velocity for robot1 is: "+str(v)
	    print "acceleration for robot1 is: "+str(a)
	    print "--------------------------------------"
            resp = r1.set_acc(a) 

            while r0.get_state().state.pose.pose.position.x < 0.0:
		#ifi r1 has large displacement, keep it from going past the frame
		if r1.get_state().state.pose.pose.position.y >= 2.0:
			p1 = Point()
            		p1.x = 0.0
            		p1.y = 3.0
            		r1.go_to_point(p1)
			rospy.sleep(0.5)
                rospy.sleep(0.5)

            print "robot0 passed mid"
	    numberOfCrossings = numberOfCrossings + 1

	    #possible that r1 has greater final velocity than r0's constant velocity, if r0 doesn't accelerate fast enough
	    #after the intersection, it could result in collision because of the robots tail. Solution to give both same velocity
	    #after one has passed intersection point?

            r1.set_acc(0.1)
            while r0.get_state().state.pose.pose.position.x < 2.0 or r1.get_state().state.pose.pose.position.y < 2.0:
		if r1.get_state().state.pose.pose.position.y >= 2.0:
			p1 = Point()
            		p1.x = 0.0
            		p1.y = 3.0
            		r1.go_to_point(p1)
			rospy.sleep(0.5)
		if r0.get_state().state.pose.pose.position.x >= 2.0:
			p0 = Point()
            		p0.x = 3.0
            		p0.y = 0.0
            		r0.go_to_point(p0)
			rospy.sleep(0.5)
                rospy.sleep(0.5)

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
