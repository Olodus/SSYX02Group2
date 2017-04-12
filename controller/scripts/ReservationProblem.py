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
	startTime = rospy.get_time()
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

            r0time1 = 5.0
            r0time2 = 10.0

            r1time1 = 5.0
            r1time2 = 10.0

            r0state = r0.get_state().state
            r1state = r1.get_state().state

            #The intersection is defined to begin at -1.0 and to end at 1.0
            r0intstart = [-1.0, 0.0]
            r0intend = [1.0, 0.0]
	    r1intstart = [0.0, -1.0]
	    r1intend = [0.0, 1.0]

            r0dist2ip = math.sqrt(math.pow(r0intstart[0]-r0.get_state().state.pose.pose.position.x,2)+math.pow(r0intstart[1]-r0.get_state().state.pose.pose.position.y,2))
	    r1dist2ip = math.sqrt(math.pow(r1intstart[0]-r1state.pose.pose.position.x,2)+math.pow(r1intstart[1]-r1state.pose.pose.position.y,2))
            r0.set_speed(r0dist2ip/r0time1)
	    #r1.set_Speed(r1dist2ip/r1time1)

            ## After r1time1 has passed
            ## Accelerera sa att

	    if r0dist2ip/r0time1 > 1.0:
	    	print "Velocity exceeding limits, try again"

	    r0starttime = rospy.get_time()-startTime+r0time1
            timePassed = rospy.get_time()-startTime
	    tempTime = timePassed

	    print "Problem started after time: "+str(tempTime)
            while timePassed < r0starttime:
	        timePassed = rospy.get_time()-startTime
	        rospy.sleep(0.1)
	    print "First reserved time-slot reached after time: "+str(timePassed-tempTime)

	    r0dist2ip = math.sqrt(math.pow(r0intend[0]-r0.get_state().state.pose.pose.position.x,2)+math.pow(r0intend[1]-r0.get_state().state.pose.pose.position.y,2))
	    v = r0.get_state().state.twist.twist.linear.x
	    t = r0time2-r0time1
	    a = 2*(r0dist2ip-v*t)/t**2
	    vfinal = v+a*t
	    print "Current positions: x="+str(r0.get_state().state.pose.pose.position.x)+", y="+str(r0.get_state().state.pose.pose.position.y)
	    print "Going to position: x="+str(r0intend[0])+", y="+str(r0intend[1])
	    print "distance to end of intersection: "+str(r0dist2ip)
	    print "time left to next time-slot: "+str(t)
	    print "acceleration to reach end of intersection at next time-slot: "+str(a)
	    print "final velocity: "+str(vfinal)
	    '''
	    while vfinal < 0:
		r0dist2ip = math.sqrt(math.pow(r0intend[0]-r0state.pose.pose.position.x,2)+math.pow(r0intend[1]-r0state.pose.pose.position.y,2))
	   	v = r1.get_state().state.twist.twist.linear.x
	    	a = 2*(r1dist2ip-v*t)/t**2
	   	vfinal = v+a*t
	    '''

	    resp = r0.set_acc(a)

	    r0endtime = rospy.get_time()-startTime+t
            timePassed = rospy.get_time()-startTime

	    while timePassed < r0endtime:
		timePassed = rospy.get_time()-startTime
	        rospy.sleep(0.1)
	    print "Second reserved time-slot reached after time: "+str(timePassed-tempTime)

	    
	    p0 = Point()
            p0.x = 3.0
            p0.y = 0.0
            r0.go_to_point(p0)
	    rospy.sleep(1.0)



    except rospy.ROSInterruptException:
        pass
