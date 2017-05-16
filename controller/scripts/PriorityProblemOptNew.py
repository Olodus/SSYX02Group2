#!/usr/bin/env python

from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Path
from controller.srv import *
import sys
import math
import tf
import rospy
from Helper import RobotServices
import Helper as h
import random

def intersection_point(x1,y1,theta1,x2,y2,theta2):
	IP_x = (y2-y1-math.atan(theta2)*x2+math.atan(theta1)*x1)/(math.atan(theta1)-math.atan(theta2))
	IP_y = math.atan(theta1)*IP_x+y1-math.atan(theta1)*x1
	IP = [IP_x, IP_y]
	return IP

def robot_angle(state):
	quart = (state.pose.pose.orientation.x, state.pose.pose.orientation.y, state.pose.pose.orientation.z, state.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quart)
        robot_angle = euler[2]
	if robot_angle > math.pi:
            robot_angle = robot_angle-2*math.pi
        if robot_angle < -math.pi:
            robot_angle = robot_angle+2*math.pi
	return robot_angle

def opt_acc(vfinal, a, displacement, r1state, v, t, IP_y):
	displacement_inc = -(IP_y+displacement) - 0.05
	displacement_dec = IP_y+displacement + 0.05
	calculations_int1 = 0
	calculations_int2 = 0
	calculations_ext = 0

	#dont aim behind you
	if IP_y+displacement < r1state.pose.pose.position.y:
		r1dist2ip = math.sqrt(math.pow(r1state.pose.pose.position.x,2)+math.pow(-displacement-r1state.pose.pose.position.y,2))
	    	a = 2*(r1dist2ip-v*t)/t**2
		vfinal = v+a*t

	#displacement can be increased as far as possible but it can't be decreased to behind the robot
	while vfinal < 0 or vfinal >= 1.0 or math.fabs(a) < 0.006:
	    calculations_ext = calculations_ext + 1
	    #displacement_dec = IP_y+displacement + 0.05
	    #displacement_inc = -(IP_y+displacement) - 0.05

	    while vfinal < 0 or math.fabs(a) < 0.006:
		vfinal_old = vfinal
		a_old = a

		displacement_inc = displacement_inc + 0.05
		r1dist2ip = math.sqrt(math.pow(r1state.pose.pose.position.x,2)+math.pow(displacement_inc-r1state.pose.pose.position.y,2))
	    	a = 2*(r1dist2ip-v*t)/t**2
		vfinal = v+a*t
		'''
		if vfinal_old < 0:
			#print "vfinal < 0 ("+str(vfinal_old)+"), calculating new acceleration ("+str(a)+"), new vfinal is: "+str(vfinal)+", displacement is: "+str(displacement_inc)
		if math.fabs(a_old) < 0.006:
			#print "acceleration value was too small ("+str(a_old)+"), calculated new acceleration now has value: "+str(a)+", displacement is: "+str(displacement_inc)
		'''
		calculations_int1 = calculations_int1 + 1
		if calculations_int1 > 500:
			#print "Reached maximum allowed calculations ("+str(calculations_int1)+") for finding final velocity greater than 0, returning emergency acceleration value"
			return -0.5

	    #displacement or displacement_start?
	    if vfinal >= 1.0 and displacement_dec - 0.05 <= r1state.pose.pose.position.y:
		if -0.95 < r1state.pose.pose.position.y < -0.75:
			#print "Probability of collision, returning emergency acceleration value"
			#print "Determined from displacement="+str(displacement_dec)+", vfinal="+str(vfinal)+", y="+str(r1state.pose.pose.position.y)
			return -0.5
		else:
			#print "No fully optimized solution found, try again later"
			#print "Returned acceleration is: "+str(a)
			return a

	    #displacement or displacement_start?
	    while vfinal >= 1.0 and displacement_dec - 0.05 > r1state.pose.pose.position.y:
		vfinal_old = vfinal
		displacement_dec = displacement_dec - 0.05
		r1dist2ip = math.sqrt(math.pow(r1state.pose.pose.position.x,2)+math.pow(displacement_dec-r1state.pose.pose.position.y,2))
	    	a = 2*(r1dist2ip-v*t)/t**2
		vfinal = v+a*t
		#print "vfinal >= 1.0 ("+str(vfinal_old)+"), calculated new acceleration ("+str(a)+"), new vfinal is: "+str(vfinal)+", displacement is: "+str(displacement_dec)
		calculations_int2 = calculations_int2 + 1
		if calculations_int2 > 500:
			#print "Reached maximum allowed calculations ("+str(calculations_int2)+") for finding final velocity lesser than 1.0, returning emergency acceleration value"
			return -0.5

	    if calculations_ext > 50:
		#print "Reached maximum allowed calculations, returning emergency acceleration value"
		#print "Number of calculations done to find satsifying acceleration is: "+str(calculations_ext)
		return -0.5
	
	#print "Number of calculations done to find satsifying acceleration is: "+str(calculations_ext+calculations_int1+calculations_int2)
	#print "Returned acceleration is: "+str(a)
	#print "Current position is: x="+str(r1state.pose.pose.position.x)+", y="+str(r1state.pose.pose.position.y)
	return a

if __name__ == '__main__':
    try:
        rospy.init_node("Controller")
        rospy.sleep(0.5)
        r0 = RobotServices(0)
        r1 = RobotServices(1)

        rospy.wait_for_service("Robot" + str(0) + "/calibrate_angle")
        r0calibrate = rospy.ServiceProxy("Robot" + str(0) + "/calibrate_angle", Stop)
        rospy.wait_for_service("Robot" + str(1) + "/calibrate_angle")
        r1calibrate = rospy.ServiceProxy("Robot" + str(1) + "/calibrate_angle", Stop)
	numberOfCrossings = 0
	a_old = 10000

	
	gdist1 = 1.5
	gdist2 = 2.0
	pdist1 = 4.0
	pdist2 = 1.0
	
	'''
	gdist1 = 1.25
	gdist2 = 4.0
	pdist1 = 1.5
	pdist2 = 1.0
	'''
        print "Controller setup done"

        #r0.set_speed(0.3)
        #r1.set_speed(0.3)
        #rospy.sleep(3.0)
        #r0.stop()
        #r1.stop()

        while True:
            # Both robots go to their start points
            p0 = Point()
            p0.x = -pdist1
            p0.y = 0.0
	    p1 = Point()
            p1.x = 0.0
            p1.y = -pdist1

	    r0.aim_at_point(p0)
	    r1.aim_at_point(p1)

	    h.wait_til_both_ready(r0,r1)

			r0calibrate()
			r1calibrate()

            r0.steer_towards(p0)
	    r0.set_speed(0.4)
            r1.steer_towards(p1)
	    r1.set_speed(0.4)
            #rospy.sleep(1.5)

            h.wait_til_both_ready(r0,r1)
            print "Both robots are now at starting points"

            # Both robots aim at the other side of the intersections
            p0 = Point()
            p0.x = pdist2
            p0.y = 0.0
            r0.aim_at_point(p0)
            p1 = Point()
            p1.x = 0.0
            p1.y = pdist2
            r1.aim_at_point(p1)
            #rospy.sleep(0.5)
            h.wait_til_both_ready(r0, r1)
            print "Both robots are now aimed correctly"

            # Now both robots are ready to enter the intersection
            # First set the speed they'll enter the problem with
	    
	    r0intvel = [0.1,pdist1/15]
	    r1intvel = [0.1,pdist1/15]
	    rand0 = random.uniform(r0intvel[0],r1intvel[1])
	    rand1 = random.uniform(r0intvel[0],r1intvel[1])

	    r0.steer_towards(p0)
            r0.set_speed(rand0)
	    r1.steer_towards(p1)
            r1.set_speed(rand1)

	    if rand0 < rand1:
	    	rospy.sleep(rand1*6.0)
	    else:
		rospy.sleep(rand0*6.0)

            print "Both robots are now at correct speeds"
	    print "speed set for robot0 is: "+str(rand0)
	    print "speed set for robot1 is: "+str(rand1)

	    #is it a good idea to store state in variable?
	    while r0.get_state().state.pose.pose.position.x < 0.0 and r1.get_state().state.pose.pose.position.y <= 0.8*pdist2:
		    r0state = r0.get_state().state
		    r1state = r1.get_state().state
		
		    r0_angle = robot_angle(r0state)
		    r1_angle = robot_angle(r1state)
		    IP = intersection_point(r0state.pose.pose.position.x, r0state.pose.pose.position.y, r0_angle, r1state.pose.pose.position.x, r1state.pose.pose.position.x, r1_angle)
		    IP_x = IP[0]
		    IP_y = IP[1]
		    #print "Intersection point is: x="+str(IP_x)+", y="+str(IP_y)

		    if rand0 <= 0.25 or rand0 >= 0.4 or rand1 <= 0.25 or rand1 >= 0.4:
			displacement = -gdist2
		    else:
		    	displacement = -gdist1
		    r0dist2ip = math.sqrt(math.pow(IP_x-r0state.pose.pose.position.x,2)+math.pow(IP_y-r0state.pose.pose.position.y,2))
		    r1dist2ip = math.sqrt(math.pow(IP_x-r1state.pose.pose.position.x,2)+math.pow(IP_y+displacement-r1state.pose.pose.position.y,2))
		    t = r0dist2ip/r0state.twist.twist.linear.x

		    #print "--------------------------------------"
		    #print "number of crossings is: "+str(numberOfCrossings)
		    #print "time to cross [0,0] for robot0 is: "+str(t)
		    #print "distance to intersection for robot0 is: "+str(r0dist2ip)
		    #print "velocity of robot0 is: "+str(r0state.twist.twist.linear.x)
		    v = r1state.twist.twist.linear.x
		    a = 2*(r1dist2ip-v*t)/t**2
		    vfinal = v+a*t
		
		    a = opt_acc(vfinal, a, displacement, r1state, v, t, IP_y)

		    #print "time to cross [0,"+str(displacement)+"] for robot1 is: "+str(t)
		    #print "distance to initial intersection for robot1 is: "+str(r1dist2ip)
		    #print "velocity for robot1 is: "+str(v)
		    #print "new calculated acceleration value is: "+str(a)

		    if a < a_old:	
		    	resp = r1.set_acc(a)
		        a_old = a
			#print "new minimzed acceleration solution found, acceleration for robot1 is now: "+str(a)
		        #print "--------------------------------------"
		    else:
			resp = r1.set_acc(a_old)
			#print "acceleration for robot1 is: "+str(a_old)
		        #print "--------------------------------------"

		    rospy.sleep(0.5)

            #print "robot0 passed mid"
	    numberOfCrossings = numberOfCrossings + 1

	    #possible that r1 has greater final velocity than r0's constant velocity, if r0 doesn't accelerate fast enough
	    #after the intersection, it could result in collision because of the robots tail. Solution to give both same velocity
	    #after one has passed intersection point?

            r1.set_acc(0.025)

            h.wait_til_both_ready(r0, r1)

            temp_r = r0
            r0 = r1
            r1 = temp_r

            temp_c = r0calibrate
            r0calibrate = r1calibrate
            r1calibrate = temp_c

	    a_old = 10000


    except rospy.ROSInterruptException:
        pass
