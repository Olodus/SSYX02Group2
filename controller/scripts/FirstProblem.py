#!/usr/bin/env python

from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from controller.srv import *
import sys
import math
import rospy

class PresentationProblem(object):

    def initial(self):

	global a
	global path0
	global path0x
	global path0y
	global path1
	global path1x
	global path1y
	global interPoint1
	global interPoint0
	global intersectionPassed
	global hasRan
	global ready1
	global ready0

	intersectionPassed = False
	hasRan = False
	ready1 = False
	ready0 = False
	a = 3.0
	path0 = Path()
	path0x = [a, a, 0.0, 0.0, -a, -a]
	path0y = [0.0, -a, -a, a, a, 0.0]

	path1 = Path()
	path1x = [0.0, -a, -a, a, a, 0.0]
	path1y = [a, a, 0.0, 0.0, -a, -a]

	interPoint1 = Point(0.0, a, 0.0)
	interPoint0 = Point(a, 0.0, 0.0)

	for i in xrange(0,len(path0x)):
		loc0 = Pose()
		loc0.position.x = path0x[i]
		loc0.position.y = path0y[i]
		stamp0 = PoseStamped()
		stamp0.pose = loc0
		path0.poses.append(stamp0)

		loc1 = Pose()
		loc1.position.x = path1x[i]
		loc1.position.y = path1y[i]
		stamp1 = PoseStamped()
		stamp1.pose = loc1
		path1.poses.append(stamp1)

    def run(self):
	global count0
	global count1
	global ready0
	global ready1
	global intersectionPassed
	global hasRan

	if intersectionPassed:
		print "IntersectionPassed = True"
	else:
		print "IntersectionPassed = False"

	rospy.wait_for_service('Robot0/get_state')
	r0_state = rospy.ServiceProxy("Robot0/get_state",GetState)
        data0 = r0_state()
	robot0x = data0.state.pose.pose.position.x
	robot0y = data0.state.pose.pose.position.y
	distToInterPoint0r0 = math.sqrt(math.pow(path0x[2]-robot0x,2)+math.pow(path0y[2]-robot0y,2))
	distToInterPoint1r0 = math.sqrt(math.pow(path0x[5]-robot0x,2)+math.pow(path0y[5]-robot0y,2))

	rospy.wait_for_service('Robot1/get_state')
	r1_state = rospy.ServiceProxy("Robot1/get_state",GetState)
        data1 = r1_state()
	robot1x = data1.state.pose.pose.position.x
	robot1y = data1.state.pose.pose.position.y
	distToInterPoint0r1 = math.sqrt(math.pow(path0x[2]-robot1x,2)+math.pow(path0y[2]-robot1y,2))
	distToInterPoint1r1 = math.sqrt(math.pow(path0x[5]-robot1x,2)+math.pow(path0y[5]-robot1y,2))

	if (distToInterPoint0r0<=0.3 or distToInterPoint1r0<=0.3) and (not ready0):
	    if count0 == 1:
		rospy.wait_for_service('Robot0/stop')
	    	r0_stop = rospy.ServiceProxy("Robot0/stop", Stop)
            	r0_stop()
	    	count0 = 2
		print "robot0 is stoppping"
		rospy.sleep(1.0)
	    if distToInterPoint0r0<=0.3:
		    print "robot0 is turning"
		    rospy.wait_for_service('Robot0/aim_at_point')
		    r0_aim = rospy.ServiceProxy("Robot0/aim_at_point", AimAtPoint)
		    do00 = r0_aim(interPoint1)
		    #print "robot0 is turning to point: x="+str(0)+" y="+str(a)
		    rospy.sleep(5.0)
		    if data0.state.twist.twist.angular.z <= 0.005:
		    	ready0 = True
		    	print "robot0 is ready"
	    if distToInterPoint1r0<=0.3:
		    print "robot0 is turning"
		    rospy.wait_for_service('Robot0/aim_at_point')
		    r0_aim = rospy.ServiceProxy("Robot0/aim_at_point", AimAtPoint)
		    do10 = r0_aim(interPoint0)
		    #print "robot0 is turning to point: x="+str(a)+" y="+str(0)
		    rospy.sleep(5.0)
		    if data0.state.twist.twist.angular.z <= 0.005:
		    	ready0 = True
		    	print "robot0 is ready"
	elif (not ready0):
	    rospy.wait_for_service('Robot0/follow_path')
	    r0_path = rospy.ServiceProxy("Robot0/follow_path", FollowPath)
	    if (not intersectionPassed):
	    	blabla = r0_path(path0)
		print "robot0 is following path0"
	    else:
		blabla = r0_path(path1)
		print "robot0 is following path1"
	    #print "robot0 is following the path"
	    count0 = 1
	    ready0 = False
	    hasRan = False


	if (distToInterPoint0r1<=0.3 or distToInterPoint1r1<=0.3) and (not ready1):
	    if count1 == 1:
		rospy.wait_for_service("Robot1/stop")
	    	r1_stop = rospy.ServiceProxy("Robot1/stop", Stop)
            	r1_stop()
		count1 = 2
		print "robot1 is stopping"
		rospy.sleep(1.0)
	    if distToInterPoint0r1<=0.3:
		    print "robot1 is turning"
		    rospy.wait_for_service('Robot1/aim_at_point')
		    r1_aim = rospy.ServiceProxy("Robot1/aim_at_point", AimAtPoint)
		    do01 = r1_aim(interPoint1)
		    #print "robot1 is turning to point: x="+str(0)+" y="+str(a)
		    rospy.sleep(5.0)
		    if data1.state.twist.twist.angular.z <=0.005:
		    	ready1 = True
		    	print "robot1 is ready"
	    if distToInterPoint1r1<=0.3:
		    print "robot1 is turning"
		    rospy.wait_for_service('Robot1/aim_at_point')
		    r1_aim = rospy.ServiceProxy("Robot1/aim_at_point", AimAtPoint)
		    do11 = r1_aim(interPoint0)
		    #print "robot1 is turning to point: x="+str(a)+" y="+str(0)
		    rospy.sleep(5.0)
		    if data1.state.twist.twist.angular.z <=0.005:
		    	ready1 = True
		    	print "robot1 is ready"
	elif (not ready1):
	    rospy.wait_for_service('Robot1/follow_path')
	    r1_path = rospy.ServiceProxy("Robot1/follow_path", FollowPath)
	    if (not intersectionPassed):
	    	bloblo = r1_path(path1)
		print "robot1 is following path1"
	    else:
		bloblo = r1_path(path0)
		print "robot1 is following path0"
	    #print "robot1 is following the path"
	    count1 = 1
	    ready1 = False
	    hasRan = False

	if ready0 == True and ready1 == True:
			print "-----------------------------------"
			print "Crossing intersection!"

			if (not hasRan):
				intersectionPassed = (not intersectionPassed)
				hasRan = True
			print "Setting robot1 acceleration"
			rospy.wait_for_service("Robot1/set_acc")
			r1_acc = rospy.ServiceProxy("Robot1/set_acc",SetAcc)
			resp1 = r1_acc(0.5)
			rospy.sleep(1.0)
			print "Seting robot0 acceleration"
			rospy.wait_for_service("Robot0/set_acc")
			r0_acc = rospy.ServiceProxy("Robot0/set_acc",SetAcc)
			resp0 = r0_acc(1.0)
			rospy.sleep(1.0)

			print "Robots going to points!"
			print "robot0 is at point: x="+str(data0.state.pose.pose.position.x)+", y="+str(data0.state.pose.pose.position.y)
			print "robot1 is at point: x="+str(data1.state.pose.pose.position.x)+", y="+str(data1.state.pose.pose.position.y)
			rospy.wait_for_service("Robot1/go_to_point")
			r1_go = rospy.ServiceProxy("Robot1/go_to_point", GoToPoint)
			rospy.wait_for_service("Robot0/go_to_point")
			r0_go = rospy.ServiceProxy("Robot0/go_to_point", GoToPoint)

			if intersectionPassed:
				go1 = r1_go(interPoint0)
				go0 = r0_go(interPoint1)
				ready1 = False
				ready0 = False
			else:
				go1 = r1_go(interPoint1)
				go0 = r0_go(interPoint0)
				ready1 = False
				ready0 = False

			rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        rospy.init_node("Controller")
        r = PresentationProblem()
	r.initial()
	rospy.sleep(0.5)
        while True:
            r.run()
            rospy.sleep(1.0)

    except rospy.ROSInterruptException:
        pass

# Vad som kan brytas ut i en super Problem class
#  wait til ready metoden
#  robots variabeln
#  numberOfRobots
#  get_start_positions
