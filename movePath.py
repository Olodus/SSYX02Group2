#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Quaternion
import tf
import math
import numpy as np

import roslib;


# rospy.loginfo(rospy.get_caller_id()+"I heard %s", data.pose.pose.position.x)

#p = 0
#pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=1)
#twist = Twist()
#PI=math.pi

def setAngle(theta_curr,theta,tempPub,tempTwist):

    if math.fabs(theta_curr-theta)>10*PI/180:
	tempTwist.linear.x = 0
	if theta_curr < theta:
		if math.fabs(theta_curr - theta)<PI:
			tempTwist.angular.z = 0.3#*math.copysign(1,theta_curr-theta)
		else:
			tempTwist.angular.z = -0.3
	else:
		if math.fabs(theta_curr - theta)<PI:
			tempTwist.angular.z = -0.3
		else:
			tempTwist.angular.z = 0.3
	print "-------------------- ANGLE FAR ------------------------------"
	print "current theta:"+str(theta_curr*180/PI)
	print "theta to desired point:"+str(theta*180/PI)
	print "theta difference: "+str(math.fabs(theta_curr-theta)*180/PI)
    	tempPub.publish(tempTwist)
    	rospy.sleep(0.01)

    elif 3*PI/180<=math.fabs(theta_curr-theta) and math.fabs(theta_curr-theta)<=8*PI/180:
	tempTwist.linear.x = 0.3
	if theta_curr < theta:
		if math.fabs(theta_curr - theta)<PI:
			tempTwist.angular.z = 0.1#*math.copysign(1,theta_curr-theta)
		else:
			tempTwist.angular.z = -0.1
	else:
		if math.fabs(theta_curr - theta)<PI:
			tempTwist.angular.z = -0.1
		else:
			tempTwist.angular.z = 0.1
    	#twist.angular.z = -0.05*math.copysign(1,theta_curr-theta)
    	tempPub.publish(tempTwist)
	print "lllllllllllllllllllll ANGLE NEAR llllllllllllllllllllllllllllll"
	print "current theta:"+str(theta_curr*180/PI)
	print "theta to desired point:"+str(theta*180/PI)
	print "theta difference: "+str(math.fabs(theta_curr-theta)*180/PI)
    	rospy.sleep(0.1)

    else:
	tempTwist.linear.x=0.5
	tempTwist.angular.z=0
	tempPub.publish(tempTwist)
	print "aaaaaaaaaaaaaaaaaaaaaaa ANGLE EXACT aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
	print "current theta:"+str(theta_curr*180/PI)
	print "theta to desired point:"+str(theta*180/PI)
	print "theta difference: "+str(math.fabs(theta_curr-theta)*180/PI)
	rospy.sleep(0.01)

def distanceToPoint(x,y,pointx,pointy):
    dist = math.sqrt(math.pow(pointx-x,2)+math.pow(pointy-y,2))
    print "distance to point:"+str(dist)
    return dist

def angleToPoint(x,y,pointx,pointy):

    angle = math.atan2(pointy-y,pointx-x)
    '''
    if pointy > y and pointx > x:
	print "Intern kvadrant 1"
        angle = math.atan(math.fabs(pointy-y)/math.fabs(pointx-x))
    if pointy > y and pointx < x:
	print "Intern kvadrant 2"
        angle = PI/2 + math.atan(math.fabs(x-pointx)/math.fabs(pointy-y))
    if pointy < y and pointx < x:
	print "Intern kvadrant 3"
        angle = -PI + math.atan(math.fabs(y-pointy)/math.fabs(x-pointx))
    if pointy < y and pointx > x:
	print "Intern kvadrant 4"
        angle = -math.atan(math.fabs(pointx-x)/math.fabs(y-pointy))
    if pointy == y and pointx > x:
        angle = 0
    if pointy == y and pointx < x:
        angle = PI
    if pointy > y and pointx == x:
        angle = PI/2
    if pointy < y and pointx == x:
        angle = -PI/2'''
    return angle

def callback(data):
    global p
    global pathx
    global pathy
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quart)
    
    if distanceToPoint(x,y,pathx[p],pathy[p])<=0.1:

	p= (p+1)%6
	print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
	print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
	print "+++++++++++++++++++ VI HAR NATT EN PUNKT ++++++++++++++++++++++"
	print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
	print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
        print "next point:"+str(p+1)
	rospy.sleep(1.0)

    else:
        #global twist
	#global pub
	tempAngle = angleToPoint(x, y, pathx[p], pathy[p])
	setAngle(euler[2], tempAngle, pub,twist)
	#print "headed towards point: "+str(p+1)
	#twist.linear.x = 0.1
	pub.publish(twist)

def callback1(data1):
    global p1
    global pathx
    global pathy
    x1 = data1.pose.pose.position.x
    y1 = data1.pose.pose.position.y
    quart1 = (data1.pose.pose.orientation.x, data1.pose.pose.orientation.y, data1.pose.pose.orientation.z, data1.pose.pose.orientation.w)
    euler1 = tf.transformations.euler_from_quaternion(quart1)
    print x1
    print y1

    if distanceToPoint(x1,y1,pathx[p1],pathy[p1]+1)<=0.1:

	p1= (p1+1)%6
	print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
	print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
	print "+++++++++++++++++++ VI HAR NATT EN PUNKT ++++++++++++++++++++++"
	print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
	print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
        print "next point:"+str(p1+1)
	rospy.sleep(1.0)

    else:
        #global twist
	#global pub
	tempAngle1 = angleToPoint(x1, y1, pathx[p1], pathy[p1]+1)
	setAngle(euler1[2], tempAngle1, pub1, twist1)
	print "headed towards point: "+str(p1+1)
	#twist.linear.x = 0.1
	pub1.publish(twist1)

def movePath():
    rospy.init_node('oodometry', anonymous=True)
    sub = rospy.Subscriber('RosAria/pose', Odometry, callback)
    sub1 = rospy.Subscriber('RosAria1/pose', Odometry, callback1)
    rospy.spin()

def setStartValues():
    global p
    p = 0
    global p1
    p1 = 3
    global pathx
    a=2
    pathx = (a, a, 0, 0, -a, -a)
    global pathy
    pathy = (0, -a, -a, a, a, 0)
    global twist
    twist = Twist()
    global twist1
    twist1 = Twist()
    global PI
    PI = math.pi
    global angle
    angle = 1
    global pub
    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=1)
    global pub1
    pub1 = rospy.Publisher("RosAria1/cmd_vel", Twist, queue_size=1)

if __name__ == '__main__':
    setStartValues()
    try:
        movePath()
    except rospy.ROSInterruptException:
        pass
