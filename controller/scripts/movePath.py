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

p = 0
pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=1)
twist = Twist()
PI=3.14159265358

pathx = (1, 1, 0, 0, -1, -1)
pathy = (0, -1, -1, 1, 1, 0)

def setAngle(theta_r,theta):

    if math.fabs(theta_r-theta)>0.2:
    	twist.angular.z = -0.2*math.copysign(1,theta_r-theta)
    	pub.publish(twist)
    	rospy.sleep(0.01)

    elif 0.05<=math.fabs(theta_r-theta)<=0.2: #loopa sa den fortsatter i en cirke
    	twist.angular.z = -0.2*math.copysign(1,theta_r-theta)
    	pub.publish(twist)
    	rospy.sleep(0.01)
    
    else: #loopa sa den fortsatter i en cirkel
	twist.angular.z=0
	pub.publish(twist)
	rospy.sleep(0.01)

def distanceToPoint(x,y,pointx,pointy):
    dist = math.sqrt(math.pow(pointx-x,2)+math.pow(pointy-y,2))
    return dist

def angleToPoint(x,y,pointx,pointy):
    print x
    print y
    print pointx 
    print pointy
    angle = math.atan((pointy-y)/(pointx-x))
    print(angle*180/PI)
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
        
        print "p = " 
        print p

    else:
        global twist
	global pub
	tempAngle = angleToPoint(x, y, pathx[p], pathy[p])
	setAngle(euler[2], tempAngle)
	twist.linear.x = 0.2
	pub.publish(twist)

def movePath():
    rospy.init_node('oodometry', anonymous=True)
    sub = rospy.Subscriber('RosAria/pose', Odometry, callback)
    rospy.spin()

def setStartValues():
    global p
    p = 0
    global pathx
    pathx = (1, 1, 0, 0, -1, -1)
    global pathy 
    pathy = (0, -1, -1, 1, 1, 0)
    global twist
    twist = Twist()
    global angle
    angle = 1
    global pub
    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=1)

if __name__ == '__main__':
    setStartValues()
    try:
        movePath()
    except rospy.ROSInterruptException:
        pass
