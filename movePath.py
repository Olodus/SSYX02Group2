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

class Robot(object):
	def __init__(self, idNumber, startPoint, startX, startY):
		self.idNumber = idNumber
		self.nextPoint = startPoint
		self.twist = Twist()
		self.pub = rospy.Publisher("RosAria"+str(idNumber)+"/cmd_vel", Twist, queue_size=1)
		self.offsetX = startX
		self.offsetY = startY
		self.x = startX
		self.y = startY
		self.waitMode = False

	def publishTwist(self):
		if not waitMode:
			self.pub.publish(self.twist)
		else:
			self.twist.linear.x = 0.0
			self.pub.publish(self.twist)



def setAngle(theta_curr,theta):
    if math.fabs(theta_curr-theta)>10*PI/180:
		if theta_curr < theta:
			if math.fabs(theta_curr - theta)<PI:
				return 0.3
			else:
				return -0.3
		else:
			if math.fabs(theta_curr - theta)<PI:
				return -0.3
			else:
				return 0.3

    elif 3*PI/180<=math.fabs(theta_curr-theta) and math.fabs(theta_curr-theta)<=8*PI/180:
		if theta_curr < theta:
			if math.fabs(theta_curr - theta)<PI:
				return = 0.1
			else:
				return -0.1
		else:
			if math.fabs(theta_curr - theta)<PI:
				return -0.1
			else:
				return 0.1

    else:
		return 0.0

def distanceToPoint(x,y,pointx,pointy):
    dist = math.sqrt(math.pow(pointx-x,2)+math.pow(pointy-y,2))
    return dist

def angleToPoint(x,y,pointx,pointy):
    angle = math.atan2(pointy-y,pointx-x)
    return angle

def callback0(data):
	global robot0

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quart)
	robot_angle = euler[2]

	if distanceToPoint(x,y,pathx[robot0.nextPoint],pathy[robot0.nextPoint])<=0.2:
		robot0.nextPoint= (robot0.nextPoint+1)%len(pathx)
		if (robot0.nextPoint == intersection_point1 or robot0.nextPoint == intersection_point2) and synch_robots:
			robot0.waitMode = True

	point_angle = angleToPoint(x, y, pathx[robot0.nextPoint], pathy[robot0.nextPoint])
	ang = setAngle(robot_angle, point_angle)
	robot0.twist.angular.z = ang
	if math.fabs(ang) > 0.2:
		robot0.twist.linear.x = 0.0
	else:
		robot0.twist.linear.x = 0.3

	robot0.publishTwist()
	rospy.sleep(0.1)

def callback1(data):
	global robot1

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quart)
	robot_angle = euler[2]

	if distanceToPoint(x,y,pathx[robot1.nextPoint]+robot1.offsetX,pathy[robot1.nextPoint]+robot1.offsetY)<=0.2:
		robot1.nextPoint = (robot1.nextPoint+1)%len(pathx)
		if (robot1.nextPoint == intersection_point1 or robot1.nextPoint == intersection_point2) and synch_robots:
			robot1.waitMode = True

	point_angle = angleToPoint(x, y, pathx[robot1.nextPoint]+robot1.offsetX, pathy[robot1.nextPoint]+robot1.offsetY)
	ang = setAngle(robot_angle, point_angle)
	robot1.twist.angular.z = ang
	if math.fabs(ang) > 0.2:
		robot1.twist.linear.x = 0.0
	else:
		robot1.twist.linear.x = 0.3

	robot1.publishTwist()
	rospy.sleep(0.1)

def movePath():
    rospy.init_node('oodometry', anonymous=True)
    sub = rospy.Subscriber('RosAria0/pose', Odometry, callback0)
    sub1 = rospy.Subscriber('RosAria1/pose', Odometry, callback1)
    rospy.spin()

def setStartValues():
	global robot0
	robot0 = Robot(0,0,0,0)
	global robot1
	robot1 = Robot(1,4,0,1)

	global pathx
	a=2
	pathx = (a, a, 0, 0, -a, -a)
	global pathy
	pathy = (0, -a, -a, a, a, 0)
	global intersection_point1
	global intersection_point2
	intersection_point1 = 1
	intersection_point2 = 3

	global PI
	PI = math.pi

	global synch_robots
	synch_robots = True


if __name__ == '__main__':
    setStartValues()
    try:
        movePath()
    except rospy.ROSInterruptException:
        pass
