#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Quaternion
import tf
import math
import numpy as np
import dynamic_reconfigure.client

import roslib;

class Robot(object):
	def __init__(self, idNumber, startPoint, startX, startY):
		self.idNumber = idNumber
		self.nextPoint = startPoint
		self.twist = Twist()
		self.pub = rospy.Publisher("RosAria"+str(idNumber)+"/cmd_vel", Twist, queue_size=1)
		self.x = startX
		self.y = startY
		self.waitMode = False
		self.in_intersection = False
		self.acc = 0.0
		self.width = 0.5
		self.length = 0.7
		self.vx = 0.0
		self.angle = 0.0

	def publishTwist(self):
		if not self.waitMode:
			self.pub.publish(self.twist)
		else:
			self.twist.linear.x = 0.0
			self.pub.publish(self.twist)



def setAngle(theta_curr,theta):
    if math.fabs(theta_curr-theta)>20*PI/180:
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

    elif 3*PI/180<=math.fabs(theta_curr-theta) and math.fabs(theta_curr-theta)<=20*PI/180:
		if theta_curr < theta:
			if math.fabs(theta_curr - theta)<PI:
				return 0.05
			else:
				return -0.05
		else:
			if math.fabs(theta_curr - theta)<PI:
				return -0.05
			else:
				return 0.05

    else:
		return 0.0

def distanceToPoint(x,y,pointx,pointy):
	return math.sqrt(math.pow(pointx-x,2)+math.pow(pointy-y,2))

def angleToPoint(x,y,pointx,pointy):
	return math.atan2(pointy-y,pointx-x)

def callback0(data):
	global robot0

	robot0.x = data.pose.pose.position.x
	robot0.y = data.pose.pose.position.y
	quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quart)
	robot0.angle = euler[2]

	robot1.vx = data.twist.twist.linear.x

	if distanceToPoint(robot0.x,robot0.y,pathx[robot0.nextPoint],pathy[robot0.nextPoint])<=0.2:
		robot0.nextPoint= (robot0.nextPoint+1)%len(pathx)
		if (robot0.nextPoint == intersection_point1 or robot0.nextPoint == intersection_point2):
			robot0.in_intersection = True
			if sync_robots:
				robot0.waitMode = True
				print "Waiting: robot0"
		else:
			robot0.in_intersection = False
			print "robot0 not in intersection"

	point_angle = angleToPoint(robot0.x, robot0.y, pathx[robot0.nextPoint], pathy[robot0.nextPoint])
	ang = setAngle(robot_angle, point_angle)
	robot0.twist.angular.z = ang
	if math.fabs(ang) > 0.2:
		robot0.twist.linear.x = 0.0
	else:
		if robot0.acc >= 0.0:
			robot0.twist.linear.x = max_speed
		else:
			robot0.twist.linear.x = min_speed

	robot0.publishTwist()
	rospy.sleep(0.1)

def callback1(data):
	global robot1

	robot1.x = data.pose.pose.position.x
	robot1.y = data.pose.pose.position.y+1.0
	quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quart)
	robot1.angle = euler[2]

	robot1.vx = data.twist.twist.linear.x

	if distanceToPoint(robot1.x,robot1.y,pathx[robot1.nextPoint],pathy[robot1.nextPoint])<=0.2:
		robot1.nextPoint = (robot1.nextPoint+1)%len(pathx)
		if (robot1.nextPoint == intersection_point1 or robot1.nextPoint == intersection_point2):
			robot1.in_intersection = True
			if sync_robots:
				robot1.waitMode = True
				print "Waiting: robot1"
		else:
			robot1.in_intersection = False

	point_angle = angleToPoint(robot1.x, robot1.y, pathx[robot1.nextPoint], pathy[robot1.nextPoint])
	ang = setAngle(robot_angle, point_angle)
	robot1.twist.angular.z = ang
	if math.fabs(ang) > 0.2:
		robot1.twist.linear.x = 0.0
	else:
		if robot1.acc >= 0.0:
			robot1.twist.linear.x = max_speed
		else:
			robot1.twist.linear.x = min_speed

	robot1.publishTwist()
	rospy.sleep(0.1)

def calc_t(x,y,angle,v,a):
	return nil

def calc_pos(time,x,y,angle,v,a):
	return nil

def is_inside_collisionbox(x,y):
	if x0+robot0.length/2 < x1+robot1.width/2:
		return True

# Simple function for avoiding collision
# Robot0 always sets to max speed and the other chooses a acc to avoid
def simple_obj_func():
	robot = [0.0, 0.0]

	# Set robot0 acc
	if robot0.vx >= max_speed:
		robot[0] = 0.0
	else:
		robot[0] = max_acc

	t = calc_t(robot0.x,robot0.y,robot0.angle,robot0.vx,robot[0])
	# Set robot1 acc
	robot[1] = max_acc
	pos = calc_pos(t,robot1.x,robot1.y,robot1.vx,robot[1])
	x = pos.x
	y = pos.y
	while not is_inside_collisionbox(x,y):
		robot[1] = robot[1]-0.1
		pos = calc_pos(t,robot1.x,robot1.y,robot1.vx,robot[1])
		x = pos.x
		y = pos.y

	return robot


def both_in_intersection(client0, client1):
	print "Both in intersection"
	client0.update_configuration({"trans_decel":1.0})
	robot0.acc = -1.0
	client1.update_configuration({"trans_accel":1.0})
	robot1.acc = 1.0

def outside_intersection(client0, client1):
	print "No chance of crash, hopefully..."
	client0.update_configuration({"trans_accel":1.0})
	robot0.acc = 1.0
	client1.update_configuration({"trans_accel":1.0})
	robot1.acc = 1.0

def run_controller():
	rospy.init_node('oodometry', anonymous=True)
	sub0 = rospy.Subscriber('RosAria0/pose', Odometry, callback0)
	sub1 = rospy.Subscriber('RosAria1/pose', Odometry, callback1)
	client0 = dynamic_reconfigure.client.Client("RosAria0", timeout=10)
	client1 = dynamic_reconfigure.client.Client("RosAria1", timeout=10)
	while True:
		if sync_robots:
			if robot0.waitMode and robot1.waitMode and math.fabs(robot0.twist.angular.z) < 0.05 and math.fabs(robot1.twist.angular.z) < 0.05:
				robot0.waitMode = False
				robot1.waitMode = False
			elif robot0.in_intersection and robot1.in_intersection:
				both_in_intersection(client0, client1)
			else:
				outside_intersection(client0, client1)
		elif robot0.in_intersection and robot1.in_intersection:
			both_in_intersection(client0, client1)
		else:
			outside_intersection(client0, client1)

		rospy.sleep(1.0)

def setStartValues():
	global robot0
	robot0 = Robot(0,1,0,0)
	global robot1
	robot1 = Robot(1,4,0,1)

	global pathx
	a=2
	pathx = (a, a, 0, 0, -a, -a)
	global pathy
	pathy = (0, -a, -a, a, a, 0)
	global intersection_point1
	global intersection_point2
	intersection_point1 = 0
	intersection_point2 = 3

	global PI
	PI = math.pi

	global sync_robots
	sync_robots = True

	global max_speed
	max_speed = 0.5
	global min_speed
	min_speed = 0.0
	global max_acc
	max_acc = 1.0
	global min_acc
	min_acc = -1.0


if __name__ == '__main__':
	setStartValues()
	try:
		run_controller()
	except rospy.ROSInterruptException:
		pass
