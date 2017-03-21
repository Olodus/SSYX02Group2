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
	global a
	global i
	
	a = 5
	robot0.x = data.pose.pose.position.x
	robot0.y = data.pose.pose.position.y
	quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quart)
	robot0.angle = euler[2]

	robot0.vx = data.twist.twist.linear.x

	if 0.75*a<distanceToPoint(robot0.x,robot0.y,pathx[robot0.nextPoint],pathy[robot0.nextPoint])<=a:
		i = 0
	if 0.50*a<distanceToPoint(robot0.x,robot0.y,pathx[robot0.nextPoint],pathy[robot0.nextPoint])<=0.75*a:
		i = 1
	if 0.2*a<distanceToPoint(robot0.x,robot0.y,pathx[robot0.nextPoint],pathy[robot0.nextPoint])<=0.50*a:
		i = 2
	if distanceToPoint(robot0.x,robot0.y,pathx[robot0.nextPoint],pathy[robot0.nextPoint])<=0.2:
		robot0.nextPoint= (robot0.nextPoint+1)%len(pathx)
		'''
		if (robot0.nextPoint == intersection_point1 or robot0.nextPoint == intersection_point2):
			robot0.in_intersection = True
			if sync_robots:
				robot0.waitMode = True
				print "Waiting: robot0"
		else:
			robot0.in_intersection = False
			print "robot0 not in intersection"'''

	point_angle = angleToPoint(robot0.x, robot0.y, pathx[robot0.nextPoint], pathy[robot0.nextPoint])
	ang = setAngle(robot0.angle, point_angle)
	robot0.twist.angular.z = ang
	if math.fabs(ang) > 0.2:
		robot0.twist.linear.x = 0.0
	else:
		if robot0.acc <= 0.0001 and robot0.acc > -0.0001:
			robot0.twist.linear.x = conf_speed
			'''if distanceToPoint(robot0.x,robot0.y,pathx[robot0.nextPoint],pathy[robot0.nextPoint]) <= 1.0:
				robot0.twist.linear.x = conf_speed
			else:
				robot0.twist.linear.x = robot0.vx'''
		elif robot0.acc >= 0.0:
			if distanceToPoint(robot0.x,robot0.y,pathx[robot0.nextPoint],pathy[robot0.nextPoint]) <= 1.0:
				robot0.twist.linear.x = conf_speed
			else:
				robot0.twist.linear.x = max_speed
		else:
			robot0.twist.linear.x = min_speed

	robot0.publishTwist()
	#rospy.sleep(0.05)

def callback1(data):
	global robot1

	robot1.x = data.pose.pose.position.x
	robot1.y = data.pose.pose.position.y-1.0
	quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quart)
	robot1.angle = euler[2]

	robot1.vx = data.twist.twist.linear.x

	'''
	if distanceToPoint(robot1.x,robot1.y,pathx[robot1.nextPoint],pathy[robot1.nextPoint])<=0.2:
		robot1.nextPoint = (robot1.nextPoint+1)%len(pathx)
		if (robot1.nextPoint == intersection_point1 or robot1.nextPoint == intersection_point2):
			robot1.in_intersection = True
			if sync_robots:
				robot1.waitMode = True
				print "Waiting: robot1"
		else:
			robot1.in_intersection = False
			print "robot1 not in intersection"'''

	point_angle = angleToPoint(robot1.x, robot1.y, pathx[robot1.nextPoint], pathy[robot1.nextPoint])
	ang = setAngle(robot1.angle, point_angle)
	robot1.twist.angular.z = ang
	if math.fabs(ang) > 0.2:
		robot1.twist.linear.x = 0.0
	else:
		if robot1.acc <= 0.0001 and robot1.acc > -0.0001:
			if distanceToPoint(robot1.x,robot1.y,pathx[robot1.nextPoint],pathy[robot1.nextPoint]) <= 1.0:
				robot1.twist.linear.x = conf_speed
			else:
				robot1.twist.linear.x = robot1.vx
		elif robot1.acc >= 0.0:
			if distanceToPoint(robot1.x,robot1.y,pathx[robot1.nextPoint],pathy[robot1.nextPoint]) <= 1.0:
				robot1.twist.linear.x = conf_speed
			else:
				robot1.twist.linear.x = max_speed
		else:
			robot1.twist.linear.x = min_speed

	robot1.publishTwist()
	#rospy.sleep(0.05)

def calc_t(x1,y1,v,x2,y2):
	dist = distanceToPoint(x1,y1,x2,y2)
	t = dist/v
	return t

def calc_pos(time,x,y,angle,v,a):
	x_new = a*math.pow(time,2)*math.cos(angle)/2 + v*t*math.cos(angle) + x
	y_new = a*math.pow(time,2)*math.sin(angle)/2 + v*t*math.sin(angle) + y
	pos = [x_new,y_new]
	return pos

def is_inside_collisionbox(x,y):
	if x <= c_box_w/2 and x >= -c_box_w/2:
		if y <= c_box_l/2 and y >= -c_box_l/2:
			return True

	return False


def intersection_point(x1,y1,theta1,x2,y2,theta2):
	IP_x = (y2-y1-math.atan(theta2)*x2+math.atan(theta1)*x1)/(math.atan(theta1)-math.atan(theta2))
	IP_y = math.atan(theta1)*IP_x+y1-math.atan(theta1)*x1
	IP = [IP_x, IP_y]
	return IP

# Simple function for avoiding collision
# Robot0 always sets to max speed and the other chooses a acc to avoid
def simple_obj_func():
	global once
	robot = [robot0.acc, robot1.acc]
	# Set robot0 acc
	if not once:
		robot = [0.0, 0.0]
	if robot1.nextPoint == intersection_point1 and once:
			if robot1.x > 0 or robot0.y > 0:
				print "Have passed intersection"
				robot[1] = max_acc
				robot[0] = max_acc
	elif robot1.nextPoint == intersection_point2 and once:
			if robot1.y > 0 or robot0.x > 0:
				print "Have passed intersection"
				robot[1] = max_acc
				robot[0] = max_acc
	if robot0.vx >= conf_speed and not once:
		robot[0] = 0.0
		print "r0 velocity inside loop: "+str(robot0.vx)
		print "Robot0 has constant velocity, robot1 changes velocity"
		once = True
		# Set robot1 acc
		IP = intersection_point(robot0.x, robot0.y, robot0.angle, robot1.x, robot1.y, robot1.angle)
		IP_x = IP[0]
		IP_y = IP[1]
		print "intersection x-point: "+str(IP[0])
		print "intersection y-point: "+str(IP[1])
		t = calc_t(robot0.x,robot0.y,robot0.vx,IP_x,IP_y)
		t = t - 1.0
		dist = distanceToPoint(robot1.x,robot1.y,IP_x,IP_y)
		# Is it possible for robot1 to cover the distance? If not slow it down instead
		if max_speed*t < dist:
			t = t + 2.0
		robot[1] = -(2.0*robot1.vx)/t+2.0*dist/(math.pow(t,2))
	elif not once:
		robot[0] = 0.2
		robot[1] = 0.2
		#robot0.vx = conf_speed
		#robot1.vx = conf_speed
	return robot

def both_in_intersection(client0, client1):
	print "Both in intersection"
	#acc = simple_obj_func()
	acc = [0.05, 1.0]
	client0.update_configuration({"trans_decel":math.fabs(acc[0]), "trans_accel":math.fabs(acc[0])})
	robot0.acc = acc[0]
	client1.update_configuration({"trans_decel":math.fabs(acc[1]), "trans_accel":math.fabs(acc[1])})
	robot1.acc = acc[1]
	print "acc r0: " + str(robot0.acc)
	print "acc r1: " + str(robot1.acc)
	print "vx r0: " + str(robot0.vx)
	print "vx r1: " + str(robot1.vx)

def outside_intersection(client0, client1):
	client0.update_configuration({"trans_decel":max_acc, "trans_accel":max_acc})
	robot0.acc = max_acc
	client1.update_configuration({"trans_decel":max_acc, "trans_accel":max_acc})
	robot1.acc = max_acc
	global once
	once = False

def run_controller():
	rospy.init_node('oodometry', anonymous=True)
	sub0 = rospy.Subscriber('RosAria0/pose', Odometry, callback0)
	#sub1 = rospy.Subscriber('RosAria1/pose', Odometry, callback1)
	client0 = dynamic_reconfigure.client.Client("RosAria0", timeout=10)
	#client1 = dynamic_reconfigure.client.Client("RosAria1", timeout=10)
	global i
	
	while True:
		acc = [0.05, 1.5, -1.5]
		client0.update_configuration({"trans_decel":math.fabs(acc[0]), "trans_accel":math.fabs(acc[0])})
		robot0.acc = acc[i]
		print "acc set: "+str(acc[i])
	'''while True:
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

		#rospy.sleep(0.05)'''

def setStartValues():
	global robot0
	robot0 = Robot(0,1,0,0)
	global robot1
	robot1 = Robot(1,4,0,1)

	global a
	a = 5
	global pathx
	#pathx = (a, a, 0, 0, -a, -a)
	pathx = (a, 0, a, 0, a, 0)
	global pathy
	#pathy = (0, -a, -a, a, a, 0)
	pathy = (0, 0, 0, 0, 0, 0)
	global intersection_point1
	global intersection_point2
	intersection_point1 = 0
	intersection_point2 = 3

	global PI
	PI = math.pi

	global sync_robots
	sync_robots = True

	global max_speed
	max_speed = 1.0
	global min_speed
	min_speed = 0.1
	global max_acc
	max_acc = 1.0
	global min_acc
	min_acc = -1.0
	global conf_speed
	conf_speed = 0.2
	global c_box_w
	c_box_w = 1.0
	global c_box_l
	c_box_l = 1.0

	global once
	once = False
	global i
	i = 0


if __name__ == '__main__':
	setStartValues()
	try:
		run_controller()
	except rospy.ROSInterruptException:
		pass
