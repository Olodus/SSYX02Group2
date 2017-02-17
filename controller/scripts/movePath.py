#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Quaternion
import tf

import roslib;


# rospy.loginfo(rospy.get_caller_id()+"I heard %s", data.pose.pose.position.x) 


pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=1)
twist = Twist()
PI=3.14159265358
p = 0
pathx = p *(1, 1, 0, 0, -1, -1)
pathy = p *(0, -1, -1, 1, 1, 0)

def setAngle(x,y,theta):

    if theta<PI/2*0.9:

        twist.linear.x = 0
    	twist.angular.z = 0.2
    	pub.publish(twist)
    	rospy.sleep(0.01)

    elif theta< PI/2: #loopa sa den fortsatter i en cirke
        
 
    	twist.linear.x = 0
    	twist.angular.z = 0.01
    	pub.publish(twist)
    	rospy.sleep(0.01)


    
    else: #loopa sa den fortsatter i en cirkel
	twist.angular.z=0
	pub.publish(twist)
	rospy.sleep(0.01)

def range(x,y,pointx,pointy):
    math.sqrt(math.pow(pointx-x,2)+math.pow(pointy-y,2))

def angleToPoint

def callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quart)

    if range(x,y,pathx[p],pathy[p])<=0.1:
	p++
    else
	setAngle(data.pose.pose.position.x, data.pose.pose.position.y, euler[2])
	

		


def movePath():
    angle = 1
    rospy.init_node('oodometry', anonymous=True)
    sub = rospy.Subscriber('RosAria/pose', Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    movePath()
