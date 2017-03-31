#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Quaternion
import tf

import roslib;

    
def callback(data):
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s", data.pose.pose.orientation) 
    quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quart)
    print euler[2]

       
def listener():
    rospy.init_node('oodometry', anonymous=True)
    print "hello1"
    rospy.Subscriber('RosAria/pose', Odometry, callback)
    print "hello2"
    rospy.spin()

if __name__ == '__main__':
    listener()
