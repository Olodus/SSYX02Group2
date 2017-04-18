#!/usr/bin/env python

from nav_msgs.msg import Odometry
import sys
import rospy
import csv
import time
import numpy as np
from rospy.numpy_msg import numpy_msg
from robotclient.msg import Floats
from robotclient.srv import *

'''
This is a Handler which takes the position from UWB and the angle from Pose
'''

class UWBPoseHandler(object):

    def __init__(self, robot_id):
        rospy.init_node('Sensor'+str(robot_id))
        srv = 'get_coord' + str(robot_id)
        rospy.wait_for_service(srv)
        self.get_coords = rospy.ServiceProxy(srv, GetCoord)
        self.measurement = Odometry()
        self.pub = rospy.Publisher("Sensor"+str(robot_id)+"/measurement", Odometry, queue_size=1)
        self.sub = rospy.Subscriber("RosAria"+str(robot_id)+"/pose", Odometry, self.measure)


    def measure(self, data):
        self.measurement = Odometry()

        # Take velocity from pose
        self.measurement.twist = data.twist

        # The rest is taken from UWB   as of now takes a mean of 10 measurements
        try:
            f = Floats()
            f = self.get_coords(1)
            # transform to Odometry
            x = f.data.data[0]
            y = f.data.data[1]
            self.measurement.pose.pose.position.x = x
            self.measurement.pose.pose.position.y = y
        except rospy.rospy.ServiceException:
            print "GetCoord service not responding"

        self.pub.publish(self.measurement)

if __name__ == '__main__':
    try:
        u = UWBPoseHandler(int(sys.argv[1]))
	print "Sensor setup done"
	rospy.spin()

    except rospy.ROSInterruptException:
        pass
