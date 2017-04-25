#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import types
import sys
from nav_msgs.msg import Odometry
import tf
import math

class RobotData(object):
    def __init__(self, id):
        self.id = id
        self.sensor_sub = rospy.Subscriber("Sensor/measurement" + str(id), Odometry, self.log_sensor)
        self.filter_sub = rospy.Subscriber("Filter" + str(id) + "/state", Odometry, self.log_filter)
        self.sensor_time = np.array([])
        self.sensor_data_x = np.array([])
        self.sensor_data_y = np.array([])
        self.filter_time = np.array([])
        self.filter_data_x = np.array([])
        self.filter_data_y = np.array([])
	self.filter_data_theta = np.array([])

    def log_sensor(self, data):
        self.sensor_time = np.append(self.sensor_time,data.header.stamp)
        self.sensor_data_x = np.append(self.sensor_data_x,data.pose.pose.position.x)
        self.sensor_data_y = np.append(self.sensor_data_y,data.pose.pose.position.y)

    def log_filter(self, data):
        self.filter_time = np.append(self.filter_time,data.header.stamp)
        self.filter_data_x = np.append(self.filter_data_x,data.pose.pose.position.x)
        self.filter_data_y = np.append(self.filter_data_y,data.pose.pose.position.y)
	quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quart)
        robot_angle = euler[2]
	self.filter_data_theta = np.append(self.filter_data_theta,robot_angle)


if __name__ == '__main__':
    try:
        nbr_of_robots = int(sys.argv[1])
        rospy.init_node('Logger')
        rds = []
        for i in range(0,nbr_of_robots):
            rd = RobotData(i)
            rds.append(rd)


        plt.axis([-10, 10, -10, 10])
        plt.ion()

        print "Logger setup done"

        while True:
            rospy.sleep(5.0)
            for i in range(0,nbr_of_robots):
                #print "Robot " + str(i) + " x: " + str(rds[i].sensor_data_x)
                #print "Robot " + str(i) + " y: " + str(rds[i].sensor_data_y)
                sensor_min_length = min(np.size(rds[i].sensor_data_x),np.size(rds[i].sensor_data_y))
                filter_min_length = min(np.size(rds[i].filter_data_x),np.size(rds[i].filter_data_y))
		filter_min_length = min(filter_min_length,np.size(rds[i].filter_data_theta))
                plt.plot(rds[i].sensor_data_x[:sensor_min_length],rds[i].sensor_data_y[:sensor_min_length],c="r")
                plt.plot(rds[i].filter_data_x[:filter_min_length],rds[i].filter_data_y[:filter_min_length],c="g")
		print str(rds[i].filter_data_theta[filter_min_length-1])
		#endX = 0.5*math.cos(math.radians(rds[i].filter_data_theta[filter_min_length-1]))
		#endY = 0.5*math.sin(math.radians(rds[i].filter_data_theta[filter_min_length-1]))
		#x = rds[i].filter_data_x[filter_min_length-1]
		#y = rds[i].filter_data_y[filter_min_length-1]
		#plt.plot([x,x+endX],[y,y+endY],color="b")
            plt.pause(0.1)
            plt.savefig("Path.png")


    except rospy.ROSInterruptException:
        pass
