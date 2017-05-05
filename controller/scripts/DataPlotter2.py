#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
#from matplotlib.legend_handler import HandlerLine2D
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
	self.prediction_sub = rospy.Subscriber("Prediction" + str(id) + "/state", Odometry, self.log_prediction)
        self.sensor_time = np.array([])
        self.sensor_data_x = np.array([])
        self.sensor_data_y = np.array([])
        self.filter_time = np.array([])
        self.filter_data_x = np.array([])
        self.filter_data_y = np.array([])
	self.filter_data_theta = np.array([])
        self.prediction_time = np.array([])
        self.prediction_data_x = np.array([])
        self.prediction_data_y = np.array([])
	self.prediction_data_theta = np.array([])

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

    def log_prediction(self, data):
        self.prediction_time = np.append(self.prediction_time,data.header.stamp)
        self.prediction_data_x = np.append(self.prediction_data_x,data.pose.pose.position.x)
        self.prediction_data_y = np.append(self.prediction_data_y,data.pose.pose.position.y)
	quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quart)
        robot_angle = euler[2]
	self.prediction_data_theta = np.append(self.prediction_data_theta,robot_angle)

if __name__ == '__main__':
    try:
	legendAdded = False
        nbr_of_robots = int(sys.argv[1])
        rospy.init_node('Logger')
        rds = []
        for i in range(0,nbr_of_robots):
	    print i
            rd = RobotData(i)
            rds.append(rd)

	#plt.axis([-10, 10, -10, 10])
        plt.ion()

        print "Logger setup done"

        while True:
            rospy.sleep(3.0)
            for i in range(0,nbr_of_robots):
                #print "Robot " + str(i) + " x: " + str(rds[i].sensor_data_x)
                #print "Robot " + str(i) + " y: " + str(rds[i].sensor_data_y)
                sensor_min_length = min(np.size(rds[i].sensor_data_x),np.size(rds[i].sensor_data_y))
                filter_min_length = min(np.size(rds[i].filter_data_x),np.size(rds[i].filter_data_y))
                prediction_min_length = min(np.size(rds[i].prediction_data_x),np.size(rds[i].prediction_data_y))
		filter_min_length = min(filter_min_length,np.size(rds[i].filter_data_theta))
		#if filter_min_length < 1000:
		#    plt.axis([0, filter_min_length+10, -4, 4])
		#else:
		plt.axis([-6.0, 2.0, -6.0, 2.0])
		plt.xlabel('x [m]')
		plt.ylabel('y [m]')


		#t_filter = range(0,filter_min_length)
		#t_sensor = range(0,filter_min_length)
                #red_dot = plt.plot(rds[i].sensor_data_x[:sensor_min_length],rds[i].sensor_data_y[:sensor_min_length], 'ro',markersize = 5, marker='o', label="Measured state")

                green_dot = plt.plot(rds[i].filter_data_x[:filter_min_length],rds[i].filter_data_y[:filter_min_length],'go', markersize = 5, marker='o', label="Filtered state")

		#blue_dot = plt.plot(rds[i].prediction_data_x[:prediction_min_length],rds[i].prediction_data_y[:prediction_min_length],'bo', markersize = 5, marker = 'o', label="Predicted state")
		if not legendAdded:
		    #plt.legend()
		    legendAdded = True

		#plt.legend(handles = [red_dot, blue_dot, green_dot])
		#plt.plot(t_sensor,rds[i].sensor_data_x[:sensor_min_length],c="r")
		#plt.plot(t_sensor,rds[i].sensor_data_y[:sensor_min_length],c="r")
		#plt.plot(t_filter,rds[i].filter_data_x[:filter_min_length],c="g")
		#plt.plot(t_filter,rds[i].filter_data_y[:filter_min_length],c="g")
		#print str(rds[i].filter_data_theta)
		#endX = 0.5*math.cos(math.radians(rds[i].filter_data_theta[filter_min_length-1]))
		#endY = 0.5*math.sin(math.radians(rds[i].filter_data_theta[filter_min_length-1]))
		#x = rds[i].filter_data_x[filter_min_length-1]
		#y = rds[i].filter_data_y[filter_min_length-1]
		#f, plt.subplot()
		#plt.plot(t_filter,rds[i].filter_data_theta[:filter_min_length],c="y")
            plt.pause(0.1)
            plt.savefig("Path.png")


    except rospy.ROSInterruptException:
        pass
