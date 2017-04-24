#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import types
import sys

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

    def log_sensor(self, data):
        self.sensor_time = np.append(self.sensor_time,data.header.stamp)
        self.sensor_data_x = np.append(self.sensor_data_x,data.pose.pose.position.x)
        self.sensor_data_y = np.append(self.sensor_data_y,data.pose.pose.position.y)

    def log_filter(self, data):
        self.filter_time = np.append(self.filter_time,data.header.stamp)
        self.filter_data_x = np.append(self.filter_data_x,data.pose.pose.position.x)
        self.filter_data_y = np.append(self.filter_data_y,data.pose.pose.position.y)


if __name__ == '__main__':
    try:
        nbr_of_robots = int(sys.argv[1])
        rospy.init_node('Logger')
        rds = []
        for i in range(0,nbr_of_robots):
            rd = RobotData(i)
            rds.append(rd)


        plt.axis([-5, 5, -5, 5])
        plt.ion()

        print "Logger setup done"

        while True:
            rospy.sleep(5.0)
            for i in range(0,nbr_of_robots):
                print "Robot " + str(i) + " x: " + str(rds[i].sensor_data_x)
                print "Robot " + str(i) + " y: " + str(rds[i].sensor_data_y)
                sensor_min_length = min(np.size(rds[i].sensor_data_x),np.size(rds[i].sensor_data_y))
                filter_min_length = min(np.size(rds[i].filter_data_x),np.size(rds[i].filter_data_y))
                plt.plot(rds[i].sensor_data_x[:sensor_min_length],rds[i].sensor_data_y[:sensor_min_length],c="r")
                plt.plot(rds[i].filter_data_x[:filter_min_length],rds[i].filter_data_y[:filter_min_length],c="g")
            plt.pause(0.1)
            plt.savefig("Path.png")


    except rospy.ROSInterruptException:
        pass
