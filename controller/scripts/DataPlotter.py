#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import types
import sys

class RobotData(object):
    def __init__(self, id):
        self.id = id
        self.sensor_time = np.array()
        self.sensor_data_x = np.array()
        self.sensor_data_y = np.array()
        self.filter_time = np.array()
        self.filter_data_x = np.array()
        self.filter_data_y = np.array()

    def log_sensor(self, data):
        self.sensor_time.append(data.header.stamp)
        self.sensor_data_x.append(data.pose.pose.position.x)
        self.sensor_data_y.append(data.pose.pose.position.y)

    def log_filter(self, data):
        self.filter_time.append(data.header.stamp)
        self.filter_data_x.append(data.pose.pose.position.x)
        self.filter_data_y.append(data.pose.pose.position.y)


if __name__ == '__main__':
    try:
        rospy.init_node('Logger')
        nbr_of_robots = int(sys.argv[1])
        for i in range(0,nbr_of_robots):
            rd = RobotData(i)
            rds[i] = rd
            self.sensor_sub = rospy.Subscriber("Sensor"+str(i)+"/measurement", Odometry, rd.log_sensor)
            self.filter_sub = rospy.Subscriber("Filter"+str(i)+"/state", Odometry, rd.log_filter)

        plt.axis([0, 10, 0, 10])
        plt.ion()

        print "Logger setup done"

        while True:
            for i in range(0,nbr_of_robots):
                plt.scatter(rds[i].sensor_data_x,rds[i].sensor_data_y,c="r")
                plt.scatter(rds[i].filter_data_x,rds[i].filter_data_y,c="g")
            plt.pause(0.1)


    except rospy.ROSInterruptException:
        pass
