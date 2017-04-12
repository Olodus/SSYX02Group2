#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy

if __name__ == '__main__':
    rospy.init_node('Logger')

    self.sensor_sub = rospy.Subscriber("Sensor"+str(robot_id)+"/measurement", Odometry, log_sensor)
    self.filter_sub = rospy.Subscriber("Filter"+str(robot_id)+"/state", Odometry, log_filter)

    plt.axis([0, 10, 0, 1])
    plt.ion()

    print "Logger setup done"

    while True:

        plt.pause(0.05)

def log_filter(data):

def log_sensor(data):
