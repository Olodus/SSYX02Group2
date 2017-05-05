#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
#from matplotlib.legend_handler import HandlerLine2D
import rospy
import types
import sys
from nav_msgs.msg import Odometry
from std_msgs.msg import *
import tf
import math
import csv

class RobotData(object):
    def __init__(self, id):
        self.id = id
        self.sensor_sub = rospy.Subscriber("Sensor/measurement" + str(id), Odometry, self.log_sensor)
        self.filter_sub = rospy.Subscriber("Filter" + str(id) + "/state", Odometry, self.log_filter)
        self.prediction_sub = rospy.Subscriber("Prediction" + str(id) + "/state", Odometry, self.log_prediction)
        self.sensor_time = np.array([])
        self.sensor_data_x = np.array([])
        self.sensor_data_y = np.array([])
        self.sensor_data_v = np.array([])
        self.filter_time = np.array([])
        self.filter_data_x = np.array([])
        self.filter_data_y = np.array([])
        self.filter_data_theta = np.array([])
        self.filter_data_v = np.array([])
        self.prediction_time = np.array([])
        self.prediction_data_x = np.array([])
        self.prediction_data_y = np.array([])
        self.prediction_data_theta = np.array([])

    def log_sensor(self, data):
        global logging
        if logging:
            self.sensor_time = np.append(self.sensor_time,rospy.get_time())
            self.sensor_data_x = np.append(self.sensor_data_x,data.pose.pose.position.x)
            self.sensor_data_y = np.append(self.sensor_data_y,data.pose.pose.position.y)
            self.sensor_data_v = np.append(self.sensor_data_y,data.twist.twist.linear.x)

    def log_filter(self, data):
        global logging
        if logging:
            self.filter_time = np.append(self.filter_time,rospy.get_time())
            self.filter_data_x = np.append(self.filter_data_x,data.pose.pose.position.x)
            self.filter_data_y = np.append(self.filter_data_y,data.pose.pose.position.y)
            self.filter_data_v = np.append(self.filter_data_y,data.twist.twist.linear.x)
            quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quart)
            robot_angle = euler[2]
            self.filter_data_theta = np.append(self.filter_data_theta,robot_angle)

    def log_prediction(self, data):
        global logging
        if logging:
            self.prediction_time = np.append(self.prediction_time,rospy.get_time())
            self.prediction_data_x = np.append(self.prediction_data_x,data.pose.pose.position.x)
            self.prediction_data_y = np.append(self.prediction_data_y,data.pose.pose.position.y)
            quart = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quart)
            robot_angle = euler[2]
            self.prediction_data_theta = np.append(self.prediction_data_theta,robot_angle)

def start_log(data):
    global logging
    logging = True
    print "Started logging"

def stop_log(data):
    global logging
    logging = False
    print "Stopped logging"

def save_log():
    print "Saving log"
    for i in range(0, nbr_of_robots):
        with open("~/measurements_robot"+str(i)+"_sensor.csv", "wb") as f:
            writer = csv.writer(f)
            writer.writerow(rds[i].sensor_time)
            writer.writerow(rds[i].sensor_data_x)
            writer.writerow(rds[i].sensor_data_y)
            writer.writerow(rds[i].sensor_data_v)
        with open("~/measurements_robot"+str(i)+"_filter.csv", "wb") as f:
            writer = csv.writer(f)
            writer.writerow(rds[i].filter_time)
            writer.writerow(rds[i].filter_data_x)
            writer.writerow(rds[i].filter_data_y)
            writer.writerow(rds[i].filter_data_v)

def reset_log(data):
    global rds
    global nbr_of_robots
    global logging
    # Cant start new if you are already logging
    if not logging:
        rds = []
        for i in range(0, nbr_of_robots):
            print i
            rd = RobotData(i)
            rds.append(rd)

def log_control_action(data):
    global control_action_data
    global control_action_time
    control_action_data = np.append(control_action_data, data)
    control_action_time = np.append(control_action_time, rospy.get_time())

def plot_xy(req):
    # plt.axis([-10, 10, -10, 10])
    plt.ion()

    for i in range(0, nbr_of_robots):
        sensor_min_length = min(np.size(rds[i].sensor_data_x), np.size(rds[i].sensor_data_y))
        filter_min_length = min(np.size(rds[i].filter_data_x), np.size(rds[i].filter_data_y))
        prediction_min_length = min(np.size(rds[i].prediction_data_x), np.size(rds[i].prediction_data_y))
        filter_min_length = min(filter_min_length, np.size(rds[i].filter_data_theta))
        # if filter_min_length < 1000:
        #    plt.axis([0, filter_min_length+10, -4, 4])
        # else:
        plt.axis([-6.0, 2.0, -6.0, 2.0])
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')

        # t_filter = range(0,filter_min_length)
        # t_sensor = range(0,filter_min_length)
        red_dot = plt.plot(rds[i].sensor_data_x[:sensor_min_length], rds[i].sensor_data_y[:sensor_min_length], 'ro',
                           markersize=5, marker='o', label="Measured state")

        green_dot = plt.plot(rds[i].filter_data_x[:filter_min_length], rds[i].filter_data_y[:filter_min_length], 'go',
                             markersize=5, marker='o', label="Filtered state")

        blue_dot = plt.plot(rds[i].prediction_data_x[:prediction_min_length],
                            rds[i].prediction_data_y[:prediction_min_length], 'bo', markersize=5, marker='o',
                            label="Predicted state")

    plt.pause(0.1)
    plt.savefig("plot_xy.png")

def plot_dist_to_mid(req):
    # plt.axis([-10, 10, -10, 10])
    plt.ion()

    for i in range(0, nbr_of_robots):
        filter_min_length = min(np.size(rds[i].filter_data_x), np.size(rds[i].filter_data_y))
        filter_min_length = min(filter_min_length, np.size(rds[i].filter_time))
        start_x = rds[i].filter_data_x[0]
        start_y = rds[i].filter_data_y[0]
        length_from_start = np.array([])

        for j in range(0, filter_min_length):
            length_from_start = np.append(math.sqrt((rds[i].filter_data_x[j]-start_x) ** 2 + (rds[i].filter_data_y[j]-start_y) ** 2))

        plt.axis([-6.0, 2.0, -6.0, 2.0])
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')

        if i == 0:
            dot = plt.plot(rds[i].filter_time[:filter_min_length], length_from_start[:filter_min_length], 'go',
                             markersize=5, marker='o', label="Robot 0")
        else:
            dot = plt.plot(rds[i].filter_time[:filter_min_length], length_from_start[:filter_min_length], 'bo',
                             markersize=5, marker='o', label="Robot 1")


    plt.pause(0.1)
    plt.savefig("plot_dist_to_mid.png")


if __name__ == '__main__':
    try:
        global control_action_data
        global control_action_time
        control_action_data = np.array([])
        control_action_time = np.array([])
        global nbr_of_robots
        nbr_of_robots = int(sys.argv[1])
        rospy.init_node('Logger')
        global rds
        rds = []
        for i in range(0,nbr_of_robots):
            print i
            rd = RobotData(i)
            rds.append(rd)

        start_sub = rospy.Subscriber("Logger/start_log", Empty, start_log)
        stop_sub = rospy.Subscriber("Logger/stop_log", Empty, stop_log)
        reset_sub = rospy.Subscriber("Logger/reset_log", Empty, reset_log)
        save_sub = rospy.Subscriber("Logger/save_log", Empty, save_log)
        control_action_sub = rospy.Subscriber("Logger/control_log", string, log_control_action)

        plot_xy_sub = rospy.Subscriber("Logger/plot_xy", Empty, plot_xy)

        plot_dist_sub = rospy.Subscriber("Logger/plot_dist_to_mid", Empty, plot_dist_to_mid)

        print "Logger setup done"
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
