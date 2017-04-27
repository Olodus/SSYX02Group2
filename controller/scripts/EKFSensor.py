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
import matplotlib.pyplot as plt

'''
This is a Handler which takes the position from UWB and the angle from Pose
'''

class EKFHandler(object):

    def __init__(self, robot_id):
        self.robot_id = robot_id
        srv = 'get_coord' + str(robot_id)
        rospy.wait_for_service(srv)
        self.get_coords = rospy.ServiceProxy(srv, GetCoord)
        self.measurement = Odometry()
        self.pose_pub = rospy.Publisher("robot_pose_ekf/odom", Odometry, queue_size=1)
        self.uwb_pub = rospy.Publisher("robot_pose_ekf/vo", Odometry, queue_size=1)
        self.sub = rospy.Subscriber("RosAria"+str(robot_id)+"/pose", Odometry, self.update)
        self.covariance_measured = False

    def update(self,data):
        if self.covariance_measured:
            self.pose_pub.publish(data)

        try:
            f = Floats()
            tmp_pos = []
            while np.size(tmp_pos) != 2:
                f = self.get_coords(1)
                tmp_pos = f.data.data
                x = tmp_pos[0]
                y = tmp_pos[1]
            self.measurement.pose.pose.position.x = x
            self.measurement.pose.pose.position.y = y
            self.measurement.pose.pose.position.z = 0
            self.measurement.pose.pose.orientation.x = 1
            self.measurement.pose.pose.orientation.y = 0
            self.measurement.pose.pose.orientation.z = 0
            self.measurement.pose.pose.orientation.w = 0
        except rospy.ServiceException:
            print "GetCoord service not responding"

        self.measurement.pose.covariance = {self.covx, 0, 0, 0, 0, 0,
                                            0, self.covy, 0, 0, 0, 0,
                                            0, 0, 99999, 0, 0, 0,
                                            0, 0, 0, 99999, 0, 0,
                                            0, 0, 0, 0, 99999, 0,
                                            0, 0, 0, 0, 0, 99999}

        if self.covariance_measured:
            self.uwb_pub.publish(self.measurement)

    def measure_cov(self):
        n = 100
        xarr = np.array([], dtype=np.float32)
        yarr = np.array([], dtype=np.float32)
        i = 0
        error_count = 0
        while i < n and error_count < 100:
            tmp_pos = np.array([], dtype=np.float32)
            try:
                f = Floats()
                tmp_pos = []
                while np.size(tmp_pos) != 2:
                    f = self.get_coords(1)
                    tmp_pos = f.data.data
                    xarr = np.append(xarr, tmp_pos[0])
                    yarr = np.append(yarr, tmp_pos[1])
                    i += 1
                else:
                    error_count += 1
                    print("Invalid reading, check if all the unicorns are at home")
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

        self.cov = np.cov(xarr,yarr)
        self.covx = np.cov(xarr)
        self.covy = np.cov(yarr)
        plt.plot(xarr,yarr)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.savefig("Cov_R"+str(self.robot_id)+".png")

if __name__ == '__main__':
    try:
        rospy.init_node('Sensor')

        e0 = EKFHandler(0)
        e1 = EKFHandler(1)
        e0.measure_cov()
        e1.measure_cov()
        e0.covariance_measured = True
        e1.covariance_measured = True
        print "Sensor setup done"

        while True:
            e0.measure()
            rospy.sleep(0.05)
            e1.measure()
            rospy.sleep(0.05)


    except rospy.ROSInterruptException:
        pass
