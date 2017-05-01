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

class UWBPoseHandler(object):

    def __init__(self, robot_id):
        srv = 'get_coord' + str(robot_id)
        rospy.wait_for_service(srv)
        self.get_coords = rospy.ServiceProxy(srv, GetCoord)
        self.measurement = Odometry()
        self.pub = rospy.Publisher("Sensor/measurement"+str(robot_id), Odometry, queue_size=1)
        self.sub = rospy.Subscriber("RosAria"+str(robot_id)+"/pose", Odometry, self.update)
        self.state = Odometry()
        self.covariance_sent = False

    def update(self,data):
        self.state = data

    def measure(self):
        if self.covariance_sent:
            self.measurement = Odometry()

            # Take velocity from pose
            self.measurement.twist = self.state.twist

            # The rest is taken from UWB   as of now takes a mean of 10 measurements
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
            except rospy.ServiceException:
                print "GetCoord service not responding"

            self.pub.publish(self.measurement)

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

        cov = np.cov(xarr,yarr)
        cov_arr = []
        for i in range(0,len(cov)):
            for j in range(0,len(cov[i])):
                cov_arr.append(cov[i,j])
        plt.plot(xarr,yarr)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.savefig('Covariance.png')
        return cov_arr

if __name__ == '__main__':
    try:
        rospy.init_node('Sensor')
        #u = UWBPoseHandler(int(sys.argv[1]))
        u0 = UWBPoseHandler(0)
        u1 = UWBPoseHandler(1)
        c = u.measure_cov()
        #Send Covariance
        pub0 = rospy.Publisher("Sensor/covariance", float64[4],queue_size=1)
        pub0.publish(c)
        u0.covariance_sent = True
        u1.covariance_sent = True
        print "Sensor setup done"

        while True:
            u0.measure()
            rospy.sleep(0.05)
            u1.measure()
            rospy.sleep(0.05)


    except rospy.ROSInterruptException:
        pass
