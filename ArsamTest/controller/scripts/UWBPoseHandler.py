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
        self.pub = rospy.Publisher("Sensor"+str(robot_id)+"/measurement", Odometry, queue_size=1)
        self.sub = rospy.Subscriber("RosAria"+str(robot_id)+"/pose", Odometry, self.measure)

        srv = 'get_coord' + str(0)
        rospy.wait_for_service(srv)
        get_coords = rospy.ServiceProxy(srv, GetCoord)

        self.measurement = Odometry()

    def measure(self, data):
        # Take orientation and velocity from pose
        self.measurement.pose.pose.orientation = data.pose.pose.orientation
        self.measurement.twist = data.twist

        # The rest is taken from UWB   as of now takes a mean of 10 measurements
        xarr = np.array([], dtype=np.float32)
        yarr = np.array([], dtype=np.float32)
        i = 0
        n = 10
        error_count = 0
        while i < n and error_count < 100:
            print("remaining time: " + str((n - i) * 0.14/60))
            print("Take:" + str(i))
            tmp_pos = np.array([],
                             dtype=np.float32)
            try:
                #startTime = time.time()
                f = Floats()
                f = get_coords(1)
                #stopTime = time.time()
                tmp_pos = f.data.data
                if np.size(tmp_pos) != 3:
                    xarr = np.append(xarr, tmp_pos[0])
                    yarr = np.append(yarr, tmp_pos[1])
                    i += 1
                else:
                    error_count += 1
                    print("Invalid reading, check if all the unicorns are at home")
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            #print(str(tmp_pos))

        xmean = np.mean(xarr)
        ymean = np.mean(yarr)
        cov = np.cov(xarr,yarr)
        covx = np.cov(xarr)
        covy = np.cov(yarr)

        self.measurement.pose.pose.position.x = xmean
        self.measurement.pose.pose.position.y = ymean
        self.measurement.pose.covariance = cov

        self.pub.publish(self.measurement)

if __name__ == '__main__':
    try:
        UWBPoseHandler(int(sys.argv[1]))

    except rospy.ROSInterruptException:
        pass