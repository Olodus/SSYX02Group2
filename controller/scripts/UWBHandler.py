from nav_msgs.msg import Odometry
import sys
import rospy
import csv
import time
import numpy as np
from rospy.numpy_msg import numpy_msg
from robotclient.msg import Floats
from robotclient.srv import *

class UWBHandler(object):

    def __init__(self, robot_id):
        rospy.init_node('Sensor'+str(robot_id))
        self.measurement = Odometry()
        self.pub = rospy.Publisher("Sensor"+str(robot_id)+"/measurement", Odometry, queue_size=1)

        srv = 'get_coord' + str(0)
        rospy.wait_for_service(srv)
        get_coords = rospy.ServiceProxy(srv, GetCoord)

    def measure():
        # call UWB service for measurement
        try:
            f = Floats()
            f = self.get_coords(1)
            # transform to Odometry
            x = f.data.data[0]
            y = f.data.data[1]
            self.measurement = Odometry()
            self.measurement.pose.pose.position.x = x
            self.measurement.pose.pose.position.y = y
        except rospy.rospy.ServiceException:
            pass

        # publish Odometry
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
                f = get_coords(1)
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

        cov = np.cov(xarr,yarr)
        plt.plot(xarr,yarr)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.savefig('Covariance.png')
        return cov

if __name__ == '__main__':
    try:
        # Take the cmd argument which tells you the robot_id

        uwb = UWBHandler(arg)
        c = uwb.measure_cov()
        #Send Covariance

        while True:
            uwb.measure()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
