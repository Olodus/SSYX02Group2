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

        # transform to Odometry

        # publish Odometry
        self.pub.publish(self.measurement)

if __name__ == '__main__':
    try:
        # Take the cmd argument which tells you the robot_id

        uwb = UWBHandler(arg)

        while True:
            uwb.measure()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
