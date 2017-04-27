#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import Odometry
from kalman2 import Kalman
#from Kalman123 import Kalman
import math
import tf
import numpy as np

class KalmanFilter(object):
    def __init__(self, robot_id):
        # Does this need to be a node? Maybe it could be a tf?
        rospy.init_node('Filter'+str(robot_id))

        self.pub = rospy.Publisher("Filter"+str(robot_id)+"/state", Odometry, queue_size=1)
        self.sub = rospy.Subscriber("Sensor/measurement"+str(robot_id), Odometry, self.new_measurement)
        self.kf = Kalman()
        self.state_out = Odometry()
        self.state = np.array([[0],[0],[0]])
        self.startTime = rospy.get_time()
        self.oldTime = 0.0

    def new_measurement(self, data):
        self.startTime = rospy.get_time()
        timestep = self.startTime - self.oldTime
        self.oldTime = self.startTime
	print timestep
        self.state = Kalman.update(self.kf,self.state, data.twist.twist.linear.x,data.twist.twist.angular.z,timestep, [data.pose.pose.position.x, data.pose.pose.position.y])
        self.state_out.pose.pose.position.x = self.state[0]
        self.state_out.pose.pose.position.y = self.state[1]
        self.state_out.twist.twist.linear.x = data.twist.twist.linear.x
	angle = 1+self.state[2]-1
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
        self.state_out.pose.pose.orientation.x = quat[0]
        self.state_out.pose.pose.orientation.y = quat[1]
        self.state_out.pose.pose.orientation.z = quat[2]
        self.state_out.pose.pose.orientation.w = quat[3]
        self.state_out.twist.twist.angular.z = data.twist.twist.angular.z
        #print str(self.state)

        self.pub.publish(self.state_out)


if __name__ == '__main__':
    try:
        k = KalmanFilter(int(sys.argv[1]))
        print "Filter setup done."
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
