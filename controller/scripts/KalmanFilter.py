#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import Odometry
from kalman import *
import math
import tf

class KalmanFilter(object):
    def __init__(self, robot_id):
        # Does this need to be a node? Maybe it could be a tf?
        rospy.init_node('Filter'+str(robot_id))

        self.pub = rospy.Publisher("Filter"+str(robot_id)+"/state", Odometry, queue_size=1)
        self.sub = rospy.Subscriber("Sensor"+str(robot_id)+"/measurement", Odometry, self.new_measurement)
        self.kf = Kalman()
        self.state_out = Odometry()

    def new_measurement(self, data):
        self.state = Kalman.update(self.kf,self.state,data.twist.twist.linear.x,data.twist.twist.angular.z,0.1,[data.pose.pose.position.x, data.pose.pose.position.y])
        self.state_out.pose.pose.position.x = self.state[0]
        self.state_out.pose.pose.position.y = self.state[2]
        self.state_out.twist.twist.linear.x = math.sqrt(self.state[1]**2+self.state[3]**2)
        quart = tf.transformations.quarternions_from_euler(0.0,0.0,self.state[4])
        self.state_out.pose.pose.orientation.x = quart[0]
        self.state_out.pose.pose.orientation.y = quart[1]
        self.state_out.pose.pose.orientation.z = quart[2]
        self.state_out.pose.pose.orientation.w = quart[3]
        self.state_out.twist.twist.angular.z = self.state[5]

        self.pub.publish(self.state_out)

#    def check_previous_prediction(self):

#    def remove_predicted_error(self):

#    def create_new_prediction(self):

if __name__ == '__main__':
    try:
        k = KalmanFilter(int(sys.argv[1]))
        print "Filter setup done."
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
