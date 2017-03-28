#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import Odometry

class KalmanFilter(object):
    def __init__(self, robot_id):
        # Does this need to be a node? Maybe... maybe not... ?
        rospy.init_node('Filter'+str(robot_id))

        self.pub = rospy.Publisher("Filter"+str(robot_id)+"/state", Odometry, queue_size=1)
        self.sub = rospy.Subscriber("Sensor"+str(robot_id)+"/state", Odometry, self.new_measurement)
        self.measurement = Odometry()
        self.prediction = Odometry()
        self.state = Odometry()

    def new_measurement(self, data):
        self.measurement = data
        #check_previous_prediction()
        #remove_predicted_error()
        #create_new_prediction()

        self.pub.publish(self.state)

#    def check_previous_prediction(self):

#    def remove_predicted_error(self):

#    def create_new_prediction(self):

if __name__ == '__main__':
    try:
        k = KalmanFilter(int(sys.argv[1]))
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
