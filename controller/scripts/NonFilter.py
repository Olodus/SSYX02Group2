#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import Odometry

class NoFilter(object):
    def __init__(self, robot_id):
        rospy.init_node('Filter'+str(robot_id))

        self.pub = rospy.Publisher("Filter"+str(robot_id)+"/state", Odometry, queue_size=1)
        self.sub = rospy.Subscriber("Sensor"+str(robot_id)+"/measurement", Odometry, self.new_measurement)

    def new_measurement(self, data):
        self.pub.publish(data)

if __name__ == '__main__':
    try:
        n = NoFilter(int(sys.argv[1]))
        print "NonFilter setup done."
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
