#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import Odometry

class EKFFilter(object):
    def __init__(self, robot_id):
        rospy.init_node('Filter'+str(robot_id))

        self.pub = rospy.Publisher("Filter"+str(robot_id)+"/state", Odometry, queue_size=1)
        self.ekf_sub = rospy.Subscriber("robot_pose_ekf"+str(robot_id)+"/odom_combined", Odometry, self.ekf_update)
        self.pose_sub = rospy.Subscriber("RosAria"+str(robot_id)+"/pose", Odometry, self.new_speed)
        self.pose_state = Odometry()

    def new_speed(self,data):
        self.pose_state = data

    def ekf_update(self, data):
        self.pose_state.pose = data
        self.pub.publish(data)

if __name__ == '__main__':
    try:
        n = EKFFilter(int(sys.argv[1]))
        print "Filter setup done."
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
