#!/usr/bin/env python

from nav_msgs.msg import Odometry
import sys
import rospy
import random
import math

class EKFPoseHandler(object):

    def __init__(self, robot_id, offsetX, offsetY, std_dev):
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.pub = rospy.Publisher("robot_pose_rkf"+str(robot_id)+"/odom"+str(robot_id), Odometry, queue_size=1)
        self.sub = rospy.Subscriber("RosAria"+str(robot_id)+"/pose", Odometry, self.update)
        self.std_dev = std_dev
        self.state = Odometry()

    def update(self, data):
        n_x = 0.0
        n_y = 0.0
        if math.fabs(self.std_dev) > 0.0001:
            n_x = random.gauss(0.0, self.std_dev)
            n_y = random.gauss(0.0, self.std_dev)
        data.pose.pose.position.x = data.pose.pose.position.x + self.offsetX + n_x
        data.pose.pose.position.y = data.pose.pose.position.y + self.offsetY + n_y
        self.state = data

    def measure(self):
        self.pub.publish(self.state)

if __name__ == '__main__':
    try:
        nbr_of_robots = int(sys.argv[1])
        dev = float(sys.argv[2])
        rospy.init_node('Sensor')
        rs = []
        for i in range(0, nbr_of_robots):
            r = EKFPoseHandler(i, 0.0, -float(i), dev)
            rs.append(r)
        print "PoseHandler setup done."

        while True:
            for i in range(0, nbr_of_robots):
                rs[i].measure()
                rospy.sleep(0.05)

    except rospy.ROSInterruptException:
        pass
