#!/usr/bin/env python

from nav_msgs.msg import Odometry
import sys
import rospy
import random
import math

class PoseHandler(object):

    def __init__(self, robot_id, offsetX, offsetY, noice):
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.pub = rospy.Publisher("Sensor/measurement"+str(robot_id), Odometry, queue_size=1)
        self.sub = rospy.Subscriber("RosAria"+str(robot_id)+"/pose", Odometry, self.update)
        self.noice = noice
        self.state = Odometry()

    def update(self, data):
        n_x = 0.0
        n_y = 0.0
        if math.fabs(self.noice) > 0.0001:
            n_x = random.uniform(-self.noice,self.noice)
            n_y = random.uniform(-self.noice,self.noice)
        data.pose.pose.position.x = data.pose.pose.position.x + self.offsetX + n_x
        data.pose.pose.position.y = data.pose.pose.position.y + self.offsetY + n_y
        self.state = data

    def measure(self):
        self.pub.publish(self.state)

if __name__ == '__main__':
    try:
        rospy.init_node('Sensor')
        p0 = PoseHandler(int(sys.argv[1]), 0.0, 0.0, float(sys.argv[2]))
        p1 = PoseHandler(int(sys.argv[1]), 0.0, -1.0, float(sys.argv[2]))

        print "PoseHandler setup done."

        while True:
            p0.measure()
            rospy.sleep(0.05)
            p1.measure()
            rospy.sleep(0.05)

    except rospy.ROSInterruptException:
        pass
