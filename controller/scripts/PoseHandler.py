#!/usr/bin/env python

from nav_msgs.msg import Odometry
import sys
import rospy
import random
import math

class PoseHandler(object):

    def __init__(self, robot_id, offsetX, offsetY, noice):
        rospy.init_node('Sensor'+str(robot_id))
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.pub = rospy.Publisher("Sensor"+str(robot_id)+"/measurement", Odometry, queue_size=1)
        self.sub = rospy.Subscriber("RosAria"+str(robot_id)+"/pose", Odometry, self.measure)
        self.noice = noice

    def measure(self, data):
        n_x = 0.0
        n_y = 0.0
        if math.fabs(self.noice) > 0.0001:
            n_x = random.uniform(-self.noice,self.noice)
            n_y = random.uniform(-self.noice,self.noice)
        data.pose.pose.position.x = data.pose.pose.position.x + self.offsetX + n_x
        data.pose.pose.position.y = data.pose.pose.position.y + self.offsetY + n_y
        self.pub.publish(data)

if __name__ == '__main__':
    try:
        if int(sys.argv[1])==0:
            PoseHandler(int(sys.argv[1]),0.0,0.0,float(sys.argv[2]))
        else:
            PoseHandler(int(sys.argv[1]),0.0,-1.0,float(sys.argv[2]))
        print "PoseHandler setup done."
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
