#!/usr/bin/env python

from nav_msgs.msg import Odometry
import sys
import rospy

class PoseHandler(object):

    def __init__(self, robot_id, offsetX, offsetY):
        rospy.init_node('Sensor'+str(robot_id))
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.pub = rospy.Publisher("Sensor"+str(robot_id)+"/measurement", Odometry, queue_size=1)
        self.sub = rospy.Subscriber("RosAria"+str(robot_id)+"/pose", Odometry, self.measure)

    def measure(self, data):
        data.pose.pose.position.x = data.pose.pose.position.x + self.offsetX
        data.pose.pose.position.y = data.pose.pose.position.y + self.offsetY
        self.pub.publish(data)

if __name__ == '__main__':
    try:
        if int(sys.argv[1])==0:
            PoseHandler(int(sys.argv[1]),0.0,0.0)
        else:
            PoseHandler(int(sys.argv[1]),0.0,1.0)
        print "PoseHandler setup done."
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
