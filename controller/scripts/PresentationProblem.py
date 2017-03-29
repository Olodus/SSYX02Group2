#!/usr/bin/env python

from geometry_msgs.msg import Point
from controller.srv import *
import sys
import rospy

class PresentationProblem(object):

    def run(self):
        r1_g2p = rospy.ServiceProxy("Robot0/go_to_point",GoToPoint)
        p = Point()
        p.x = 5.0
        p.y = 5.0
        resp = r1_g2p(p)
        print str(resp)



if __name__ == '__main__':
    try:
        rospy.init_node("Controller")
        r = PresentationProblem()
        while True:
            r.run()
            rospy.sleep(1.0)

    except rospy.ROSInterruptException:
        pass

# Vad som kan brytas ut i en super Problem class
#  wait til ready metoden
#  robots variabeln
#  numberOfRobots
#  get_start_positions
