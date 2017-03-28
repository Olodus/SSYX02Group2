#!/usr/bin/env python

from geometry_msgs.msg import Point
from controller.srv import *
import sys
import rospy

class PresentationProblem(object):

    def run(self):
        r1_acc = rospy.ServiceProxy("Robot0/set_acc",SetAcc)
        resp = r1_acc(0.5)



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
