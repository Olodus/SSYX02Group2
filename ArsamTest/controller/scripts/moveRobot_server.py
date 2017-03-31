#!/usr/bin/env python

#from geometry_msgs.msg import Pose
from controller.srv import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import rospy


twist = Twist()
#pose = Pose()
def handle_moveStraight(req):
    pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size=10)
    
    if req.a ==1:
	twist.linear.x = req.b
#	print msg.pose.pose
    else:
	twist.angular.z = req.b

    pub.publish(twist)
#    sub.
    rospy.sleep(2)

 
    return moveStraightResponse(req.a + req.b)

def moveStraight_server():
    rospy.init_node('moveStraight_server')
    s = rospy.Service('move_Straight', moveStraight, handle_moveStraight)
    print "Give command: movement(0=straight, 1=angular) and speed"
    rospy.spin()

if __name__ == "__main__":
    moveStraight_server()


