#!/usr/bin/env python

import rospy
from controller.srv import *

class RobotServices(object):
    def __init__(self,robot_id):
        self.robot_id = robot_id
        self.is_ready = setup_is_ready(robot_id)
        self.aim_at_point = setup_aim_at_point(robot_id)
        self.go_to_point = setup_go_to_point(robot_id)
        self.follow_path = setup_follow_path(robot_id)
        self.set_speed = setup_set_speed(robot_id)
        self.stop = setup_stop(robot_id)
        self.set_acc = setup_set_acc(robot_id)
        self.get_state = setup_get_state(robot_id)
        self.set_ang_vel = setup_set_ang_vel(robot_id)
        self.steer_towards = setup_steer_towards(robot_id)



def setup_stop(robot_id):
    rospy.wait_for_service("Robot"+str(robot_id)+"/stop")
    return rospy.ServiceProxy("Robot"+str(robot_id)+"/stop",Stop)

def setup_is_ready(robot_id):
    rospy.wait_for_service("Robot"+str(robot_id)+"/is_ready")
    return rospy.ServiceProxy("Robot"+str(robot_id)+"/is_ready",IsReady)

def setup_aim_at_point(robot_id):
    rospy.wait_for_service("Robot"+str(robot_id)+"/aim_at_point")
    return rospy.ServiceProxy("Robot"+str(robot_id)+"/aim_at_point",AimAtPoint)

def setup_go_to_point(robot_id):
    rospy.wait_for_service("Robot"+str(robot_id)+"/go_to_point")
    return rospy.ServiceProxy("Robot"+str(robot_id)+"/go_to_point",GoToPoint)

def setup_follow_path(robot_id):
    rospy.wait_for_service("Robot"+str(robot_id)+"/follow_path")
    return rospy.ServiceProxy("Robot"+str(robot_id)+"/follow_path",FollowPath)

def setup_set_speed(robot_id):
    rospy.wait_for_service("Robot"+str(robot_id)+"/set_speed")
    return rospy.ServiceProxy("Robot"+str(robot_id)+"/set_speed",SetSpeed)

def setup_set_acc(robot_id):
    rospy.wait_for_service("Robot"+str(robot_id)+"/set_acc")
    return rospy.ServiceProxy("Robot"+str(robot_id)+"/set_acc",SetAcc)

def setup_set_ang_vel(robot_id):
    rospy.wait_for_service("Robot" + str(robot_id) + "/set_ang_vel")
    return rospy.ServiceProxy("Robot" + str(robot_id) + "/set_ang_vel", SetAngVel)

def setup_steer_towards(robot_id):
    rospy.wait_for_service("Robot" + str(robot_id) + "/steer_towards")
    return rospy.ServiceProxy("Robot" + str(robot_id) + "/steer_towards", SteerTowards)

def setup_get_state(robot_id):
    rospy.wait_for_service("Robot"+str(robot_id)+"/get_state")
    return rospy.ServiceProxy("Robot"+str(robot_id)+"/get_state",GetState)

def wait_til_both_ready(r0,r1):
    while not r0.is_ready().robot_ready:
        rospy.sleep(1.0)
    while not r1.is_ready().robot_ready:
        rospy.sleep(1.0)
