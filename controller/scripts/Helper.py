#!/usr/bin/env python

import rospy

class RobotServices(object):
    def __init__(robot_id):
      self.robot_id = robot_id
      self.is_ready = setup_is_ready(robot_id)
      self.aim_at_point = setup_aim_at_point(robot_id)
      self.go_to_point = setup_go_to_point(robot_id)
      self.follow_path = setup_follow_path(robot_id)
      self.set_speed = setup_set_speed(robot_id)
      self.stop = setup_stop(robot_id)
      self.set_acc = setup_set_acc(robot_id)
      self.get_state = setup_get_state(robot_id)


def setup_stop(robot_id):
  wait_for_service("Robot"+str(robot_id)+"/stop")
  return rospy.ServiceProxy("Robot"+str(robot_id)+"/stop",Stop)

def setup_is_ready(robot_id):
  wait_for_service("Robot"+str(robot_id)+"/is_ready")
  return rospy.ServiceProxy("Robot"+str(robot_id)+"/is_ready",IsReady)

def setup_aim_at_point(robot_id):
  wait_for_service("Robot"+str(robot_id)+"/aim_at_point")
  return rospy.ServiceProxy("Robot"+str(robot_id)+"/aim_at_point",AimAtPoint)

def setup_go_to_point(robot_id):
  wait_for_service("Robot"+str(robot_id)+"/go_to_point")
  return rospy.ServiceProxy("Robot"+str(robot_id)+"/go_to_point",GoToPoint)

def setup_follow_path(robot_id):
  wait_for_service("Robot"+str(robot_id)+"/follow_path")
  return rospy.ServiceProxy("Robot"+str(robot_id)+"/follow_path",FollowPath)

def setup_set_speed(robot_id):
  wait_for_service("Robot"+str(robot_id)+"/set_speed")
  return rospy.ServiceProxy("Robot"+str(robot_id)+"/set_speed",SetSpeed)

def setup_set_acc(robot_id):
  wait_for_service("Robot"+str(robot_id)+"/set_acc")
  return rospy.ServiceProxy("Robot"+str(robot_id)+"/set_acc",SetAcc)

def setup_get_state(robot_id):
  wait_for_service("Robot"+str(robot_id)+"/get_state")
  return rospy.ServiceProxy("Robot"+str(robot_id)+"/get_state",GetState)
