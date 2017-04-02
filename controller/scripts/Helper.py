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


def setup_get_state(robot_id):
  wait_for_service("Robot"+str(robot_id)+"/get_state")
  return rospy.ServiceProxy("Robot"+str(robot_id)+"/get_state",GetState)

def setup_stop(robot_id):

def setup_is_ready(robot_id):

def setup_aim_at_point(robot_id):

def setup_go_to_point(robot_id):

def setup_follow_path(robot_id):

def setup_set_speed(robot_id):

def setup_set_acc(robot_id):

def setup_get_state(robot_id):
