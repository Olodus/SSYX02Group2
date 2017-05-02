#!/usr/bin/env python

from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from controller.srv import *
import sys
import math
import rospy
from Helper import RobotServices
import Helper as h
import threading
import random
import time

def run_controller(r0,r1):
    # Both robots go to their start points
    p0 = Point()
    p0.x = -a
    p0.y = 0.0
    r0.go_to_point(p0)
    p1 = Point()
    p1.x = 0.0
    p1.y = -a
    r1.go_to_point(p1)
    rospy.sleep(0.5)
    h.wait_til_both_ready(r0, r1)
    print "Both robots are now at starting points"

    # Both robots aim at the other side of the intersections
    p0 = Point()
    p0.x = a
    p0.y = 0.0
    r0.aim_at_point(p0)
    p1 = Point()
    p1.x = 0.0
    p1.y = a
    r1.aim_at_point(p1)
    rospy.sleep(0.5)
    h.wait_til_both_ready(r0, r1)
    print "Both robots are now aimed correctly"

    thread0 = threading.Thread(group=None, target=drive_robot,name="Thread0", args=(r0,p0))
    thread1 = threading.Thread(group=None, target=drive_robot,name="Thread1", args=(r1,p1))

    thread0.start()
    thread1.start()


def drive_robot(r,p):
    r.steer_towards(p)
    speed_interval = [0.1, 0.2]
    rand = random.uniform(speed_interval[0], speed_interval[1])
    r.set_speed(rand)

    state = r.get_state().state
    range = math.sqrt(state.pose.pose.position.x ** 2 + state.pose.pose.position.y ** 2)
    while range >= asking_range:
        state = r.get_state().state
        range = math.sqrt(state.pose.pose.position.x ** 2 + state.pose.pose.position.y ** 2)
        time.sleep(0.1)

    r.stop()
    permission.acquire()
    r.steer_towards(p)
    r.set_speed(0.2)
    while not r.is_ready().robot_ready:
        time.sleep(0.1)


if __name__ == '__main__':
    try:
        rospy.init_node("Controller")
        rospy.sleep(0.5)
        r0 = RobotServices(0)
        r1 = RobotServices(1)
        startTime = rospy.get_time()
        print "Controller setup done"

        a = 5.0
        global asking_range
        asking_range = 2.5
        global permission
        permission = threading.Semaphore()

        while True:
            run_controller(r0, r1)
            tmp_r = r0
            r0 = r1
            r1 = tmp_r

    except rospy.ROSInterruptException:
        pass
