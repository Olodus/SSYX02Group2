#!/usr/bin/env python

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Quaternion, Twist
from controller.srv import *
import tf
import dynamic_reconfigure.client
import math
import rospy
import sys
import dynamic_reconfigure.client
import threading


# Missions
#  -1 : No mission
#   0 : Go to Point
#   1 : Follow path
#   2 : Aim at point
#	3 : Free moving
#	4 : Set acceleration (not used anymore)
#	5 : Stop
#   6 : Steer towards

####### Do Missions ########
# All do methods return True if they are done
# This way all do methods can call other do methods to do part of their job
# without having to check their "done test"


class Robot(object):

    def update_state(self,data):
        self.state = data

    def do_mission(self):
        self.twist = Twist()
        if self.mission == 0:
            return self.do_go_to_point()
        elif self.mission == 1:
            return self.do_follow_path()
        elif self.mission == 2:
            return self.do_aim_at_point()
        elif self.mission == 3:
            return self.do_set_speed()
        elif self.mission == 4:
            self.do_set_acc()
        elif self.mission == 5:
            return self.do_stop()
        elif self.mission == 6:
            return self.do_steer_towards()

    def publish_twist(self):
        self.pub.publish(self.twist)

    def end_mission(self):
        if self.mission == 0:
            # Go to point
            self.mission = 5
        elif self.mission == 1:
            # Follow path
            self.mission = 5
        elif self.mission == 2:
            # Aim at point
            self.mission = 5
        elif self.mission == 3:
            # Set speed
            self.executing = False
            self.moving = True
        elif self.mission == 4:
            # Set acc
            # dont need to do something right?
            self.moving = True
        elif self.mission == 5:
            # Stop
            self.executing = False
            self.mission = -1
        elif self.mission == 6:
            # Steer towards
            self.mission = 5

    def get_state(self, req):
        return {'state': self.state}

    def is_ready(self, req):
        return {'robot_ready': not self.executing}

    def aim_at_point(self, req):
        self.mission_lock.acquire()
        if self.executing:
            error = "Error: Robot " + str(self.id_nbr) + ": Is already doing something"
            self.mission_lock.release()
            return {'succeded': False, 'error_msg': error}
        else:
            if self.mission == 4:
                para = {'trans_accel': 1.0, 'trans_decel': 1.0}
                self.dyn_par.update_configuration(para)
            self.executing = True
            self.mission = 2
            self.aim_point = req.point
            msg = "Robot " + str(self.id_nbr) + ": Will aim itself towards given point"
            self.mission_lock.release()
            return {'succeded': True, 'error_msg': msg}

    def go_to_point(self, req):
        self.mission_lock.acquire()
        if self.executing:
            error = "Error: Robot " + str(self.id_nbr) + ": Is already doing something"
            self.mission_lock.release()
            return {'succeded': False, 'error_msg': error}
        else:
            if self.mission == 4:
                para = {'trans_accel': 1.0, 'trans_decel': 1.0}
                self.dyn_par.update_configuration(para)
            self.executing = True
            self.mission = 0
            self.go_point = req.point
            msg = "Robot " + str(self.id_nbr) + ": Will go to given point"
            self.mission_lock.release()
            return {'succeded': True, 'error_msg': msg}


    def follow_path(self, req):
        self.mission_lock.acquire()
        if self.executing:
            error = "Robot " + str(self.id_nbr) + ": Is already doing something"
            self.mission_lock.release()
            return {'succeded': False, 'error_msg': error}
        else:
            if self.mission == 4:
                para = {'trans_accel': 1.0, 'trans_decel': 1.0}
                self.dyn_par.update_configuration(para)
            self.executing = True
            self.mission = 1
            self.path = req.path
            self.path_index = 0
            msg = "Robot " + str(self.id_nbr) + ": Will follow given path"
            self.mission_lock.release()
            return {'succeded': True, 'error_msg': msg}

    def set_speed(self, req):
        self.mission_lock.acquire()
        if self.executing:
            if self.mission == 6:
                self.is_steering_with_acc = False
                self.desired_speed = req.speed
                msg = "Robot " + str(self.id_nbr) + ": Will set speed to " + str(req.speed)
                self.mission_lock.release()
                return {'succeded': True, 'error_msg': msg}
            else:
                error = "Robot " + str(self.id_nbr) + ": Can't set speed since already on a mission (use stop to abort mission)"
                self.mission_lock.release()
                return {'succeded': False, 'error_msg': error}
        else:
            para = {'trans_accel': 1.0, 'trans_decel': 1.0}
            self.dyn_par.update_configuration(para)
            self.mission = 3
            self.is_freemoving_with_acc = False
            self.desired_speed = req.speed
            msg = "Robot " + str(self.id_nbr) + ": Will set speed to " + str(req.speed)
            self.mission_lock.release()
            return {'succeded': True, 'error_msg': msg}

    def stop(self,req):
        para = {'trans_accel': 1.0, 'trans_decel': 1.0}
        self.dyn_par.update_configuration(para)
        self.mission_lock.acquire()
        if self.executing:
            error = "Robot " + str(self.id_nbr) + ": Stopped executing it's mission"
            self.mission = 5
            self.mission_lock.release()
            return {'succeded': True, 'error_msg': error}
        else:
            msg = "Robot " + str(self.id_nbr) + ": Stopping"
            self.mission = 5
            self.mission_lock.release()
            return {'succeded': False, 'error_msg': msg}

    def set_ang_speed(self,req):
        self.mission_lock.acquire()
        if self.executing:
            error = "Robot " + str(self.id_nbr) + ": Can't set ang vel since already on a mission (use stop to abort mission)"
            self.mission_lock.release()
            return {'succeded': False, 'error_msg': error}
        else:
            self.mission = 3
            self.ang_vel = req.ang_vel
            msg = "Robot " + str(self.id_nbr) + ": Will set ang vel to " + str(req.ang_vel)
            self.mission_lock.release()
            return {'succeded': True, 'error_msg': msg}

    def set_acc(self, req):
        self.mission_lock.acquire()
        if self.executing:
            if self.mission == 6:
                self.is_steering_with_acc = True
                self.acc = req.acc
                msg = "Robot " + str(self.id_nbr) + ": Will set acc to " + str(req.acc)
                self.mission_lock.release()
                return {'succeded': True, 'error_msg': msg}
            else:
                error = "Robot " + str(self.id_nbr) + ": Can't set acc since already on a mission (use stop to abort mission)"
                self.mission_lock.release()
                return {'succeded': False, 'error_msg': error}
        else:
            self.mission = 3
            self.acc = req.acc
            self.is_freemoving_with_acc = True
            msg = "Robot " + str(self.id_nbr) + ": Will set acc to " + str(req.acc)
            self.mission_lock.release()
            return {'succeded': True, 'error_msg': msg}

    def steer_towards(self, req):
        self.mission_lock.acquire()
        if self.executing:
            error = "Robot " + str(self.id_nbr) + ": Can't steer towards since already on a mission (use stop to abort mission)"
            self.mission_lock.release()
            return {'succeded': False, 'error_msg': error}
        else:
            self.executing = True
            self.is_steering_with_acc = False
            self.mission = 6
            msg = "Robot " + str(self.id_nbr) + ": Will steer towards" + str(req.point)
            self.mission_lock.release()
            return {'succeded': True, 'error_msg': msg}


    def do_go_to_point(self):
        x = self.state.pose.pose.position.x
        y = self.state.pose.pose.position.y
        pointx = self.go_point.x
        pointy = self.go_point.y

        self.length_to_point = math.sqrt(math.pow(pointx-x,2)+math.pow(pointy-y,2))
        length = self.length_to_point

        self.aim_point = self.go_point
        if not self.small_steer():
            return False
        elif length >= 0.1:
            self.desired_speed = length*0.7
            self.do_set_speed()
            return False
        else:
            return True


    def do_follow_path(self):
        if self.path_index == len(self.path.poses):
            return True
        else:
            self.go_point.x = self.path.poses[self.path_index].pose.position.x
            self.go_point.y = self.path.poses[self.path_index].pose.position.y
            if self.do_go_to_point():
                self.path_index = self.path_index+1
            return False

    def do_aim_at_point(self):
        x = self.state.pose.pose.position.x
        y = self.state.pose.pose.position.y
        pointx = self.aim_point.x
        pointy = self.aim_point.y
        angle = math.atan2(pointy-y,pointx-x)
        quart = (self.state.pose.pose.orientation.x, self.state.pose.pose.orientation.y, self.state.pose.pose.orientation.z, self.state.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quart)
        robot_angle = euler[2]
        self.angle_to_point = robot_angle - angle
        if self.angle_to_point > math.pi:
            self.angle_to_point = self.angle_to_point-2*math.pi
        if self.angle_to_point < -math.pi:
            self.angle_to_point = self.angle_to_point+2*math.pi
        if self.angle_to_point < 0:
            rotSpeed = math.fabs(self.angle_to_point)*(2.0/math.pi)
        if self.angle_to_point > 0:
            rotSpeed = -math.fabs(self.angle_to_point)*(2.0/math.pi)

        if math.fabs(self.angle_to_point)>0.5*math.pi/180:
            self.twist.angular.z = rotSpeed
            #self.twist.linear.x = 0.05
            return False
        else:
            self.twist.angular.z = 0.0
            return True

    def do_set_speed(self):
        self.twist.linear.x = self.desired_speed
        return True#math.fabs(self.state.twist.twist.linear.x - self.desired_speed) <= 0.001

    def do_freemove(self):
        if self.is_freemoving_with_acc:
            self.do_set_acc()
        else:
            self.do_set_speed()
        self.do_set_ang_vel()
        return True

    def do_set_ang_vel(self):
        self.twist.angular.z = self.ang_vel
        return True

    def do_set_acc(self):
        #self.emulate_acc()
        self.parameter_acc()

    def do_stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        return math.fabs(self.state.twist.twist.linear.x) <= 0.01 and math.fabs(self.state.twist.twist.angular.z) <= 0.01

    def do_steer_towards(self):
        self.aim_point = self.steer_point
        self.do_aim_at_point()

        # Either do set speed or set acc
        if self.is_steering_with_acc:
            self.do_set_acc()
        else:
            self.do_set_speed()

        x = self.state.pose.pose.position.x
        y = self.state.pose.pose.position.y
        pointx = self.go_point.x
        pointy = self.go_point.y

        length = math.sqrt(math.pow(pointx - x, 2) + math.pow(pointy - y, 2))
        return length <= 0.1

    def small_steer(self):
        self.do_aim_at_point()
        x = self.state.pose.pose.position.x
        y = self.state.pose.pose.position.y
        pointx = self.aim_point.x
        pointy = self.aim_point.y
        angle = math.atan2(pointy-y,pointx-x)
        quart = (self.state.pose.pose.orientation.x, self.state.pose.pose.orientation.y, self.state.pose.pose.orientation.z, self.state.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quart)
        robot_angle = euler[2]
        angle2point = robot_angle - angle
        return math.fabs(angle2point) < math.pi/12

    def parameter_acc(self):
    ########what is a good value here?
        if math.fabs(self.last_acc-self.acc)>0.00000001:
            if self.acc > 0:
                para = {'trans_accel':math.fabs(self.acc),'trans_decel':math.fabs(self.acc)}
                self.dyn_par.update_configuration(para)
                self.desired_speed = self.max_speed
                self.do_set_speed()
            else:
                para = {'trans_accel':math.fabs(self.acc),'trans_decel':math.fabs(self.acc)}
                self.dyn_par.update_configuration(para)
                self.desired_speed = 0.0
                self.do_set_speed()
        else:
            if self.acc > 0:
                self.desired_speed = self.max_speed
                self.do_set_speed()
            else:
                self.desired_speed = 0.0
                self.do_set_speed()
        self.last_acc = self.acc
    '''
    timeNow = rospy.get_time()
    timePassed1 = timeNow-self.startTime
    if timePassed1-self.timePassed2 >= 3.0:
        print "current position of robot"+str(self.id_nbr)+" is: x="+str(self.state.pose.pose.position.x)+", y="+str(self.state.pose.pose.position.y)
        print "current velocity of robot"+str(self.id_nbr)+" is: "+str(self.state.twist.twist.linear.x)
        timeNow = rospy.get_time()
        print "time passed: "+str(timePassed1)
        self.timePassed2 = timeNow-self.startTime
    '''

    def emulate_acc(self):
        #TODO Create acc implementation that handles accual time not just 0.1 sec
        curr_speed = self.state.twist.twist.linear.x
        self.desired_speed = curr_speed + self.acc*0.1
        self.do_set_speed()

    def __init__(self, id_nbr):
        rospy.init_node('Robot'+str(id_nbr))
        self.id_nbr = id_nbr
        self.name = 'Robot'+str(id_nbr) #TODO replace all places name could be used instead
        self.pub = rospy.Publisher("RosAria"+str(id_nbr)+"/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("Filter"+str(id_nbr)+"/state", Odometry, self.update_state)
        self.twist = Twist()
        self.startTime = rospy.get_time()
        self.timePassed2 = 0.0

        # State
        self.state = Odometry()
        self.executing = False

        # Mission
        self.mission = -1
        self.aim_point = Point()
        self.angle_to_point = 0.0
        self.go_point = Point()
        self.length_to_point = 0.0
        self.path = Path()
        self.path_index = 0
        self.desired_speed = 0.0
        self.acc = 0.0
        self.last_acc = 0.0
        self.steer_point = Point()
        self.is_steering_with_acc = False
        self.is_freemoving_with_acc = False

        # Constants
        self.cruising_speed = 0.3
        self.max_speed = 1.0

        ##### Creates all services #####
        # Creating 'is_ready' service
        a = rospy.Service('Robot'+str(id_nbr)+'/is_ready', IsReady, self.is_ready)

        # Creating 'aim_at_point' service
        b = rospy.Service('Robot'+str(id_nbr)+'/aim_at_point', AimAtPoint, self.aim_at_point)

        # Creating 'go_to_point' service
        c = rospy.Service('Robot'+str(id_nbr)+'/go_to_point', GoToPoint, self.go_to_point)

        # Creating 'follow_path' service
        d = rospy.Service('Robot'+str(id_nbr)+'/follow_path', FollowPath, self.follow_path)

        # Creating 'set_speed' service
        e = rospy.Service('Robot'+str(id_nbr)+'/set_speed', SetSpeed, self.set_speed)

        # Creating 'stop' service
        f = rospy.Service('Robot'+str(id_nbr)+'/stop', Stop, self.stop)

        # Creating 'set_acc' service
        g = rospy.Service('Robot'+str(id_nbr)+'/set_acc', SetAcc, self.set_acc)

        # Creating 'steer_towards' service
        t = rospy.Service('Robot'+str(id_nbr)+'/steer_towards', SteerTowards, self.steer_towards)

        # Creating 'get_state' service
        h = rospy.Service('Robot' + str(id_nbr) + '/get_state', GetState, self.get_state)

        # Setting up a dynamic_parameter client
        self.dyn_par = dynamic_reconfigure.client.Client("RosAria"+str(self.id_nbr), timeout=10)

        self.mission_lock = threading.Lock()


if __name__ == '__main__':
    try:
        r = Robot(int(sys.argv[1]))
        print "Robot setup done"
        while True:
            if r.mission != -1:
                mission_done = r.do_mission()
                r.publish_twist()
                if mission_done:
                    r.end_mission()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
