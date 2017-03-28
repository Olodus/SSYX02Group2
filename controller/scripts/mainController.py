#!/usr/bin/env python
import rospy
import KalmanFilter,PresentationProblem,RobotHandler
from PresentationProblem import PresentationProblem
from RobotHandler import RobotHandler

# Only supports 1 or 2 robots for now
def simulator_setup(nbr_of_robots, start_positions):
    # Create pose handlers
     # subscribe pose handlers to Rosaria pose
     # give correct offset if several
    # Create KalmanFilters
     # subscribe them to pose handlers
    # Create robot handlers
     # subscribe them to KalmanFilter
     # make robots move towards their startPoint

    robots = [RobotHandler(0),RobotHandler(1)]
    for i in range(0,nbr_of_robots-1):
        offsetX = 0.0
        offsetY = 0.0
        if i == 1:
            offsetY = 1.0

        h = PoseHandler(i,offsetX,offsetY)
        h_sub = rospy.Subscriber("RosAria"+str(i)+"/pose", Odometry, h.measure)
        kf = KalmanFilter(i)
        kf_sub = rospy.Subscriber("Sensor"+str(i)+"/measurement", Odometry, kf.new_measurement)
        #robots[i] = RobotHandler(i)
        r_sub = rospy.Subscriber("Filter"+str(i)+"/state", Odometry, robots[i].update_state)

    print len(robots)
    return robots

def real_world_setup(nbr_of_robots):
    # Create UWBHandlers
     # This is done from the commandline
    # Create KalmanFilters
     # Subscribe to UWBHandlers
    # Create RobotHandlers
     # Suscribe them to KalmanFilters
     # Make them move towards start_positions

    robots = []
    for i in range(0,nbr_of_robots-1):
        kf = KalmanFilter(i)
        kf_sub = rospy.Subscriber("Sensor"+str(i)+"/measurement", Odometry, kf.new_measurement)
        robots[i] = RobotHandler(i)
        r_sub = rospy.Subscriber("Filter"+str(i)+"/state", Odometry, robots[i].update_state)

    return robots

if __name__ == '__main__':
    try:
        rospy.init_node('Controller', anonymous=True)

        problem = PresentationProblem()
        nr = problem.nbr_of_robots
        start = problem.get_start_positions()

        robots = simulator_setup(nr, start)
        #robots = real_world_setup(nr)

        problem.run(robots)

    except rospy.ROSInterruptException:
        pass
