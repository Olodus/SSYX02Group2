#!/bin/bash

rosrun controller PoseHandler.py 0.0 &
sleep 1
rosrun controller KalmanFilter.py 0 &
sleep 1
rosrun controller KalmanFilter.py 1 &
sleep 1
rosrun controller RobotHandler.py 0 &
sleep 1
rosrun controller RobotHandler.py 1 &
sleep 1
if [[ $* == *-priority* ]]
then 
	rosrun controller testKalman.py
fi
if [[ $* == *-reservation* ]]
then
	rosrun controller ReservationProblem.py
fi
