#!/bin/bash

rosrun controller PoseHandler.py 0 &
sleep 1
rosrun controller PoseHandler.py 1 &
sleep 1
rosrun controller NonFilter.py 0 &
sleep 1
rosrun controller NonFilter.py 1 &
sleep 1
rosrun controller RobotHandler.py 0 &
sleep 1
rosrun controller RobotHandler.py 1 &
sleep 1
rosrun controller PriorityProblem.py 
