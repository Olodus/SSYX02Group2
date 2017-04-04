#!/bin/bash
cd ~
cd catkin_ws
. devel/setup.bash
rosrun controller PoseHandler.py 0 &
sleep 1
rosrun controller PoseHandler.py 1 &
sleep 1
rosrun controller NonFilter.py 0 &
sleep 1
rosrun controller NonFilter.py 1 &
sleep 1
rosrun controller RobotHandler.py 0 &
sleep 2
rosrun controller RobotHandler.py 1 &
sleep 2
rosrun controller PriorityProblem.py 
