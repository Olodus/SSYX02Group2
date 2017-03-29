#!/bin/bash
if [[ $* == *-sim* ]]
then
	cd ~
	roscore & 
	MobileSim -r p3dx:robot1 --nomap &
	cd catkin_ws
	. devel/setup.bash
	rosrun rosaria RosAria __name:="RosAria0" & 
	sleep 1
	rosrun controller PoseHandler.py 0 &
	sleep 1
	rosrun controller KalmanFilter.py 0 &
	sleep 1
	rosrun controller RobotHandler.py 0 &
	sleep 1
	rosrun controller PresentationProblem.py 
else
	cd ~
	roscore & 
	cd catkin_ws
	. devel/setup.bash
	rosrun rosaria RosAria __name:="RosAria0" & 
	sleep 1
	rosrun controller UWBPoseHandler.py 0 &
	sleep 1
	rosrun controller KalmanFilter.py 0 &
	sleep 1
	rosrun controller RobotHandler.py 0 &
	sleep 1
	rosrun controller PresentationProblem.py 
fi
return
