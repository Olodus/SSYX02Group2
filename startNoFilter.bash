#!/bin/bash
cd ~
roscore & 
MobileSim -r p3dx:robot1 -r p3dx:robot2 --nomap &
cd catkin_ws
. devel/setup.bash
rosrun rosaria RosAria __name:=RosAria0 _port:=localhost:8101 &
sleep 1
rosrun rosaria RosAria __name:=RosAria1 _port:=localhost:8102 & 
sleep 1
x-terminal-emulator -e bash startRobots.bash
