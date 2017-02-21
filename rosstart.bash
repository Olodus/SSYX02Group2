#!/bin/bash
roscore & MobileSim -r p3dx:robot1 -r p3dx:robot2 --nomap &
cd catkin_ws
. devel/setup.bash
rosrun rosaria RosAria __name:=RosAria _port:=localhost:8101 & rosrun rosaria RosAria __name:=RosAria1 _port:=localhost:8102
