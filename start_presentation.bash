#!/bin/bash
killall -9 rosmaster
if [[ $* == *-sim_priority* ]]
then
	roscore &
	MobileSim -r p3dx:robot1 --nomap & #-r p3dx:robot2 --nomap &
	rosrun rosaria RosAria __name:="RosAria0" _port:=localhost:8101 &
	sleep 1
	#rosrun rosaria RosAria __name:="RosAria1" _port:=localhost:8102 &
	#sleep 1
	x-terminal-emulator -e bash robot_run.bash -priority
fi

if [[ $* == *-priority* ]]
then
	roscore &
	rosrun rosaria RosAria __name:="RosAria0" &
	sleep 1
	x-terminal-emulator -e bash robot_run.bash -priority
fi

if [[ $* == *-sim_reservation* ]]
then
	roscore &
	MobileSim -r p3dx:robot1 -r p3dx:robot2 --nomap &
	rosrun rosaria RosAria __name:="RosAria0" _port:=localhost:8101 &
	sleep 1
	rosrun rosaria RosAria __name:="RosAria1" _port:=localhost:8102 &
	sleep 1
	x-terminal-emulator -e bash robot_run.bash -reservation
fi

if [[ $* == *-reservation* ]]
then
	roscore &
	rosrun rosaria RosAria __name:="RosAria0" &
	sleep 1
	x-terminal-emulator -e bash robot_run.bash -reservation
fi

return
