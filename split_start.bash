if [[ $* == *-robot0* ]]
then
  rosrun controller get_coord_server.py __name:=get_coord_server0 _ip_of_uwb:=101
  sleep 1
	rosrun rosaria RosAria __name:="RosAria0" _port:=localhost:8101 &
	sleep 1
  rosrun controller KalmanFilter.py 0 &
  sleep 1
  rosrun controller RobotHandler.py 0 &
fi

if [[ $* == *-robot1* ]]
then
  rosrun controller get_coord_server.py __name:=get_coord_server1 _ip_of_uwb:=102
  sleep 1
  rosrun rosaria RosAria __name:="RosAria1" _port:=localhost:8102 &
	sleep 1
  rosrun controller KalmanFilter.py 1 &
  sleep 1
  rosrun controller RobotHandler.py 1 &
fi
