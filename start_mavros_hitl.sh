sudo chmod 777 /dev/ttyACM0 & sleep 1;
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14550" & sleep 10;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 32 10000 0 0 0 0 0 & sleep 1;
wait;
