#!/bin/bash
# This one is really risky because we are backgrounding the roslaunch command, 
# so when you Ctrl C this bash script, that roslaunch lives on!
# But if we do not backgrounding roslaunch, each command is a blocking function, which is bad
PID_list=/tmp/recon.pid

trap ctrl_c INT

function ctrl_c() {
        echo "Quitting ALL"
	pkill -F $PID_list
	rm $PID_list
}

roslaunch recon_sim recon.launch robot_ns:=panda_1 arm_id:=panda_1 conveyor_length:=3 y:=-0.5 &
echo $! >> $PID_list
sleep 10
rosservice call /gazebo/pause_physics "{}"
sleep 10
roslaunch recon_sim recon_dual.launch robot_ns:=panda_2 arm_id:=panda_2 y:=0.5 &
echo $! >> $PID_list
