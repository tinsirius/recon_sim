#!/bin/bash
roslaunch recon_sim recon.launch robot_ns:=panda_1 paused:=true arm_id:=panda_1 conveyor_length:=3 y:=-0.5
sleep 10
roslaunch recon_sim dual_arm_second.launch robot_ns:=panda_2 arm_id:=panda_2 y:=0.5
