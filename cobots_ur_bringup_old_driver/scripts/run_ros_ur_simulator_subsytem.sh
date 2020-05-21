## Cobots ROS UR Simulator Script
# Author: Curt Henrichs
# 
# This script should be copied to your desktop after installing everything 
# necessary to run the ROS subsystem. Simply update the ROS_TYPE to be on of 
# the following: planner, simulated, physical. Then run the script.

ROS_TYPE="planner"

# Launch ROS system
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch cobots_ur_bringup main.launch local:=false type:=$ROS_TYPE
