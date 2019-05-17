#!/bin/bash

set -e -x


#source /catkin-ws/devel/setup.bash
#cd /catkin-ws/ 
#catkin_make

#cd /home/software/catkin_ws/src
source /home/software/docker/env.sh
cd /home/software/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="duckietown_msgs" - DTCATKIN_BLACKLIST_PACKAGES="apriltag_ros"
roslaunch traffic_light traffic_light_node.launch veh:=$HOSTNAME
