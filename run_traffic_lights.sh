#!/bin/bash

set -e -x

source /home/software/docker/env.sh

roslaunch traffic_light traffic_light_node.launch veh:=$HOSTNAME
