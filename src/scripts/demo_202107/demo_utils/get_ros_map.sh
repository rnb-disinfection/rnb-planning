#!/bin/sh
export ROS_MASTER_URI=http://$1:11311 # master ip
export ROS_HOSTNAME=$2 # slave ip (this pc)
python ros_map_utils.py $2