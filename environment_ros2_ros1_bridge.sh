#! /bin/bash

#################### ROS1 ####################

if [ $# -gt 0 ]; then
    echo "ROS_MASTER_IP set to $1"
    export ROS_MASTER_URI=http://$1:11311
else
    export ROS_MASTER_URI=http://127.0.0.1:11311
    echo "ROS_MASTER_URI set to $ROS_MASTER_URI"
fi


if [ $# -gt 1 ]; then
	export ROS_IP=$2
    echo "ROS_IP set to $ROS_IP"
else
    export ROS_IP=127.0.0.1
    echo "ROS_IP set to $ROS_IP"
fi

#################### ROS2-ROS1-BRIDGE ####################

source /opt/ros/humble/setup.bash
source ./ros-humble-ros1-bridge/install/local_setup.bash
# export ROS_DOMAIN_ID=0
# export ROS_LOCALHOST_ONLY=1