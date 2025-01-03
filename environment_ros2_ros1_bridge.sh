#! /bin/bash

#################### ROS1 ####################

source environment_ros1.sh

#################### ROS2-ROS1-BRIDGE ####################

source /opt/ros/humble/setup.bash
source ~/ros-humble-ros1-bridge/install/local_setup.bash
export ROS_DOMAIN_ID=0
# export ROS_LOCALHOST_ONLY=1