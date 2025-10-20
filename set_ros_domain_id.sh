#!/bin/bash
# Check if a parameter is provided, otherwise prompt the user.
if [ "$#" -eq 0 ]; then
    read -p "Enter ROS_DOMAIN_ID: " ROS_DOMAIN_ID
else
    ROS_DOMAIN_ID=$1
fi

export ROS_DOMAIN_ID
echo "ROS_DOMAIN_ID is set to ${ROS_DOMAIN_ID}"
