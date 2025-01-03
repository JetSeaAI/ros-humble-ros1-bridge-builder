#!/usr/bin/env bash

BASH_OPTION=bash

IMG=argnctu/ros-humble-ros1-bridge-builder:jetson-orin

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}") && echo $containerid
docker exec -it \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e LINES="$(tput lines)" \
    ${containerid} \
    $BASH_OPTION
xhost -