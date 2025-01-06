#!/usr/bin/env bash

ARGS=("$@")

# project variable
REPOSITORY="jetseaai/ros-humble-ros1-bridge-builder"
TAG="x86-cpu"

IMG="${REPOSITORY}:${TAG}"

USER_NAME="arg"
PROJ_NAME="ros-humble-ros1-bridge-builder"
CONTAINER_NAME="ros-humble-ros1-bridge"
# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)

CONTAINER_ID=$(docker ps -aqf "ancestor=${IMG}")
if [ $CONTAINER_ID ]; then
  echo "Attach to docker container $CONTAINER_ID"
  xhost +
  docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES="$(tput lines)" -it ${CONTAINER_ID} bash
  xhost -
  return
fi

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<<"$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi

docker run \
    -it \
    --rm \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e HOME=/home/${USER_NAME} \
    -e OPENAI_API_KEY=$OPENAI_API_KEY\
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
    -e PYTHONPATH=/home/arg/$PROJ_NAME \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "/var/run/docker.sock:/var/run/docker.sock" \
    -v "/home/$USER/$PROJ_NAME:/home/${USER_NAME}/$PROJ_NAME" \
    -w "/home/${USER_NAME}/$PROJ_NAME" \
    --user "root:root" \
    --name "${CONTAINER_NAME}" \
    --network host \
    --privileged \
    --security-opt seccomp=unconfined \
    "${IMG}"| tar xvzf - \
