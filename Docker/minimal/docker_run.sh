#!/usr/bin/env bash

ARGS=("$@")

REPOSITORY="jetseaai/ros2-bridge"
TAG="cpu"

IMG="${REPOSITORY}:${TAG}"

USER_NAME="arg"
REPO_NAME="ros-humble-ros1-bridge-builder"
CONTAINER_NAME="ros2-bridge-cpu"

CONTAINER_ID=$(docker ps -aqf "ancestor=${IMG}")
if [ $CONTAINER_ID ]; then
  echo "Attach to docker container $CONTAINER_ID"
  xhost +
  docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES="$(tput lines)" -it ${CONTAINER_ID} bash
  xhost -
  return
fi

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
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
  -e XAUTHORITY=$XAUTH \
  -e REPO_NAME=$REPO_NAME \
  -e HOME=/home/${USER_NAME} \
  -e OPENAI_API_KEY=$OPENAI_API_KEY\
  -v "$XAUTH:$XAUTH" \
  -v "/home/${USER}/${REPO_NAME}:/home/${USER_NAME}/${REPO_NAME}" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev:/dev" \
  -v "/var/run/docker.sock:/var/run/docker.sock" \
  --user "root:root" \
  --workdir "/home/${USER_NAME}/${REPO_NAME}" \
  --name "${CONTAINER_NAME}" \
  --network host \
  --privileged \
  --security-opt seccomp=unconfined \
  "${IMG}" \

  # -v "/home/${USER}/${REPO_NAME}/launch:/home/${USER_NAME}/${REPO_NAME}"/launch \
  # -v "/home/${USER}/${REPO_NAME}/share:/home/${USER_NAME}/${REPO_NAME}"/share \
# -e "TERM=xterm-256color" \
