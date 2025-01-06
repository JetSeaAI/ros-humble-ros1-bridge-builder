REPOSITORY="jetseaai/ros-humble-ros1-bridge-builder"
TAG="x86-cpu"

IMG="${REPOSITORY}:${TAG}"

USER_NAME="arg"
PROJ_NAME="ros-humble-ros1-bridge-builder"
CONTAINER_NAME="ros-humble-ros1-bridge"


docker run --rm ${IMG} | tar xvzf -