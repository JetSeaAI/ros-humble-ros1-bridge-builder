REPOSITORY="jetseaai/ros-humble-ros1-bridge-builder"
TAG="x86-cpu"

IMG="${REPOSITORY}:${TAG}"

docker run --rm ${IMG} | tar xvzf -