REPOSITORY="jetseaai/ros-humble-ros1-bridge-builder"
TAG="x86-cpu"

IMG="${REPOSITORY}:${TAG}"


 docker build . --build-arg ADD_ros_tutorials=0 -t ${IMG}