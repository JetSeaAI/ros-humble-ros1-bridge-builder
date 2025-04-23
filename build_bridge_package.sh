REPOSITORY="jetseaai/ros-humble-ros1-bridge-builder"
TAG="x86-cpu"

IMG="${REPOSITORY}:${TAG}"

echo "Running Docker container: ${IMG}..."
docker run --rm ${IMG} | tar xvzf - || { echo "Extraction failed. Exiting."; exit 1; }
echo "Extraction completed."

echo "Checking if 'ros-humble-ros1-bridge/' exists in the parent directory..."
if [ -d "../ros-humble-ros1-bridge" ]; then
    echo "'ros-humble-ros1-bridge/' already exists. Removing it to overwrite..."
    rm -rf ../ros-humble-ros1-bridge
fi

echo "Moving 'ros-humble-ros1-bridge/' to the parent directory..."
mv ros-humble-ros1-bridge/ ../
echo "Move completed. The package is now in the parent directory."
echo "Creating symbolic link to the package in the current directory..."
ln -s ../ros-humble-ros1-bridge ros-humble-ros1-bridge
echo "Symbolic link created."
