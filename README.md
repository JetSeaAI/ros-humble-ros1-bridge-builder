# ros-humble-ros1-bridge

ROS2-ROS1-Bridge Guide

## Dependencies

- [Robot Operating System (ROS) 2](http://docs.ros.org/en/humble/) (middleware for robotics),

**Important:** Ensure you have `ros-humble-desktop` installed on your system to avoid potential errors.

## Clone repo

```bash
git clone git@github.com:JetSeaAI/ros-humble-ros1-bridge-builder.git
```

## Enter the repo

```bash
cd ros-humble-ros1-bridge-builder
```

## Build the docker

We build without ros-tutorals.

x86:

```bash
  docker build . --build-arg ADD_ros_tutorials=0 -t jetseaai/ros-humble-ros1-bridge-builder:x86-cpu
```

Or using build script:

```bash
source build_bridge_docker.sh
```

Pending testing on ARM architecture....

Alternative builds:

``` bash
  # **[OPTIONAL]** If you want to build ros-tutorals support:
  docker build . --build-arg ADD_ros_tutorials=1 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build grid-map support:  (bridging the ros-humble-grid-map package)
  docker build . --build-arg ADD_grid_map=1 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build an example custom message:
  docker build . --build-arg ADD_example_custom_msgs=1 -t ros-humble-ros1-bridge-builder
```

- Note1: Don't forget to install the necessary `ros-humble-grid-map` packages on your ROS2 Humble if you choose to build the bridge with the `grid-map` support added.

## Create the package

Start from the latest Ubuntu 22.04 (Jammy) ROS 2 Humble Desktop system, create the "ros-humble-ros1-bridge/" ROS2 package:

``` bash
    docker run --rm jetseaai/ros-humble-ros1-bridge-builder:x86-cpu | tar xvzf -
```

Or using build script:

x86:

```bash
    source build_bridge_package.sh
```

Pending testing on ARM architecture....

- Note1: It's **important** that you have **`ros-humble-desktop`** installed on your ROS2 Humble system because we want to **match it with the builder image as closely as possible**.

Otherwise you may get an error about missing `ibexample_interfaces__rosidl_typesupport_cpp.so`. See issue [here](https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/issues/10)

- Note1: There is no compilation at this point, the `docker run` command simply spits out a pre-compiled tarball for either amd64 or arm64 architecture, depending on the architecture of the machine you used to created the builder image.

- Note2: The assumption is that this tarball contains configurations and libraries matching your ROS2 Humble system very closely, although not identical.

- Note3: We don't really need the builder image anymore, to delete it, do:

``` bash
    docker rmi ros-humble-ros1-bridge-builder
```

## Usage

On Ubuntu 22.04 (Jammy) ROS 2 Humble Desktop system, and replace the [IP] to your ROS1 Master IP.

```bash
    source enviroment_ros2_ros1_bridge.sh [IP]
```

and run dynamic_bridge to bridge all topics.

```bash
    source bridge.sh
```
