# ros-humble-ros1-bridge

ROS2-ROS1-Bridge Guide

## Clone repo

```bash
git clone git@github.com:JetSeaAI/ros-humble-ros1-bridge-builder.git
```

## Update repo and submodules

```bash
git pull
git submodule sync --recursive
git submodule update --init --recursive
```

## Enter the repo

```bash
cd ros-humble-ros1-bridge-builder
```

## Build the docker

We build without ros-tutorals.

x86:

```bash
  docker build . --build-arg ADD_ros_tutorials=0 - -t jetseaai/ros-humble-ros1-bridge-builder:x86-cpu
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

0.) Start from the latest Ubuntu 22.04 (Jammy) ROS 2 Humble Desktop system, create the "ros-humble-ros1-bridge/" ROS2 package:

``` bash
    docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
```

- Note1: It's **important** that you have **`ros-humble-desktop`** installed on your ROS2 Humble system because we want to **match it with the builder image as closely as possible**.

Otherwise you may get an error about missing `ibexample_interfaces__rosidl_typesupport_cpp.so`.  See issue https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/issues/10

- Note1: There is no compilation at this point, the `docker run` command simply spits out a pre-compiled tarball for either amd64 or arm64 architecture, depending on the architecture of the machine you used to created the builder image.

- Note2: The assumption is that this tarball contains configurations and libraries matching your ROS2 Humble system very closely, although not identical.

- Note3: We don't really need the builder image anymore, to delete it, do:

``` bash
    docker rmi ros-humble-ros1-bridge-builder
```

## Usage

### Terminal 1: ROSCORE

1. Docker Run

Run this script to pull docker image to your workstation.

```bash
source Docker/jetson-orin/run.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Roscore

```bash
roscore
```
### Terminal 2: ROS1 & ROS2 BRIDGE

1. Docker Run

```bash
source Docker/jetson-orin/run.sh
```

2. Source ROS1 & ROS2 Environment

```bash
source environment_ros2_ros1_bridge.sh
```

3. Run Ros1_bridge

```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### Download bags

Please download bags folder and put in the root path of the repo: [Link](http://gofile.me/773h8/WtbSM5xSh) 

### Terminal 3: ROS1

1. Docker Run

```bash
source Docker/jetson-orin/run.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Run ROS1 package

Image Bag
```bash
cd bags/images-2024-11-23-15-28-41/
rosbag play 2024-11-23-15-28-41.bag 
```

Pose & Layser Bag
```bash
cd bags/1123_1528/
rosbag play 2024-11-23-15-28-44_0.bag 
```

### Terminal 4: ROS2

1. Docker Run

```bash
source Docker/jetson-orin/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Run ROS2 Bag Example

Radar Bag
```bash
cd bags/recorded_rosbag_halo_20241123-141728/
ros2 bag play recorded_rosbag_halo_20241123-141728_0.db3
```

### Terminal 5
1. Docker Run

```bash
source Docker/jetson-orin/run.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Rviz

```bash
rviz -d rviz/radar-example.rviz
```

### Terminal 6
1. Docker Run

```bash
source Docker/jetson-orin/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Rviz2

```bash
rviz2 -d rviz2/lidar-camera-example.rviz
```

