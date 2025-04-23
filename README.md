# ros-humble-ros1-bridge

ROS2-ROS1-Bridge Guide

## Dependencies

- [Robot Operating System (ROS) 2](http://docs.ros.org/en/humble/) (middleware for robotics),

**Important:** Ensure you have `ros-humble-desktop` installed on your system to avoid potential errors.

## Clone the repository

```bash
git clone git@github.com:JetSeaAI/ros-humble-ros1-bridge-builder.git
```

## Enter the repository

```bash
cd ros-humble-ros1-bridge-builder
```

## x86 Usage

You can pull the Docker image directly:

```bash
    source Docker/minimal/pull.sh
    source Docker/minimal/docker_run.sh
```

See the [Usage](#usage) section for instructions on how to use the bridge.

## Build the Docker image

We build without `ros-tutorials`.

x86:

```bash
  docker build . --build-arg ADD_ros_tutorials=0 -t jetseaai/ros-humble-ros1-bridge-builder:x86-cpu
```

Or use the build script:

```bash
source build_bridge_docker_builder.sh
```

Pending testing on ARM architecture...

Alternative builds:

```bash
  # **[OPTIONAL]** If you want to build ros-tutorials support:
  docker build . --build-arg ADD_ros_tutorials=1 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build grid-map support: (bridging the ros-humble-grid-map package)
  docker build . --build-arg ADD_grid_map=1 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build an example custom message:
  docker build . --build-arg ADD_example_custom_msgs=1 -t ros-humble-ros1-bridge-builder
```

- Note1: Don't forget to install the necessary `ros-humble-grid-map` packages on your ROS2 Humble system if you choose to build the bridge with the `grid-map` support added.

## Create the package

Use the build script to create the "ros-humble-ros1-bridge/" ROS2 package. The package will be created, moved to the parent folder, and a symbolic link will be created in the current folder:

x86:

```bash
    source build_bridge_package.sh
```

## Create the Docker environment to run the bridge

Use the script to build the ros1-bridge environment:

```bash
    source Docker/minimal/build.sh
```

## Run the Docker environment including the ros1-bridge package

```bash
    source Docker/minimal/docker_run.sh
```

Pending testing on ARM architecture...

- Note0: If you are running this bridge **without Docker**, it's **important** that you have **`ros-humble-desktop`** installed on your ROS2 Humble system because we want to **match it with the builder image as closely as possible**.

Otherwise, you may get an error about missing `libexample_interfaces__rosidl_typesupport_cpp.so`. See the issue [here](https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/issues/10).

- Note1: There is no compilation at this point. The `docker run` command simply outputs a pre-compiled tarball for either amd64 or arm64 architecture, depending on the architecture of the machine you used to create the builder image.

- Note2: The assumption is that this tarball contains configurations and libraries matching your ROS2 Humble system very closely, although not identical.

- Note3: We don't really need the builder image anymore. To delete it, run:

```bash
    docker rmi ros-humble-ros1-bridge-builder
```

## Usage

On an Ubuntu 22.04 (Jammy) ROS 2 Humble Desktop system or in Docker, replace `[IP]` with your ROS1 Master IP:

```bash
    source environment_ros2_ros1_bridge.sh [IP]
```

Then run `dynamic_bridge` to bridge all topics:

```bash
    source bridge.sh
```
