#!/usr/bin/env bash

REPOSITORY="jetseaai/ros2-bridge"
TAG="cpu"

IMG="${REPOSITORY}:${TAG}"

docker pull "${IMG}"
