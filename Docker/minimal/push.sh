#!/usr/bin/env bash

REPOSITORY="jetseaai/ros2-bridge"
TAG="cpu"


LASTEST_IMG="${REPOSITORY}:latest"
IMG="${REPOSITORY}:${TAG}"

docker image push "${IMG}"
docker image push "${LASTEST_IMG}"