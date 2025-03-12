#!/bin/bash

# Build the Docker image from personalrobotics/ros-openrave
docker build -t openrave_ikfast \
    --build-arg USER_ID=$(id -u) \
    --build-arg GROUP_ID=$(id -g) \
    -f Dockerfile .