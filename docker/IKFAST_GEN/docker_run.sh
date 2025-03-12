#!/bin/bash

# Get the current directory path
CURRENT_DIR=$(pwd)

# Run the Docker container
docker run -it \
    --name openrave_container \
    -v ${CURRENT_DIR}:/workspace \
    -e DISPLAY=${DISPLAY} \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network host \
    --rm \
    personalrobotics/ros-openrave