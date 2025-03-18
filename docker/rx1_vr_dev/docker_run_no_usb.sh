#!/bin/bash

# Allow local GUI forwarding
xhost +local:docker

# Define the USB device
USB_DEVICE="/dev/ttyACM0"

# Check if the USB device exists
# if [ ! -e "$USB_DEVICE" ]; then
#     # Print a warning in red
#    echo -e "\033[0;31mWARNING: USB device $USB_DEVICE not found. Starting container without device access.\033[0m"
#    DEVICE_OPTION=""  # Leave the device option empty if not found
# else
#    DEVICE_OPTION="--device=$USB_DEVICE:$USB_DEVICE"
# fi

# Run the Docker container interactively with GPU and GUI support
docker run -it \
    --gpus all \
    --name rx1-vr \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    $DEVICE_OPTION \
    --net=host \
    --privileged \
    rx1-vr

