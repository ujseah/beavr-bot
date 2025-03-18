#!/bin/bash

# Start the container if not already running
docker start rx1-vr-usb 2>/dev/null

# Attach to the running container
docker exec -it rx1-vr-usb bash
