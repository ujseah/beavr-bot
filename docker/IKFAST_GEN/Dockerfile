FROM personalrobotics/ros-openrave

# Update and install additional packages if needed
RUN apt-get update && \
    apt-get install -y sudo

# Create workspace directory
RUN mkdir -p /workspace

# Set working directory
WORKDIR /workspace

# Keep container running
CMD ["/bin/bash"]