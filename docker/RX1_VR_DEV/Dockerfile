# Use the official ROS Noetic image as the base
FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools, GUI dependencies, and Python
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    git \
    net-tools \
    iputils-ping \
    x11-apps \
    python3-pip \
    python3-venv \
    psmisc \
    && apt-get clean

# Install dependencies
RUN apt-get update && apt-get install -y \
    libnlopt-dev \
    liborocos-kdl-dev \
    ros-noetic-kdl-parser \
    ros-noetic-kdl-conversions \
    ros-noetic-tf2-kdl \
    libeigen3-dev \
    && apt-get clean

# Remove pre-installed trac_ik packages to avoid conflicts
RUN apt-get remove -y ros-noetic-trac-ik ros-noetic-trac-ik-kinematics-plugin || true

RUN apt-get update

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update

# Set up the ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
SHELL ["/bin/bash", "-c"]

# Enable GUI forwarding
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Add NVIDIA ICD and EGL configuration
COPY nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json
COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Set NVIDIA environment variables
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

# Install NVIDIA drivers and OpenGL libraries
RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-xserver-utils \
    vulkan-utils \
    && apt-get clean

# Install Vulkan support (optional, for advanced graphics)
RUN apt-get update && apt-get install -y \
    libvulkan1 \
    && apt-get clean
    
RUN apt-get update && apt-get install python3-tk -y 

# Install Python packages
RUN pip3 install blosc pyserial

# Create a Catkin workspace
RUN mkdir -p /home/catkin_ws/src

# Set working directory
WORKDIR /home/catkin_ws

# Clone the necessary repositories into the src directory
RUN git clone https://bitbucket.org/traclabs/trac_ik.git src/trac_ik
RUN git clone https://github.com/Red-Rabbit-Robotics/feetech_lib.git src/feetech_lib
RUN git clone https://github.com/Red-Rabbit-Robotics/ik_solver_lib.git src/ik_solver_lib

# Our forked RX1
RUN git clone --depth 1 https://github.com/ARCLab-MIT/rx1 src/rx1

# Ensure updated packages
RUN rosdep update

# Install dependencies using rosdep
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

# Source the workspace setup script
RUN echo "source /home/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Keep the container running
CMD ["bash", "-c", "roscore > /dev/null 2>&1 & exec bash"]

