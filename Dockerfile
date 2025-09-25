# ROS Noetic Dockerfile for dcd_timeref development
FROM ros:noetic-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    # ROS build tools
    python3-catkin-tools \
    python3-catkin-pkg \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    # Development tools
    gdb \
    valgrind \
    htop \
    tree \
    # Install catkin_make
    ros-noetic-catkin \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Initialize rosdep
RUN rosdep update

# Create catkin workspace
RUN mkdir -p /workspace/catkin_ws/src

# Set up ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /workspace/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set working directory to catkin workspace
WORKDIR /workspace/catkin_ws

# Source the workspace
RUN echo "source /workspace/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set default command
CMD ["/bin/bash"]
