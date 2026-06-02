FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# rosdep may already be initialized in some apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    python3-pip \
    python3-colcon-common- ROS images.
RUN rosdep init 2>/dev/null || true
RUN rosdep update || true

# Make ROS 2 available in every interactive shell.
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
