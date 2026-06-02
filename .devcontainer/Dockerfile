FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    ros-humble-diagnostic-msgs \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true

# Auto-source ROS 2 for interactive terminals.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /workspaces
