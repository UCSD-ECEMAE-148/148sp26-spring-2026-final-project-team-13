# Dockerfile (ROS 2 Humble camera container)
# Default to ARM64 so this builds on Jetson without qemu/binfmt emulation.
# Override for x86_64 if needed, e.g.:
#   docker build --build-arg ROS_BASE_IMAGE=osrf/ros:humble-ros-base-jammy .
ARG ROS_DISTRO=humble
ARG ROS_BASE_IMAGE=arm64v8/ros:humble-ros-base-jammy
FROM ${ROS_BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# 1) Base build tools + camera-node deps + CycloneDDS RMW
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-yaml \
    build-essential \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# 2) Install Python packaging tooling compatible with ament_python on Humble,
#    then install the DepthAI SDK from PyPI.
RUN pip3 install --no-cache-dir "setuptools==58.2.0" wheel && \
    pip3 install --no-cache-dir --prefer-binary "depthai<3"

# Use CycloneDDS as the RMW for host-container topic visibility
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

# 3) Create workspace
WORKDIR /ws
RUN mkdir -p src

# Keep a helper run script in the default shell directory.
COPY run.sh /ws/run.sh
RUN chmod +x /ws/run.sh

# 4) Copy your ROS2 packages into src/
#    (adjust path if your repo layout differs)
COPY . /ws/src/

# 5) Resolve package deps.
#    DepthAI itself is installed with pip above because it does not have a
#    reliable apt/rosdep path on Humble ARM builds.
RUN apt-get update && (rosdep init 2>/dev/null || true) && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys=python3-depthai && \
    rm -rf /var/lib/apt/lists/*

# 6) Build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install

# 7) Container startup: source ROS + workspace, then launch
# CMD ["bash", "-lc", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ws/install/setup.bash && ros2 launch <your_package> <your_launch>.launch.py"]
