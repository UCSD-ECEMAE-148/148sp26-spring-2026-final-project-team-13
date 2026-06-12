FROM ros:humble-ros-base

LABEL maintainer="Clemente Donoso <clemente.donosok@gmail.com>"
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# CPU-only runtime for Raspberry Pi 5. No CUDA, CuPy, TensorRT, or Jetson base.

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    python3-yaml \
    python3-opencv \
    ros-humble-ament-index-python \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-sensor-msgs \
    ros-humble-sensor-msgs-py \
    ros-humble-foxglove-bridge \
    ros-humble-launch-xml \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf2-ros \
    build-essential \
    git \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libopenblas-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel
RUN python3 -m pip install --no-cache-dir \
    pyyaml \
    open3d \
    depthai==2.29.0
RUN python3 -m pip install --no-cache-dir "numpy<2" torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --no-cache-dir "numpy<2" "opencv-python<4.10" ultralytics

WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Copy the fusion package into the workspace build context.
COPY . /ros2_ws/src/ros2_camera_lidar_fusion

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]