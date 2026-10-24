# ZED cameras -> ROS 2 (Humble), so this PC no longer depends on another user's ZED launcher.
# Publishes the same /zed_multi/<name>/... topics the lab's zed_tools/launch_all.sh did (see docker/zed_cameras.sh).
#
#   docker build -t zed-ros2 -f docker/zed.Dockerfile docker
#
# ZED SDK 4.2.5 (CUDA 12.1 build; SDK 4.2 still supports the first-generation ZED) + zed-ros2-wrapper humble-v4.2.5
# (the wrapper version must match the SDK). Base: the CUDA 12 devel image already on this machine; the SDK runs on
# any CUDA 12.x runtime. The SDK's calibration files and AI models are cached on a mounted folder (zed_cameras.sh).

FROM nvidia/cuda:12.8.1-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 NVIDIA_DRIVER_CAPABILITIES=all
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl wget gnupg2 lsb-release ca-certificates zstd less udev sudo apt-transport-https git \
        build-essential cmake python3-pip python3-dev libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble (ros-base) + build tools
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ZED SDK (installer flags as in Stereolabs' own Dockerfiles; CUDA comes from the base image)
ARG ZED_SDK_URL=https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu22
RUN wget -q -O /tmp/zed_sdk.run "$ZED_SDK_URL" && chmod +x /tmp/zed_sdk.run \
    && /tmp/zed_sdk.run -- silent skip_cuda \
    && rm -f /tmp/zed_sdk.run \
    && ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so \
    && rm -rf /var/lib/apt/lists/*

# ZED ROS 2 wrapper
WORKDIR /ros2_ws
RUN git clone --recursive --depth 1 --branch humble-v4.2.5 https://github.com/stereolabs/zed-ros2-wrapper.git \
        src/zed-ros2-wrapper
RUN apt-get update && (rosdep init || true) && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/humble/setup.bash && source /ros2_ws/install/local_setup.bash" >> /etc/bash.bashrc
CMD ["bash"]
