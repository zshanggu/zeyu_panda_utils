# Workstation-side environment for deoxys control, teleop and data collection (ROS 2 Humble).
# deoxys_control and zeyu_panda_utils are NOT baked in: run.sh mounts them from the host so edits
# take effect without a rebuild. The NUC keeps running bin/franka-interface + gripper-interface.
#
# Build:  docker build -t panda-deoxys -f docker/Dockerfile docker
# Run:    docker/run.sh [command ...]

FROM ros:humble-ros-base

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        python3-pip libhidapi-hidraw0 libhidapi-libusb0 libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# deoxys runtime deps (pybullet is imported by franka_interface, glfw by io_devices), pinned
# protobuf to match the protoc 3.13 bindings in deoxys/proto, and numpy<2 for ROS/numba.
RUN pip3 install --no-cache-dir \
        "numpy<2" "protobuf==3.20.0" pyzmq PyYAML easydict termcolor tqdm numba glfw hidapi pybullet \
        h5py opencv-python

ENV PYTHONPATH=/data/zeyu/Deformable/deoxys_control/deoxys:/data/zeyu/Deformable/zeyu_panda_utils/src
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
WORKDIR /data/zeyu/Deformable/zeyu_panda_utils
CMD ["bash"]
