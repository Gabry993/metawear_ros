FROM ros:foxy
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    locales \
    python3-pip \
    build-essential \
    bluetooth \
    bluez \
    libbluetooth-dev \
    libboost-all-dev \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# COPY ./version.txt ./version.txt

RUN python3 -m pip install metawear pymetawear numpy-quaternion

COPY ./ /ros_ws/src/metawear_ros

RUN cd /ros_ws && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select metawear_ros && \
  rm -r build log src

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/ros_ws\/install\/setup.bash"' \
  /ros_entrypoint.sh
