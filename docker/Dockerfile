FROM ros:galactic
MAINTAINER Gabriele Abbate "gabriele.abbate@supsi.ch"

RUN apt-get update && apt-get install -y \
    locales \
    python3-pip \
    build-essential \
    bluetooth \
    bluez \
    libbluetooth-dev \
    libboost-all-dev \
    libudev-dev \
    python3-pykdl \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# COPY ./version.txt ./version.txt

RUN python3 -m pip install "warble==1.2.6" "metawear==0.7.0" "pymetawear==0.12.0" numpy-quaternion

COPY ./metawear_ros/ /ros_ws/src/metawear_ros

# RUN mkdir -p /ros_ws/src/ && cd /ros_ws/src/ && git clone -b ros2 https://github.com/Gabry993/metawear_ros.git

RUN cd /ros_ws && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select metawear_ros && \
  rm -r build log src

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/ros_ws\/install\/setup.bash"' \
  /ros_entrypoint.sh
