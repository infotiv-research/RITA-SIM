FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
      locales curl gnupg lsb-release ca-certificates software-properties-common \
    && locale-gen en_US en_US.UTF-8 && update-locale LANG=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-xacro \
      python3-colcon-common-extensions \
      python3-rosdep \
      build-essential git \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update || true

RUN printf '%s\n' '. /opt/ros/jazzy/setup.bash || true' > /etc/profile.d/ros2.sh
RUN echo '. /opt/ros/jazzy/setup.bash || true' >> /root/.bashrc

WORKDIR /root
SHELL ["/bin/bash","-lc"]
