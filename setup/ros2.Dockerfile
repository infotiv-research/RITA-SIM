##############################################
# Created from template ros2.dockerfile.jinja
##############################################

###########################################
# Base image
###########################################
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y --no-install-recommends \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y --no-install-recommends tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV DEBIAN_FRONTEND=
###########################################
#  Develop image
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  qtbase5-dev \
  ros-dev-tools \
  ros-humble-ament-* \
  vim \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Check if "ubuntu" user exists, delete it if it does, then create the desired user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN apt-get update && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
RUN apt-get update && apt-get install -y --no-install-recommends git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

###########################################
#  Full image
###########################################
FROM dev AS full

ENV DEBIAN_FRONTEND=noninteractive
# Install the full release
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-ros-warehouse \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-ros2-control \
  ros-humble-controller-manager \
  ros-humble-topic-based-ros2-control \
  ros-humble-joint-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-rviz-default-plugins \
  ros-humble-ur-msgs \
  ros-humble-ur-client-library \
  ros-humble-joint-trajectory-controller \
  ros-humble-ros2-controllers \
  && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=
ENV LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib

###########################################
#  Full + deps image
###########################################
FROM full AS full-with-deps

ENV DEBIAN_FRONTEND=noninteractive
ARG ISAAC_ROS_APT_CHANNEL=release-3
ARG ISAAC_ROS_APT_COMPONENT=release-3.0

# Paquets ROS supplémentaires (hardware interface, joy, etc.)
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-hardware-interface \
  ros-humble-controller-interface \
  ros-humble-cv-bridge \
  ros-humble-nav2-msgs \
  ros-humble-pluginlib \
  ros-humble-rosidl-default-generators \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-pip \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && keyring=/usr/share/keyrings/nvidia-isaac-ros.gpg \
    && curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
       | gpg --dearmor -o ${keyring} \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=${keyring}] https://isaac.download.nvidia.com/isaac-ros/${ISAAC_ROS_APT_CHANNEL} $(. /etc/os-release && echo ${UBUNTU_CODENAME}) ${ISAAC_ROS_APT_COMPONENT}" \
       > /etc/apt/sources.list.d/nvidia-isaac-ros.list \
    && apt-get update \
    && if apt-cache show ros-humble-isaac-ros-cumotion-interfaces >/dev/null 2>&1; then \
         apt-get install -y --no-install-recommends ros-humble-isaac-ros-cumotion-interfaces; \
       else \
         echo "WARNING: ros-humble-isaac-ros-cumotion-interfaces not found in configured apt sources."; \
       fi \
    && rm -rf /var/lib/apt/lists/*

# Prépare l'espace de travail
WORKDIR /ros2_ws
COPY requirements.txt .

RUN apt-get update && apt-get install -y --no-install-recommends python3-vcstool

COPY UR_Robotiq.humble.repos /ros2_ws/UR_Robotiq.humble.repos
COPY patches/robotiq_activation_controller.cpp /tmp/robotiq_activation_controller.cpp
COPY patches/robotiq_driver/include/robotiq_driver/hardware_interface.hpp /tmp/robotiq_driver_hardware_interface.hpp
COPY patches/robotiq_driver/src/hardware_interface.cpp /tmp/robotiq_driver_hardware_interface.cpp

RUN mkdir -p /ros2_ws/src && \
    vcs import /ros2_ws/src < /ros2_ws/UR_Robotiq.humble.repos && \
    cp /tmp/robotiq_activation_controller.cpp \
      /ros2_ws/src/ros2_robotiq_gripper/robotiq_controllers/src/robotiq_activation_controller.cpp && \
    cp /tmp/robotiq_driver_hardware_interface.hpp \
      /ros2_ws/src/ros2_robotiq_gripper/robotiq_driver/include/robotiq_driver/hardware_interface.hpp && \
    cp /tmp/robotiq_driver_hardware_interface.cpp \
      /ros2_ws/src/ros2_robotiq_gripper/robotiq_driver/src/hardware_interface.cpp && \
    rm -rf /ros2_ws/UR_Robotiq.humble.repos

# Dépendances Python
RUN pip3 install --no-cache-dir -r requirements.txt && \
    pip3 install --no-cache-dir keyboard


# Compilation
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source automatique dans .bashrc de l’utilisateur ros
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /home/ros/.bashrc && \
    chown ros:ros /home/ros/.bashrc

USER ros
WORKDIR /ros2_ws
CMD ["bash"]


###########################################
#  Full+Gazebo image
###########################################
FROM full AS gazebo

ENV DEBIAN_FRONTEND=noninteractive
# Install gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    ros-humble-ros-gz \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=
