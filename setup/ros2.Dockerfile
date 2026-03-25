##############################################
# Created from template ros2.dockerfile.jinja
##############################################

###########################################
# Base image
###########################################
FROM ubuntu:24.04 AS base

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
    ros-jazzy-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV COLCON_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages
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
  ros-jazzy-ament-* \
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
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-ros-move-group \
  ros-${ROS_DISTRO}-moveit-ros-visualization \
  ros-${ROS_DISTRO}-moveit-ros-warehouse \
  ros-${ROS_DISTRO}-moveit-setup-assistant \
  ros-${ROS_DISTRO}-moveit-simple-controller-manager \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-rviz-default-plugins \
  ros-${ROS_DISTRO}-ur-msgs \
  ros-${ROS_DISTRO}-ur-client-library \
  ros-${ROS_DISTRO}-joint-trajectory-controller \
  ros-${ROS_DISTRO}-ros2-controllers \
  && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib

###########################################
#  Full + deps image
###########################################
FROM full AS full-with-deps

ENV DEBIAN_FRONTEND=noninteractive
ARG ISAAC_ROS_APT_CHANNEL=release-4
ARG ISAAC_ROS_APT_COMPONENT=main

# Paquets ROS supplémentaires (hardware interface, joy, etc.)
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-${ROS_DISTRO}-joy \
  ros-${ROS_DISTRO}-teleop-twist-joy \
  ros-${ROS_DISTRO}-hardware-interface \
  ros-${ROS_DISTRO}-controller-interface \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-nav2-msgs \
  ros-${ROS_DISTRO}-pluginlib \
  ros-${ROS_DISTRO}-rosidl-default-generators \
  ros-${ROS_DISTRO}-xacro \
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
    && if apt-cache show ros-${ROS_DISTRO}-isaac-ros-cumotion-interfaces >/dev/null 2>&1; then \
         apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-isaac-ros-cumotion-interfaces; \
        else \
         echo "WARNING: ros-${ROS_DISTRO}-isaac-ros-cumotion-interfaces not found in configured apt sources."; \
        fi \
    && rm -rf /var/lib/apt/lists/*

# Install topic_based_ros2_control into an image-owned underlay when the distro
# package is unavailable. This survives the /ros2_ws bind mount at runtime.
WORKDIR /opt/vendor_ws
RUN mkdir -p /opt/vendor_ws/src && \
    if apt-cache show ros-${ROS_DISTRO}-topic-based-ros2-control >/dev/null 2>&1; then \
      apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-topic-based-ros2-control && rm -rf /var/lib/apt/lists/*; \
    else \
      git clone --depth 1 https://github.com/PickNikRobotics/topic_based_ros2_control.git /opt/vendor_ws/src/topic_based_ros2_control && \
      bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /opt/vendor_ws && colcon build --cmake-args -DBUILD_TESTING=OFF"; \
    fi

# Prépare l'espace de travail
WORKDIR /ros2_ws
COPY requirements.txt .

RUN apt-get update && apt-get install -y --no-install-recommends python3-vcstool

COPY UR_Robotiq.jazzy.repos /ros2_ws/UR_Robotiq.jazzy.repos
COPY patches/robotiq_activation_controller.cpp /tmp/robotiq_activation_controller.cpp
COPY patches/robotiq_driver/include/robotiq_driver/hardware_interface.hpp /tmp/robotiq_driver_hardware_interface.hpp
COPY patches/robotiq_driver/src/hardware_interface.cpp /tmp/robotiq_driver_hardware_interface.cpp

RUN mkdir -p /ros2_ws/src && \
    vcs import /ros2_ws/src < /ros2_ws/UR_Robotiq.jazzy.repos && \
    cp /tmp/robotiq_activation_controller.cpp \
      /ros2_ws/src/ros2_robotiq_gripper/robotiq_controllers/src/robotiq_activation_controller.cpp && \
    cp /tmp/robotiq_driver_hardware_interface.hpp \
      /ros2_ws/src/ros2_robotiq_gripper/robotiq_driver/include/robotiq_driver/hardware_interface.hpp && \
    cp /tmp/robotiq_driver_hardware_interface.cpp \
      /ros2_ws/src/ros2_robotiq_gripper/robotiq_driver/src/hardware_interface.cpp && \
    python3 -c "from pathlib import Path; p=Path('/ros2_ws/src/cartesian_controllers/cartesian_controller_base/CMakeLists.txt'); text=p.read_text(); text=text.replace('if($ENV{ROS_DISTRO} STREQUAL \"iron\")\n        set(CARTESIAN_CONTROLLERS_IRON TRUE)\nelseif($ENV{ROS_DISTRO} STREQUAL \"humble\")', 'if($ENV{ROS_DISTRO} STREQUAL \"iron\")\n        set(CARTESIAN_CONTROLLERS_IRON TRUE)\nelseif($ENV{ROS_DISTRO} STREQUAL \"jazzy\")\n        set(CARTESIAN_CONTROLLERS_IRON TRUE)\nelseif($ENV{ROS_DISTRO} STREQUAL \"humble\")'); text=text.replace('else()\n        message(WARNING \"ROS2 version must be {iron|humble|galactic|foxy}\")\nendif()', 'else()\n        message(WARNING \"ROS2 version must be {jazzy|iron|humble|galactic|foxy}; defaulting to Jazzy/Iron API compatibility\")\n        set(CARTESIAN_CONTROLLERS_IRON TRUE)\nendif()'); text=text.replace('else()\n        message(WARNING \"ROS2 version must be {jazzy|iron|humble|galactic|foxy}\")\nendif()', 'else()\n        message(WARNING \"ROS2 version must be {jazzy|iron|humble|galactic|foxy}; defaulting to Jazzy/Iron API compatibility\")\n        set(CARTESIAN_CONTROLLERS_IRON TRUE)\nendif()'); p.write_text(text)" && \
    rm -rf /ros2_ws/UR_Robotiq.jazzy.repos

# Dépendances Python
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed -r requirements.txt && \
    pip3 install --no-cache-dir --break-system-packages --ignore-installed keyboard


# Compilation
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --cmake-args -DBUILD_TESTING=OFF"

# Source automatique dans .bashrc de l’utilisateur ros
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ros/.bashrc && \
    echo "if [ -f /opt/vendor_ws/install/setup.bash ]; then source /opt/vendor_ws/install/setup.bash; fi" >> /home/ros/.bashrc && \
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
    ros-${ROS_DISTRO}-ros-gz \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=
