ARG BASE_IMAGE=nvidia/cuda:12.2.2-devel-ubuntu22.04
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ARG ISAAC_ROS_APT_CHANNEL=release-3
ARG ISAAC_ROS_APT_COMPONENT=release-3.0
ARG TORCH_WHL_CHANNEL=cu121
ARG TORCH_VERSION=2.2.2
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV PIP_DEFAULT_TIMEOUT=300
ENV PIP_RETRIES=10

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    curl \
    git \
    gnupg2 \
    lsb-release \
    locales \
    ninja-build \
    python3-pip \
    qtbase5-dev \
    software-properties-common \
    sudo \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-trajectory-controller \
    ros-humble-joy \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-nav2-msgs \
    ros-humble-pluginlib \
    ros-humble-robot-state-publisher \
    ros-humble-cv-bridge \
    ros-humble-rosidl-default-generators \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz-rendering \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-teleop-twist-joy \
    ros-humble-tf2-ros \
    ros-humble-topic-based-ros2-control \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update \
    && keyring=/usr/share/keyrings/nvidia-isaac-ros.gpg \
    && curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
       | gpg --dearmor -o ${keyring} \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=${keyring}] https://isaac.download.nvidia.com/isaac-ros/${ISAAC_ROS_APT_CHANNEL} $(. /etc/os-release && echo ${UBUNTU_CODENAME}) ${ISAAC_ROS_APT_COMPONENT}" \
       > /etc/apt/sources.list.d/nvidia-isaac-ros.list \
    && apt-get update \
    && install_pkgs="" \
    && for pkg in \
         ros-humble-isaac-ros-cumotion \
         ros-humble-isaac-ros-cumotion-moveit \
         ros-humble-isaac-ros-cumotion-interfaces; do \
         if apt-cache show "$pkg" >/dev/null 2>&1; then \
           install_pkgs="${install_pkgs} ${pkg}"; \
         fi; \
       done \
    && if [ -n "${install_pkgs}" ]; then \
         apt-get install -y --no-install-recommends ${install_pkgs}; \
       else \
         echo "ERROR: cuMotion binary package was not found in apt sources." && exit 1; \
       fi \
    && rm -rf /var/lib/apt/lists/*


RUN pip3 install --no-cache-dir \
    --timeout ${PIP_DEFAULT_TIMEOUT} \
    --retries ${PIP_RETRIES} \
    --index-url https://download.pytorch.org/whl/${TORCH_WHL_CHANNEL} \
    torch==${TORCH_VERSION}

RUN pip3 install --no-cache-dir \
    --timeout ${PIP_DEFAULT_TIMEOUT} \
    --retries ${PIP_RETRIES} \
    warp-lang

RUN pip3 install --no-cache-dir \
    --timeout ${PIP_DEFAULT_TIMEOUT} \
    --retries ${PIP_RETRIES} \
    open3d

RUN pip3 install --no-cache-dir \
    --timeout ${PIP_DEFAULT_TIMEOUT} \
    --retries ${PIP_RETRIES} \
    ros2-numpy \
    transforms3d \
    pandas \
    scikit-learn \
    pyarrow

RUN pip3 install --no-cache-dir \
    --timeout ${PIP_DEFAULT_TIMEOUT} \
    --retries ${PIP_RETRIES} \
    trimesh \
    scipy \
    yourdfpy \
    tqdm \
    "numpy<2"

RUN pip3 install --no-cache-dir \
    --timeout ${PIP_DEFAULT_TIMEOUT} \
    --retries ${PIP_RETRIES} \
    keyboard

RUN rosdep init || true

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && ros2 pkg prefix isaac_ros_cumotion >/dev/null \
    && ros2 pkg prefix isaac_ros_cumotion_moveit >/dev/null \
    && python3 -c "import torch; import warp"

RUN if getent passwd ubuntu > /dev/null 2>&1; then \
      userdel -r ubuntu || true; \
    fi \
    && if ! getent group ${USER_GID} > /dev/null 2>&1; then \
         groupadd --gid ${USER_GID} ${USERNAME}; \
       fi \
    && existing_uid_user="$(getent passwd ${USER_UID} | cut -d: -f1 || true)" \
    && if [ -n "${existing_uid_user}" ] && [ "${existing_uid_user}" != "${USERNAME}" ]; then \
         userdel -r "${existing_uid_user}" || true; \
       fi \
    && if id -u ${USERNAME} > /dev/null 2>&1; then \
         usermod --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; \
       else \
         useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
       fi \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

ENV DEBIAN_FRONTEND=
WORKDIR /ros2_ws
USER ${USERNAME}
CMD ["bash"]
