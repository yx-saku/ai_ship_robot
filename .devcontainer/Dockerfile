ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Tokyo \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# # プロキシ下で実行する際に証明書を信頼済みとして登録
# RUN mkdir -p /usr/share/ca-certificates/cato
# COPY [ "./cate_root_ca.crt", "/usr/share/ca-certificates/cato/ca.crt" ]
# RUN echo "cato/ca.crt" >> /etc/ca-certificates.conf \
#     && update-ca-certificates

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        curl \
        locales \
        sudo \
        tzdata \
        git \
    && locale-gen en_US en_US.UTF-8 ja_JP.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN install -m 0755 -d /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /etc/apt/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
        > /etc/apt/sources.list.d/ros2.list

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-gazebo-plugins \
        ros-${ROS_DISTRO}-gazebo-ros \
        ros-${ROS_DISTRO}-launch \
        ros-${ROS_DISTRO}-launch-ros \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-ros2launch \
        ros-${ROS_DISTRO}-ros2run \
        ros-${ROS_DISTRO}-ros2topic \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-xacro \
    && (rosdep init || true) \
    && rm -rf /var/lib/apt/lists/*

RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd --uid ${USER_UID} --gid ${USER_GID} --shell /bin/bash -m ${USERNAME}; fi \
    && usermod --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "if [ -f /workspaces/ai_ship_robot/scripts/setup_workspace.sh ]; then source /workspaces/ai_ship_robot/scripts/setup_workspace.sh; else source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /etc/bash.bashrc

USER ${USERNAME}
WORKDIR /workspaces/ai_ship_robot
