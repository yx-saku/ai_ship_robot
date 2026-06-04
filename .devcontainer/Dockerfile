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

# JetPack 6のベースに合わせてUbuntu 22.04から開始し、ROS 2 Humbleは追加依存スクリプトとは別段階で導入する。
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg \
        locales \
        software-properties-common \
        tzdata \
    && add-apt-repository universe \
    && locale-gen en_US en_US.UTF-8 ja_JP.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && install -m 0755 -d /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /etc/apt/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
    && rm -rf /var/lib/apt/lists/*

# # プロキシ下で実行する際に証明書を信頼済みとして登録
# RUN mkdir -p /usr/share/ca-certificates/cato
# COPY [ "./cate_root_ca.crt", "/usr/share/ca-certificates/cato/ca.crt" ]
# RUN echo "cato/ca.crt" >> /etc/ca-certificates.conf
# #  update all
# RUN update-ca-certificates

# 本番Jetsonと同じインストールスクリプトを使い、Docker build時はworkspaceを触らずsystem依存だけを入れる。
COPY scripts/install_environment.sh /tmp/ai_ship_robot_install_environment.sh
RUN bash /tmp/ai_ship_robot_install_environment.sh --system-only \
    && rm -f /tmp/ai_ship_robot_install_environment.sh \
    && rm -rf /var/lib/apt/lists/*

RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd --uid ${USER_UID} --gid ${USER_GID} --shell /bin/bash -m ${USERNAME}; fi \
    && usermod --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "if [ -f /workspaces/ai_ship_robot/scripts/setup_workspace.sh ]; then source /workspaces/ai_ship_robot/scripts/setup_workspace.sh; else source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /etc/bash.bashrc

USER ${USERNAME}
WORKDIR /workspaces/ai_ship_robot
