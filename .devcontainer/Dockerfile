# syntax=docker/dockerfile:1.7

ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG APT_PRIMARY_MIRROR=
ARG APT_SECURITY_MIRROR=
ARG APT_FORCE_IPV4=true
ARG APT_RETRIES=3
ARG APT_HTTP_TIMEOUT=20

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Tokyo \
    LANG=C.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# 遅いIPv6経路と不安定な通信でbuildが止まりにくいよう、aptの通信設定を先に固定する。
RUN printf '%s\n' \
        "Acquire::Retries \"${APT_RETRIES}\";" \
        "Acquire::http::Timeout \"${APT_HTTP_TIMEOUT}\";" \
        "Acquire::https::Timeout \"${APT_HTTP_TIMEOUT}\";" \
        > /etc/apt/apt.conf.d/99ai-ship-robot-network \
    && if [ "${APT_FORCE_IPV4}" = "true" ]; then echo 'Acquire::ForceIPv4 "true";' >> /etc/apt/apt.conf.d/99ai-ship-robot-network; fi

# 社内回線の任意mirrorを使いつつ、未設定や不正値ではUbuntu既定mirrorへ安全に戻す。
RUN primary_mirror="${APT_PRIMARY_MIRROR%/}" \
    && security_mirror="${APT_SECURITY_MIRROR%/}" \
    && if [[ ! "${primary_mirror}" =~ ^https?:// ]]; then primary_mirror="http://ftp.riken.jp/Linux/ubuntu"; fi \
    && if [[ ! "${security_mirror}" =~ ^https?:// ]]; then security_mirror="http://ftp.riken.jp/Linux/ubuntu"; fi \
    && sed -i "s#http://archive.ubuntu.com/ubuntu#${primary_mirror}#g" /etc/apt/sources.list \
    && sed -i "s#http://security.ubuntu.com/ubuntu#${security_mirror}#g" /etc/apt/sources.list

# BuildKitのapt cache mountを使い、VS Code接続とSource Controlに必要な最小packageだけを取得する。
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        python3 \
        sudo

# # プロキシ環境の独自CA証明書を信頼済みにする。不要な環境ではこのブロック全体をコメントアウトする。
# RUN mkdir -p /usr/local/share/ca-certificates/cato
# COPY [ "./cate_root_ca.crt", "/usr/local/share/ca-certificates/cato/ca.crt" ]
# RUN update-ca-certificates

RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd --uid ${USER_UID} --gid ${USER_GID} --shell /bin/bash -m ${USERNAME}; fi \
    && usermod --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Dev Container imageにsystem依存を焼き込む。問題切り分け時は、この3行のブロックをコメントアウトして手動実行へ戻す。
RUN mkdir -p /workspaces/ai_ship_robot/scripts
COPY scripts/install_environment.sh /workspaces/ai_ship_robot/scripts/install_environment.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    HOME=/home/${USERNAME} \
    AI_SHIP_ROBOT_BASHRC=/home/${USERNAME}/.bashrc \
    AI_SHIP_ROBOT_ENV_SCRIPT=/home/${USERNAME}/.ai_ship_robot_environment.bash \
    bash /workspaces/ai_ship_robot/scripts/install_environment.sh --system-only \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc /home/${USERNAME}/.ai_ship_robot_environment.bash

USER ${USERNAME}
WORKDIR /workspaces/ai_ship_robot
