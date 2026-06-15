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
RUN --mount=type=cache,id=ai-ship-robot-apt-cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=ai-ship-robot-apt-lists,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        git \
        pcl-tools \
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

# install script と固定third_party overrideだけを image build に含め、workspace build は container 起動後へ分離する。
RUN mkdir -p /workspaces/ai_ship_robot/install \
    /workspaces/ai_ship_robot/install/third_party_overrides \
    /workspaces/ai_ship_robot/sim/install
COPY install/ /workspaces/ai_ship_robot/install/
COPY install/third_party_overrides/ /workspaces/ai_ship_robot/install/third_party_overrides/
COPY sim/install/ /workspaces/ai_ship_robot/sim/install/

# 本番向けの system 依存だけを独立layerにし、workspace build は mount 後の setup へ回す。
RUN --mount=type=cache,id=ai-ship-robot-apt-cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=ai-ship-robot-apt-lists,target=/var/lib/apt/lists,sharing=locked \
    bash /workspaces/ai_ship_robot/install/install.sh

# 実機とsimulationで共通するthird_partyは、workspace mountに隠れない /opt 配下のunderlayとして導入する。
RUN --mount=type=cache,id=ai-ship-robot-apt-cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=ai-ship-robot-apt-lists,target=/var/lib/apt/lists,sharing=locked \
    bash /workspaces/ai_ship_robot/install/install_third_party.sh

# シミュレーション追加分も system 依存だけを別layerに分け、Gazebo系依存の更新を再利用しやすくする。
RUN --mount=type=cache,id=ai-ship-robot-apt-cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=ai-ship-robot-apt-lists,target=/var/lib/apt/lists,sharing=locked \
    bash /workspaces/ai_ship_robot/sim/install/install.sh

# Gazebo専用third_partyだけを共通underlayへ追加し、simulation workspace本体のbuildとは分離する。
RUN --mount=type=cache,id=ai-ship-robot-apt-cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=ai-ship-robot-apt-lists,target=/var/lib/apt/lists,sharing=locked \
    bash /workspaces/ai_ship_robot/sim/install/install_third_party.sh \
    && mkdir -p /home/${USERNAME}/.ros/log \
    && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}/.ros

USER ${USERNAME}
WORKDIR /workspaces/ai_ship_robot
