FROM osrf/ros:humble-desktop

LABEL maintainer="Your-Name <you@example.com>"
LABEL description="ROS 2 Humble Python‑only Dev Container (Ubuntu 22.04)"

ENV LANG=ja_JP.UTF-8 \
    LANGUAGE=ja_JP:ja \
    LC_ALL=ja_JP.UTF-8

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    locales tzdata && \
    locale-gen ja_JP.UTF-8 && \
    ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ===== 開発ツール＋ROS 依存 =====
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential git \
    python3-ament-package \
    python3-colcon-common-extensions \
    python3-pip python3-venv python3-dev \
    python3-vcstool \
    python3-rosdep \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime         ros-humble-gazebo-ros-pkgs         curl wget nano less &&     (rosdep init || true) && rosdep update && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir -U pip && \
    pip install --no-cache-dir black ruff isort mypy pytest

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd  --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

ENV DEV_WS=/workspaces/ros2_humble_ws
RUN mkdir -p ${DEV_WS}
WORKDIR ${DEV_WS}

COPY --chown=${USERNAME}:${USERNAME} ./entrypoint.sh /usr/local/share/ros_entrypoint.sh
RUN chmod +x /usr/local/share/ros_entrypoint.sh
ENTRYPOINT ["/usr/local/share/ros_entrypoint.sh"]
CMD ["bash"]
