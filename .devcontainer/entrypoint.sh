#!/usr/bin/env bash
set -e

# ROS 2 Humble のセットアップ
source "/opt/ros/humble/setup.bash"

# ビルド済みワークスペースがあれば読み込み
if [ -f "$DEV_WS/install/setup.bash" ]; then
  source "$DEV_WS/install/setup.bash"
fi

exec "$@"