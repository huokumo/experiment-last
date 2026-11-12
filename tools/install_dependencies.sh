#!/usr/bin/env bash
# 装依赖。在 Ubuntu 22.04 + ROS 2 Humble 教学镜像上以 root 运行。
#   bash tools/install_dependencies.sh
set -eo pipefail

# 教学镜像常常只有内网源，或根本连不上外网。apt-get update 失败不代表依赖不可用
# （镜像里通常已经装齐），所以只提示、不中断：真缺包时下面的 install 会明确报出来，
# 学生看到的是一条中文提示，而不是一个没有上下文的 apt 错误码。
if ! apt-get update; then
  echo '提示: apt-get update 失败（可能是离线/内网镜像），继续用本地索引尝试安装。' >&2
fi

if ! apt-get install -y --no-install-recommends \
  git unzip wget python3-numpy python3-scipy python3-yaml \
  ros-humble-ros-gz libignition-gazebo6-dev ros-humble-ament-cmake-python \
  ros-humble-ament-cmake-pytest \
  ros-humble-robot-state-publisher ros-humble-tf2-ros-py ros-humble-tf2-tools \
  ros-humble-rviz2 ros-humble-rosbag2 ros-humble-launch-xml \
  ros-humble-message-filters ros-humble-ros2run ros-humble-ros2param ros-humble-ros2node \
  ros-humble-navigation2 ros-humble-nav2-bringup; then
  echo
  echo '依赖没有全部装上。请联网后重跑本脚本；若已是离线环境，' >&2
  echo '把上面的报错发出来，按缺失的包名单独安装。' >&2
  exit 1
fi

echo
echo '依赖安装完成。下一步: bash tools/prepare_workspace.sh && bash tools/build.sh'
echo '注意: 实验 5/实验 7 的 Gazebo 插件与场景由 prepare_workspace.sh 拉取，不要单独编译同名包。'
