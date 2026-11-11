#!/usr/bin/env bash
# 装依赖。在 Ubuntu 22.04 + ROS 2 Humble 教学镜像上以 root 运行。
#   bash tools/install_dependencies.sh
set -eo pipefail

apt-get update
apt-get install -y --no-install-recommends \
  git unzip wget python3-numpy python3-scipy python3-yaml \
  ros-humble-ros-gz libignition-gazebo6-dev ros-humble-ament-cmake-python \
  ros-humble-ament-cmake-pytest \
  ros-humble-robot-state-publisher ros-humble-tf2-ros-py ros-humble-tf2-tools \
  ros-humble-rviz2 ros-humble-rosbag2 ros-humble-launch-xml \
  ros-humble-message-filters ros-humble-ros2run ros-humble-ros2param ros-humble-ros2node \
  ros-humble-navigation2 ros-humble-nav2-bringup

echo
echo '依赖安装完成。下一步: bash tools/prepare_workspace.sh && bash tools/build.sh'
echo '注意: 实验 5/实验 7 的 Gazebo 插件与场景由 prepare_workspace.sh 拉取，不要单独编译同名包。'
