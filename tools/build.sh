#!/usr/bin/env bash
# 编译三个包：实验 5 的模型/相机包、实验 7 的 Gazebo 插件、以及本实验。
#   bash tools/build.sh
set -eo pipefail

WORKSPACE=${1:-$(cd "$(dirname "$0")/.." && pwd)}
source /opt/ros/humble/setup.bash

cd "$WORKSPACE"
colcon build --symlink-install \
  --packages-select experiment5_depth experiment7_lidar experiment9_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source "$WORKSPACE/install/setup.bash"
ros2 pkg prefix experiment5_depth
ros2 pkg prefix experiment7_lidar
ros2 pkg prefix experiment9_fusion

echo
echo '编译完成。算法自检（不启动仿真）：'
python3 "$WORKSPACE/src/experiment9_fusion/tests/test_algorithms.py" 2>&1 | tail -3 || true

echo
echo '启动（三选一）:'
echo '  ros2 launch experiment9_fusion experiment9.launch.py mode:=solution   # 只看融合'
echo '  ros2 launch experiment9_fusion nav_demo.launch.py mode:=solution      # 完整导航'
echo '  ros2 run experiment9_fusion check_ready.py                            # 自检'
