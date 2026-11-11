#!/usr/bin/env bash
# 编译三个包：实验 5 的模型/相机包、实验 7 的 Gazebo 插件、以及本实验。
#   bash tools/build.sh
set -eo pipefail

WORKSPACE=${1:-$(cd "$(dirname "$0")/.." && pwd)}
source /opt/ros/humble/setup.bash

cd "$WORKSPACE"

if [ ! -d src ]; then
  echo "找不到 $WORKSPACE/src，请先在工作区根目录执行本脚本" >&2
  exit 1
fi

# 教学包里每个文件的时间戳都固定成 1980-01-01，这样同一份源码打出来的包逐字节
# 相同。代价是：解压到已经有 build/ 的工作区后，源码比旧产物还"旧"，make 判定
# 不用重编、cmake 判定 install 已是最新，install/ 里就留着上一版的文件。典型症状
#   executable 'state_node.py' not found on the libexec directory ...
# 所以编译前先把源码时间统一拉到现在，再让 cmake 忽略时间戳直接重装。
find src -type f -exec touch {} +

colcon build --symlink-install \
  --packages-select experiment5_depth experiment7_lidar experiment9_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_ALWAYS=1

source "$WORKSPACE/install/setup.bash"
ros2 pkg prefix experiment5_depth
ros2 pkg prefix experiment7_lidar
ros2 pkg prefix experiment9_fusion

# 安装完整性自检：缺一个可执行文件，launch 时才会报 not found，不如在这里拦住。
missing=0
for exe in \
  "install/experiment7_lidar/lib/experiment7_lidar/state_node.py" \
  "install/experiment7_lidar/lib/experiment7_lidar/preprocess_node.py" \
  "install/experiment9_fusion/lib/experiment9_fusion/fusion_node.py" \
  "install/experiment9_fusion/lib/experiment9_fusion/check_ready.py"; do
  if [ ! -e "$WORKSPACE/$exe" ]; then
    echo "安装不完整，缺少: $WORKSPACE/$exe" >&2
    missing=1
  fi
done
if [ "$missing" -ne 0 ]; then
  echo '请清掉旧构建后重试: rm -rf build install log && bash tools/build.sh' >&2
  exit 1
fi
echo '可执行文件已全部安装。'

echo
echo '编译完成。算法自检（不启动仿真）：'
python3 "$WORKSPACE/src/experiment9_fusion/tests/test_algorithms.py" 2>&1 | tail -3 || true

echo
echo '启动（三选一）:'
echo '  ros2 launch experiment9_fusion experiment9.launch.py mode:=solution   # 只看融合'
echo '  ros2 launch experiment9_fusion nav_demo.launch.py mode:=solution      # 完整导航'
echo '  ros2 run experiment9_fusion check_ready.py                            # 自检'
