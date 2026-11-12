#!/usr/bin/env bash
# 编译三个包：实验 5 的模型/相机包、实验 7 的 Gazebo 插件、以及本实验。
#   bash tools/build.sh
#
# 脚本自己会处理"工作区里残留了上一版构建"的情况，不需要手工删 build/。
set -eo pipefail

WORKSPACE=${1:-$(cd "$(dirname "$0")/.." && pwd)}
source /opt/ros/humble/setup.bash

cd "$WORKSPACE"

if [ ! -d src ]; then
  echo "找不到 $WORKSPACE/src，请先在工作区根目录执行本脚本" >&2
  exit 1
fi

# 编译前把源码时间统一拉到现在。教学包解压覆盖到已经有 build/ 的工作区时，
# 源文件的 mtime 可能比旧产物还旧，make 会判定不用重编、cmake 会判定 install
# 已是最新，install/ 里就留着上一版的内容。
find src -type f -exec touch {} +

# 脚本一律补上可执行位。--symlink-install 装进 install/ 的是符号链接，权限跟着
# 源码走：源码是 644，install/ 里那个链接就不可执行，ros2 launch 会误报
#   executable 'state_node.py' not found on the libexec directory ...
# 解压工具、U 盘、复制粘贴都可能把权限磨掉，这里统一兜一层。
find src -path '*/scripts/*' -type f \( -name '*.py' -o -name '*.sh' \) -exec chmod +x {} +

compile() {
  colcon build --symlink-install \
    --packages-select experiment5_depth experiment7_lidar experiment9_fusion \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_ALWAYS=1
}

# 少一个可执行文件，launch 时才会报 not found，不如在这里拦住。
# 用 -x 而不是 -e：--symlink-install 下名字在、但链接指向的源码没有执行位，
# ros2 launch 同样报 not found，这种情形只有 -x 能发现。
install_complete() {
  local missing=0
  for exe in \
    install/experiment7_lidar/lib/experiment7_lidar/state_node.py \
    install/experiment7_lidar/lib/experiment7_lidar/preprocess_node.py \
    install/experiment9_fusion/lib/experiment9_fusion/fusion_node.py \
    install/experiment9_fusion/lib/experiment9_fusion/check_ready.py; do
    if [ ! -x "$exe" ]; then
      echo "  安装缺失或不可执行: $WORKSPACE/$exe" >&2
      missing=1
    fi
  done
  return "$missing"
}

compile
source "$WORKSPACE/install/setup.bash"

if ! install_complete; then
  echo '安装目录与源码不一致，清掉 build/install/log 后重新编译…'
  rm -rf build install log
  find src -type f -exec touch {} +
  find src -path '*/scripts/*' -type f \( -name '*.py' -o -name '*.sh' \) -exec chmod +x {} +
  compile
  source "$WORKSPACE/install/setup.bash"
  if ! install_complete; then
    echo '重建后仍缺可执行文件，请把上面的编译输出发出来。' >&2
    exit 1
  fi
fi

ros2 pkg prefix experiment5_depth
ros2 pkg prefix experiment7_lidar
ros2 pkg prefix experiment9_fusion
echo '可执行文件已全部安装。'

echo
echo '编译完成。算法自检（不启动仿真）：'
python3 "$WORKSPACE/src/experiment9_fusion/tests/test_algorithms.py" 2>&1 | tail -3 || true

echo
echo '启动（三选一）:'
echo '  ros2 launch experiment9_fusion experiment9.launch.py mode:=solution   # 只看融合'
echo '  ros2 launch experiment9_fusion nav_demo.launch.py mode:=solution      # 完整导航'
echo '  ros2 run experiment9_fusion check_ready.py                            # 自检'
