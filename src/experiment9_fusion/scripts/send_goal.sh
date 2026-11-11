#!/usr/bin/env bash
# 用命令行发目标点，不用 RViz 的 2D Goal Pose 按钮。
#
#   ./send_goal.sh                       # 默认目标 (4.6, 3.0)，朝向 0°
#   ./send_goal.sh 4.6 3.0 0             # x y yaw(度)
#   ./send_goal.sh 4.6 3.0 0 odom        # 指定坐标系（一般不要改）
#
# 为什么不用鼠标点：手点每次落点都不同，坐标无法复现；这条命令本身就是
# 一份可存档、可重放的证据，报告里直接贴终端输出即可。
set -euo pipefail

X=${1:-4.6}
Y=${2:-2.6}
YAW_DEG=${3:-0}
FRAME=${4:-map}

QUAT=$(python3 - "$YAW_DEG" <<'PY'
import math, sys
yaw = math.radians(float(sys.argv[1]))
print(f'{0.0:.6f} {0.0:.6f} {math.sin(yaw / 2):.6f} {math.cos(yaw / 2):.6f}')
PY
)
read -r QX QY QZ QW <<<"$QUAT"

echo "目标: frame=$FRAME  x=$X  y=$Y  yaw=${YAW_DEG}deg"
echo "四元数: x=$QX y=$QY z=$QZ w=$QW"
echo

# 注意两个常见坑：
#   1. orientation 全填 0 是零四元数，行为树会直接拒绝，表现为"发了没反应"；
#   2. 不加 --feedback 就只能等最后结果，看不到卡在哪一段。
exec ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: $FRAME}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}}}" \
  --feedback
