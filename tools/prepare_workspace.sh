#!/usr/bin/env bash
# 把实验 5（R1 模型 + 深度相机）和实验 7（Gazebo 插件 + Livox 扫描图案）
# 拉到工作区 src/ 下，版本固定，避免"上周还能跑"这类问题。
#
#   bash tools/prepare_workspace.sh              # 默认工作区 = 本仓库根目录
#   bash tools/prepare_workspace.sh /root/exp9
set -eo pipefail

WORKSPACE=${1:-$(cd "$(dirname "$0")/.." && pwd)}
case "$WORKSPACE" in /*) ;; *) echo '工作区必须是绝对路径'; exit 2 ;; esac

EXP5_COMMIT=807de9e2454d0c6bd7ab3067e6743acf429e19f9
EXP7_COMMIT=9ed58e7330504337377b973c1da227c8bd2a9715

if [ ! -d "$WORKSPACE/src/experiment9_fusion" ]; then
  echo "找不到 $WORKSPACE/src/experiment9_fusion —— 请在整个仓库根目录运行本脚本"
  exit 2
fi

fetch() {
  local name=$1 commit=$2
  local target="$WORKSPACE/src/$name"
  if [ ! -d "$target/.git" ]; then
    echo "拉取 $name ..."
    git clone --filter=blob:none "https://github.com/huokumo/$name.git" "$target"
  fi
  git -C "$target" fetch --quiet origin "$commit"
  git -C "$target" checkout --quiet "$commit"
  local actual
  actual=$(git -C "$target" rev-parse HEAD)
  if [ "$actual" != "$commit" ]; then
    echo "$name 版本不符: 期望 $commit 实际 $actual"
    exit 2
  fi
  echo "$name @ ${commit:0:8}"
}

fetch experiment5 "$EXP5_COMMIT"
fetch experiment7 "$EXP7_COMMIT"

chmod +x "$WORKSPACE/src/experiment9_fusion/scripts/"*.py \
         "$WORKSPACE/src/experiment9_fusion/scripts/"*.sh \
         "$WORKSPACE/src/experiment7/scripts/"*.py \
         "$WORKSPACE/src/experiment5/scripts/"*.py 2>/dev/null || true

echo
echo '工作区就绪:'
echo "  $WORKSPACE/src/experiment5         (R1 模型与深度相机)"
echo "  $WORKSPACE/src/experiment7         (Gazebo 插件与 Mid-360 扫描图案)"
echo "  $WORKSPACE/src/experiment9_fusion  (本实验)"
echo '下一步: bash tools/build.sh'
