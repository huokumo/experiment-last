#!/usr/bin/env bash
# 准备好实验 5（R1 模型 + 深度相机）与实验 7（Gazebo 插件 + Livox 扫描图案），
# 两种来源都支持，版本都固定，避免"上周还能跑、这周跑不起来"。
#
#   教学包（离线）：src/experiment5、src/experiment7 已随包提供，
#                   本脚本只校验文件哈希与提交号，不联网；
#   代码仓库（联网）：把两个仓库克隆到 src/ 下并切到固定提交。
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

# 教学包分支：依赖源码随包提供，只做完整性校验，不访问网络。
if [ -f "$WORKSPACE/BUNDLED_DEPENDENCIES.json" ]; then
  python3 "$WORKSPACE/tools/verify_bundled_dependencies.py" "$WORKSPACE"
  # 权限位在这一步也兜一层：资源站转存、Windows 解压都会把 755 抹成 644，而
  # --symlink-install 下 install/ 里的可执行权限完全跟随源码。build.sh 里也有
  # 同样的处理，两处都留，是为了"只跑了这一个脚本"的人也不会踩到。
  find "$WORKSPACE/src" -path '*/scripts/*' -type f \( -name '*.py' -o -name '*.sh' \) \
    -exec chmod +x {} + 2>/dev/null || true
  exit 0
fi

fetch() {
  local name=$1 commit=$2
  local target="$WORKSPACE/src/$name"
  if [ -d "$target" ] && [ ! -d "$target/.git" ]; then
    echo "$name 已存在但不是 git 仓库，也不是教学包 —— 请先删除 $target 再重试"
    exit 2
  fi
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
