#!/usr/bin/env bash
# fetch_go2_meshes.sh — 拉取 go2_description 的官方模型文件
#
# dae/ + meshes/ 合计约 50 MB（每个 7 个文件），是 Unitree 官方 Go2 模型，
# 与上游 unitreerobotics/unitree_ros 的 robots/go2_description 逐字节一致，
# 因此不入库，部署时按需下载。
#
# 用法：
#   bash tools/fetch_go2_meshes.sh          # 下载到 src/nav/go2_description/
#   bash tools/fetch_go2_meshes.sh --check  # 只检查当前文件是否与上游一致
#
# 拉完需要重新 colcon build --packages-select go2_description，
# 让 install/ 里的 share 目录带上模型（脚本会提醒）。

set -uo pipefail

PIN=ed3803887b2690d445b534084df0c50d7507d76a   # unitree_ros master @ 2026-11-28
UPSTREAM=https://github.com/unitreerobotics/unitree_ros
SUBDIR=robots/go2_description

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$REPO_ROOT/src/nav/go2_description"

die() { echo "ERROR: $*" >&2; exit 1; }

[ -f "$DEST/package.xml" ] || die "找不到 $DEST —— 请在主仓库根目录运行"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "[fetch] 克隆 $UPSTREAM @ $PIN（只取 $SUBDIR，blob 按需下载）"
git clone --quiet --filter=blob:none --no-checkout "$UPSTREAM" "$TMP/src" \
  || die "克隆失败（检查网络/代理）"
git -C "$TMP/src" sparse-checkout init --cone
git -C "$TMP/src" sparse-checkout set "$SUBDIR"
git -C "$TMP/src" checkout --quiet "$PIN" || die "checkout $PIN 失败（上游可能重写了历史）"

if [ "${1:-}" = "--check" ]; then
  rc=0
  for d in dae meshes; do
    [ -d "$DEST/$d" ] || { echo "[check] 缺少 $d/"; rc=1; continue; }
    if diff -rq "$TMP/src/$SUBDIR/$d" "$DEST/$d" >/dev/null; then
      echo "[check] $d 与上游一致"
    else
      echo "[check] $d 与上游不一致"; rc=1
    fi
  done
  exit $rc
fi

for d in dae meshes; do
  mkdir -p "$DEST/$d"
  cp -a "$TMP/src/$SUBDIR/$d/." "$DEST/$d/"
  echo "[fetch] $d: $(ls -1 "$DEST/$d" | wc -l) 个文件"
done

cat <<EOF

完成。接下来：
  colcon build --packages-select go2_description
  重新 source install/setup.bash

验证：
  ls install/go2_description/share/go2_description/dae
EOF
