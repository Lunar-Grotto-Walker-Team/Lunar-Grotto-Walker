#!/usr/bin/env python3
"""构建前置检查：vendor 环境是否就绪、Point-LIO 是否已拉取。

为什么需要它
------------
vendor 仓库（第三方驱动与基础库）**不是本仓库的依赖项**，而是独立部署的环境。
忘了 source 它的 `install/setup.bash`，`colcon build` 只会抛一串
「找不到 unitree_go」，很容易被误判成本仓库的代码问题。

这个脚本在编译前跑一遍，把「缺环境」和「代码有问题」区分开：

  1. `point_lio_ros2/` 是否已由 `vcs import src < deps.repos` 拉到 src/ 下；
  2. vendor 提供的包（`unitree_go`、`unitree_api`、`livox_ros_driver2`、
     `behaviortree_ros2`、`agx_arm_msgs`、`agx_arm_moveit`）能否在 AMENT_PREFIX_PATH 里找到。

只用标准库，不依赖 ROS 运行时，任何终端都能跑。

用法：python3 tools/preflight_deps.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

# 本仓库在构建时真正需要的 vendor 包。别加需求：加一条就要同步
# docs/dependencies.md 第 2 节那张表。
VENDOR_PACKAGES = {
    "unitree_go": ("simple_loop_closure", "utlidar_terrain_fill"),
    "livox_ros_driver2": ("point_lio",),
    "unitree_api": ("body_pose_bridge", "lgw_bt_planner"),
    "behaviortree_ros2": ("lgw_bt_planner", "go2_transit"),
    "agx_arm_msgs": ("lgw_bt_planner",),
    "agx_arm_moveit": ("lgw_bt_planner",),
}

# 由 vcs import 拉取、不在仓库里的本仓库包
IMPORTED_PACKAGES = {"point_lio_ros2": "point_lio"}

VENDOR_HINT = """\
先构建并 source vendor 工作区：
    git clone https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-Vendor.git
    cd Lunar-Grotto-Walker-Vendor && vcs import src < deps.repos
    # 按 src/README.md 把 Livox-SDK2 / librealsense / kindr 先 make install
    colcon build --symlink-install --parallel-workers 4
    source install/setup.bash          # ← 每个编译本仓库的终端都要这一句
详见 docs/dependencies.md 第 5 节。"""


def ament_prefixes() -> list[Path]:
    raw = os.environ.get("AMENT_PREFIX_PATH", "")
    return [Path(p) for p in raw.split(os.pathsep) if p]


def find_package(pkg: str, prefixes: list[Path]) -> Path | None:
    """在 AMENT_PREFIX_PATH 里定位一个 ament 包，返回它所在的 prefix。"""
    for prefix in prefixes:
        if (prefix / "share" / pkg / "package.xml").is_file():
            return prefix
    return None


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    errors: list[str] = []
    notes: list[str] = []
    env_problem = False

    # ---- 1. 本仓库里由 vcs import 拉取的包 ----
    src = root / "src"
    for name, pkg in IMPORTED_PACKAGES.items():
        pkg_dir = src / name
        if not (pkg_dir / "package.xml").is_file():
            errors.append(
                f"缺少 src/{name}/（包 {pkg}）—— 在仓库根执行：vcs import src < deps.repos"
            )
        else:
            notes.append(f"src/{name}/ 就绪（包 {pkg}）")

    # ---- 2. vendor 提供的包 ----
    prefixes = ament_prefixes()
    if not prefixes:
        env_problem = True
        errors.append("AMENT_PREFIX_PATH 为空 —— 没有 source 任何 ROS 环境")
    for pkg, consumers in VENDOR_PACKAGES.items():
        prefix = find_package(pkg, prefixes)
        if prefix is None:
            env_problem = True
            errors.append(f"找不到 {pkg}（{', '.join(consumers)} 需要）")
        else:
            notes.append(f"{pkg:18s} <- {prefix}")

    # ---- 输出 ----
    for line in notes:
        print(f"  [ok] {line}", flush=True)

    if errors:
        print("\n[preflight] 失败：", file=sys.stderr)
        for item in errors:
            print(f"  - {item}", file=sys.stderr)
        if env_problem:
            print("\n[preflight] " + VENDOR_HINT, file=sys.stderr)
        return 1

    print("[preflight] OK —— vendor 环境与 Point-LIO 都已就绪，可以 colcon build")
    return 0


if __name__ == "__main__":
    sys.exit(main())
