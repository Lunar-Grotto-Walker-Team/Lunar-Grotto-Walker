#!/usr/bin/env python3
"""仓库结构与配置校验（CI 用，不编译）。

为什么不做编译：本仓库的 ROS2 包依赖 Jetson 侧才有的 `unitree_go` 与
`livox_ros_driver2`，GitHub 托管 runner 上装不出来。真编译在 Jetson 上跑，
CI 只守住"提交进来的东西结构完整、配置能解析"这条底线。

检查项：
  1. 每个 src/<pkg>/package.xml 是合法 XML 且带 <name>
  2. ament_cmake 包必须有 CMakeLists.txt，ament_python 包必须有 setup.py
  3. 所有 .yaml / .rviz 能被 YAML 解析（rviz 配置也是 YAML）
  4. 没有把 build/ install/ log/ 提交进来
  5. 没有单个文件超过 5 MB
  6. 没有残留 .gitmodules（nanoflann 是普通目录，不是 submodule）

用法：python3 tools/ci/validate_repo.py [--root .]
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

MAX_FILE_BYTES = 5 * 1024 * 1024
SKIP_DIRS = {".git", "build", "install", "log", "__pycache__", "thirdparty"}


def iter_files(root: Path, patterns: tuple[str, ...]):
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        if any(part in SKIP_DIRS for part in path.relative_to(root).parts):
            continue
        if path.suffix in patterns:
            yield path


def check_packages(root: Path, errors: list[str]) -> int:
    src = root / "src"
    if not src.is_dir():
        errors.append("缺少 src/ 目录")
        return 0

    count = 0
    for pkg_xml in sorted(src.glob("*/package.xml")):
        pkg_dir = pkg_xml.parent
        count += 1
        try:
            tree = ET.parse(pkg_xml)
        except ET.ParseError as exc:
            errors.append(f"{pkg_xml}: XML 解析失败: {exc}")
            continue

        name = tree.getroot().findtext("name")
        if not name:
            errors.append(f"{pkg_xml}: 缺少 <name>")
            continue

        buildtools = {e.text for e in tree.getroot().findall("buildtool_depend")}
        if "ament_cmake" in buildtools and not (pkg_dir / "CMakeLists.txt").is_file():
            errors.append(f"{pkg_dir}: ament_cmake 包缺少 CMakeLists.txt")
        if "ament_python" in buildtools and not (pkg_dir / "setup.py").is_file():
            errors.append(f"{pkg_dir}: ament_python 包缺少 setup.py")

        print(f"  [pkg] {name:28s} <- {pkg_dir.relative_to(root)}")
    return count


def check_yaml(root: Path, errors: list[str]) -> int:
    count = 0
    for path in iter_files(root, (".yaml", ".yml", ".rviz")):
        count += 1
        try:
            with path.open(encoding="utf-8") as handle:
                yaml.safe_load(handle)
        except yaml.YAMLError as exc:
            errors.append(f"{path.relative_to(root)}: YAML 解析失败: {exc}")
        except UnicodeDecodeError as exc:
            errors.append(f"{path.relative_to(root)}: 不是 UTF-8 文本: {exc}")
    return count


def check_hygiene(root: Path, errors: list[str]) -> None:
    for bad in ("build", "install", "log"):
        if (root / bad).is_dir():
            errors.append(f"仓库里不应有 {bad}/（构建产物）")

    if (root / ".gitmodules").exists():
        errors.append(".gitmodules 应删除：thirdparty/nanoflann 以普通目录入库")

    for path in iter_files(root, (".pcd", ".db3", ".bag", ".mcap", ".zip", ".tar", ".gz")):
        errors.append(f"{path.relative_to(root)}: 不应提交大数据文件")

    for path in root.rglob("*"):
        if not path.is_file() or any(p in SKIP_DIRS for p in path.relative_to(root).parts):
            continue
        size = path.stat().st_size
        if size > MAX_FILE_BYTES:
            errors.append(
                f"{path.relative_to(root)}: {size / 1024 / 1024:.1f} MB 超过 5 MB 上限")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".", help="仓库根目录")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    errors: list[str] = []

    print(f"[validate] root = {root}")
    packages = check_packages(root, errors)
    yaml_files = check_yaml(root, errors)
    check_hygiene(root, errors)

    print(f"[validate] packages={packages} yaml={yaml_files} errors={len(errors)}")
    if errors:
        print("\n[validate] 失败项：", file=sys.stderr)
        for item in errors:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[validate] OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
