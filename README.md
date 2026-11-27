# Lunar-Grotto-Walker
月窟行者

## 技术栈

- **嵌入式**: C/C++
- **AI/仿真**: Python
- **CI/CD**: GitHub Actions
- **项目管理**: GitHub Projects (V2)

## 开发流程

请参考 [开发流程指南](docs/project/Git-Flow-Guide.md)

## 模块

| 模块 | 代码 | 文档 |
|---|---|---|
| 导航与建图 | `src/terrain/`、Point-LIO（独立仓库） | `docs/slam/` |
| 行为树与机械臂 | `src/lgw_bt_planner`、`src/lgw_3d_tracker`、`src/go2_transit`、`src/agx_topic_hardware` 等 | [docs/bt/](docs/bt/README.md) |
| 启动与验证脚本 | `tools/lgw/` | [tools/lgw/README.md](tools/lgw/README.md) |

## 依赖与构建

> ⚠️ **本仓库必须与 vendor 仓库配套使用，单独 clone 下来是编译不过的。**
> 第三方驱动与基础库在
> [Lunar-Grotto-Walker-Vendor](https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-Vendor)
> 里**单独部署**，不是本仓库的依赖项：先 clone 并构建它，`source` 它的
> `install/setup.bash`，再编译本仓库。vendor 升级不需要改本仓库任何文件。

GPL-2.0 的 Point-LIO 前端在
[Lunar-Grotto-Walker-PointLIO](https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-PointLIO)，
以避免与仓库内的 BSD-3 代码混在一起；它由根目录 [`deps.repos`](deps.repos)
钉 commit 拉取，是 `deps.repos` 里唯一的条目。

```bash
# 前提：vendor 已 clone 并构建好
source ../Lunar-Grotto-Walker-Vendor/install/setup.bash
vcs import src < deps.repos        # 拉取 Point-LIO
python3 tools/preflight_deps.py    # 检查 vendor 环境是否就绪
colcon build --symlink-install
```

完整清单、系统依赖、`/usr/local` SDK 安装步骤见
[依赖说明](docs/dependencies.md)。
