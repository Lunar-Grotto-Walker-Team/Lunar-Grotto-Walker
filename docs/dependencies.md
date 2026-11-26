# 依赖说明

本仓库只放团队自写的内容与**改过的上游包**；官方库统一收编在
[`Lunar-Grotto-Walker-Vendor`](https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-Vendor)，
通过根目录 [`deps.repos`](../deps.repos) 固定版本拉取。

> **为什么不直接从上游克隆再打补丁**：vendor 仓库里我们对上游的改动是 **commit**，
> 不是补丁文件。从上游克隆拿到的是未适配的代码，只能照着各目录的 `UPSTREAM.md`
> 手工改——既容易漏（比如 `AllowMulticast=spdp` 漏了会让大话题吞吐掉到 12 MB/s），
> 也无法保证团队每个人拿到的是同一套组合。上游跟踪是 vendor 仓库的职责，
> 主仓库只需要固定它的版本。

---

## 1. 本仓库内的包（源自上游，已修改）

| 包 | 上游 | License | 改动说明 |
|---|---|---|---|
| `src/point_lio_ros2`（包名 `point_lio`） | [dfloreaa/Point-LIO-ROS2](https://github.com/dfloreaa/Point-LIO-ROS2) → [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) | **GPL-2.0** | [UPSTREAM.md](../src/point_lio_ros2/UPSTREAM.md) |
| `src/simple_loop_closure_ros2`（包名 `simple_loop_closure`） | [ashBabu/simple_loop_closure](https://github.com/ashBabu/simple_loop_closure) → [vectr-ucla/SimpleLoopClosure](https://github.com/vectr-ucla/SimpleLoopClosure) | BSD-3-Clause | [UPSTREAM.md](../src/simple_loop_closure_ros2/UPSTREAM.md) |

> ⚠️ `point_lio` 是 **GPL-2.0**。公开分发无碍（保留 LICENSE 即可），但项目若需闭源或
> 商业化，这部分会受限，请提前评估。

## 2. 第三方包（来自 vendor 仓库）

| 包 | 上游 | 版本 | 本仓库用途 |
|---|---|---|---|
| `vendor/src/livox_ros_driver2` | [Livox-SDK](https://github.com/Livox-SDK/livox_ros_driver2) | master @ 2144554（1.2.8） | Mid-360S 点云驱动 |
| `vendor/src/Livox-SDK2` | [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK2) | v1.5.2 | 驱动底层 SDK |
| `vendor/src/unitree_ros2` | [unitreerobotics](https://github.com/unitreerobotics/unitree_ros2) | v0.3.0 | `unitree_go` 消息 + CycloneDDS 配置 |
| `vendor/src/BehaviorTree.ROS2` | [BehaviorTree](https://github.com/BehaviorTree/BehaviorTree.ROS2) | 0.3.0 | 行为树 ROS2 节点（含 deadline 修复） |
| `vendor/src/realsense-ros` | [IntelRealSense](https://github.com/IntelRealSense/realsense-ros) | 4.58.3 | 相机驱动（感知链路） |
| `vendor/src/librealsense` | [IntelRealSense](https://github.com/IntelRealSense/librealsense) | v2.58.3 | 相机底层 SDK |

## 3. 系统级依赖（apt）

| 依赖 | 本机版本 | 说明 |
|---|---|---|
| ROS 2 Humble | — | 容器 `self_l4t`（Ubuntu 22.04） |
| `ros-humble-gtsam` | 4.2.0 | 位姿图后端（simple_loop_closure 必需） |
| `ros-humble-pcl-ros` / `libpcl-dev` | 2.4.5 / 1.12.1 | 点云处理 |
| `libeigen3-dev` | 3.4.0 | 线性代数 |
| `libboost-all-dev` | 1.74 | `date_time`（Livox driver）、`filesystem`（SLC） |
| `ros-humble-tf2-eigen`、`ros-humble-message-filters` | — | SLC 依赖 |

安装：

```bash
sudo apt install ros-humble-gtsam ros-humble-pcl-ros ros-humble-tf2-eigen \
                 ros-humble-message-filters libpcl-dev libeigen3-dev libboost-all-dev
```

## 4. 手工安装到 `/usr/local` 的 SDK

这两个不是 ROS 包，必须 `make install` 到系统目录，ROS 驱动才能链接到。

### 4.1 Livox SDK2

```bash
cd src/vendor/src/Livox-SDK2
mkdir -p build && cd build && cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig
```

安装 5 个文件：`liblivox_lidar_sdk_{shared,static}` + 3 个头文件。
核对：`grep LIVOX_LIDAR_SDK_MINOR_VERSION /usr/local/include/livox_lidar_def.h`（应为 5，即 1.5.x）。

### 4.2 librealsense（Jetson 专属参数）

容器内 GCC 版本与 CUDA 11.4 冲突，必须指定 `gcc-10`；架构是 Jetson Orin（87）：

```bash
sudo apt install gcc-10 g++-10
export PATH=/usr/local/cuda/bin:$PATH
cd src/vendor/src/librealsense
mkdir -p build && cd build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DFORCE_RSUSB_BACKEND=true \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++-10 \
  -DCMAKE_CUDA_ARCHITECTURES=87 \
  -DBUILD_WITH_CUDA=true \
  -DBUILD_EXAMPLES=true \
  -DBUILD_GRAPHICAL_EXAMPLES=true \
  -DBUILD_PYTHON_BINDINGS=true
make -j$(nproc) && sudo make install && sudo ldconfig
```

## 5. 完整安装流程

```bash
# 1) 拉取第三方包（vendor 仓库，版本已在 deps.repos 固定）
vcs import src < deps.repos

# 2) 系统依赖
sudo apt install ros-humble-gtsam ros-humble-pcl-ros ros-humble-tf2-eigen \
                 ros-humble-message-filters libpcl-dev libeigen3-dev libboost-all-dev

# 3) 两个 /usr/local SDK（见第 4 节）

# 4) livox_ros_driver2 的 ROS2 换名（构建步骤，不是源码改动）
cd src/vendor/src/livox_ros_driver2
rm -f package.xml && cp package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/
cd -

# 5) 构建
colcon build --symlink-install --parallel-workers 4
```

> **不要用 livox 官方的 `./build.sh humble`**：它会删掉整个工作区的 `build/ install/`、
> 不带 `--packages-select` 编所有包、并在收尾删掉 `launch/`。
> 本仓库 `src/` 下还有 `librealsense` 等需要特殊 cmake 参数的包，会被拖进无参数编译。
> 详见 vendor 仓库 `livox_ros_driver2/UPSTREAM.md`。

## 6. 环境变量

节点启动前必须 source 叠加好的环境（RMW=cyclonedds、`CYCLONEDDS_URI` 指向 `eth10`、
ROS_DOMAIN_ID、各工作区 install）。导航链用 `.l4t_nav_env.sh`：

```bash
source /my_ws/l4t/.l4t_nav_env.sh
```

`CYCLONEDDS_URI` 里的 `AllowMulticast=spdp` 与 `MaxAutoParticipantIndex=30` 由
vendor 仓库 `unitree_ros2/setup.sh` 提供，**不要改回上游默认值**。

## 7. 更新依赖

```bash
# vendor 仓库跟上游更新（在其自己的 clone 里做，走 PR 到 develop）
cd Lunar-Grotto-Walker-Vendor
git subtree pull --prefix=src/<pkg> up-<name> <branch> --squash

# 主仓库升级 vendor 版本：改 deps.repos 里的 version，然后
vcs pull src
```
