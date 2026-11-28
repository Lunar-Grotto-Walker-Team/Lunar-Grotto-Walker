# 依赖说明

> **本版本验证环境**：vendor `develop` @ `7e77bc5`（2026-11-27，含 agx_arm_ros）
> —— 只在发布 / 合并到 `main` 时更新这一行，不随 vendor 每次提交变。

本仓库只放团队自写的内容与**改过的上游包**；官方库统一收编在
[`Lunar-Grotto-Walker-Vendor`](https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-Vendor)。

**vendor 不是本仓库的依赖项，而是配套的独立环境**：单独 clone、单独构建，
用它的 `install/setup.bash` 叠加到环境里之后再编译本仓库。根目录
[`deps.repos`](../deps.repos) 只负责 GPL-2.0 的 Point-LIO（见 1.3）。

> **为什么用 vendor 而不是直接从上游克隆再打补丁**：vendor 仓库里我们对上游的
> 改动是 **commit**，不是补丁文件。从上游克隆拿到的是未适配的代码，只能照着各目录
> 的 `UPSTREAM.md` 手工改——既容易漏（比如 `AllowMulticast=spdp` 漏了会让大话题
> 吞吐掉到 12 MB/s），也无法保证团队每个人拿到的是同一套组合。上游跟踪是 vendor
> 仓库的职责，主仓库只记录验证时用的 vendor 版本（本页顶部那一行）。
>
> **为什么 vendor 不写进 `deps.repos`**：写进去等于每次部署都把 vendor 克隆到
> `src/vendor/` 编第二遍，两个工作区里同名包各有一份，source 顺序一反就跑旧版本；
> 而且 vendor 每次发版都要改主仓库的 pin，主仓库会被无关变更牵着走。
> 依赖清单只留真正属于本仓库的包，环境靠 source 叠加。

---

## 1. 上游包（已修改）

### 1.1 在本仓库内（宽松许可）

| 包 | 上游 | License | 改动说明 |
|---|---|---|---|
| `src/simple_loop_closure_ros2`（包名 `simple_loop_closure`） | [ashBabu/simple_loop_closure](https://github.com/ashBabu/simple_loop_closure) → [vectr-ucla/SimpleLoopClosure](https://github.com/vectr-ucla/SimpleLoopClosure) | BSD-3-Clause | [UPSTREAM.md](../src/simple_loop_closure_ros2/UPSTREAM.md) |
| `src/frontier_exploration_ros2` | [mertgulerx/frontier-exploration-ros2](https://github.com/mertgulerx/frontier-exploration-ros2) @ `v1.6.1` | Apache-2.0 | [UPSTREAM.md](../src/frontier_exploration_ros2/UPSTREAM.md)（+5804/-43，35 个文件） |

两者都是宽松许可，与主仓库兼容，不需要像 `point_lio` 那样隔离。

### 1.2 团队自写（非上游）

| 目录 | 内容 |
|---|---|
| `src/base/` | `go2_core`、`go2_driver`、`go2_twist_bridge`（底盘驱动、`map_odom_correction`、TF 桥） |
| `src/nav/` | `go2_navigation2`（Nav2 配置/行为树）、`go2_perception`、`go2_slam`、`go2_description` |
| `src/go2_transit/`、`src/pointlio_cloud_relay/` | 摆渡 action、点云 relay |
| `src/go2_front_camera_driver/` | Go2 前置相机（GStreamer H.264） |

> `go2_description` 的 `dae/` + `meshes/` 约 50 MB，是 Unitree 官方模型，
> **不入库**：`bash tools/fetch_go2_meshes.sh` 从 `unitreerobotics/unitree_ros`
> 拉取（钉 commit，与官方逐字节一致）。缺模型时包仍可构建，只是 RViz 里没有模型。

### 1.3 在独立仓库（GPL-2.0）

| 包 | 仓库 | 上游 | License |
|---|---|---|---|
| `src/point_lio_ros2`（包名 `point_lio`） | [Lunar-Grotto-Walker-PointLIO](https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-PointLIO) | [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) → [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) | **GPL-2.0** |

> **为什么单独一个仓库**：`point_lio` 是 GPL-2.0，本仓库其余代码是 BSD-3-Clause 与
> 团队自写代码。同一仓库分发 GPL 与宽松许可代码，容易在法律上被当成一个整体作品，
> 从而让 BSD-3 部分被 GPL 覆盖（《第三方仓库修改与入库通用指南》§6/§10）。
> 拆开后 GPL 义务只由那个仓库承担；将来若需闭源交付，只需替换这一个依赖。
>
> 本仓库不复制它的代码，只通过 `deps.repos` 钉一个 commit 拉取，
> 落地路径仍是 `src/point_lio_ros2`，各 launch 里的包名 `point_lio` 不变。
> 改动清单见该仓库的 `UPSTREAM.md`。

## 2. 第三方包（来自配套的 vendor 工作区）

vendor 仓库内部也分两级：**改过的入库**（带 `UPSTREAM.md`），**未改动的**由 vendor 的
`deps.repos` 拉取。主仓库不关心它的内部分级，只用它 `install/` 里编译好的包。

**本仓库在构建时实际用到的 vendor 包**：

| vendor 包 | 谁需要 | 提供什么 |
|---|---|---|
| `unitree_ros2` → `unitree_go` / `unitree_api` | `simple_loop_closure`、`utlidar_terrain_fill`、`body_pose_bridge`、`lgw_bt_planner` | 宇树消息定义 |
| `livox_ros_driver2` | `point_lio` | Mid-360S 的 `CustomMsg` |
| `BehaviorTree.ROS2` → `behaviortree_ros2` / `btcpp_ros2_interfaces` | `lgw_bt_planner`、`go2_transit` | BT.ROS2 节点与执行动作（含 deadline 修复） |
| `agx_arm_ros` → `agx_arm_msgs` / `agx_arm_moveit` | `lgw_bt_planner` | Piper 消息、MoveIt 配置与 demo launch |

其余 vendor 包（`elevation_mapping`、`kindr`、`kindr_ros`）服务于 `nav_ws` 下的实机链路，
主仓库编译时不需要，但同一个 vendor 工作区一并提供。

### 2.1 vendor 入库的包（已修改）

| 包 | 上游 | 版本 | 本仓库用途 |
|---|---|---|---|
| `vendor/src/livox_ros_driver2` | [Livox-SDK](https://github.com/Livox-SDK/livox_ros_driver2) | master @ 2144554（1.2.8） | Mid-360S 点云驱动 |
| `vendor/src/unitree_ros2` | [unitreerobotics](https://github.com/unitreerobotics/unitree_ros2) | v0.3.0 | `unitree_go` 消息 + CycloneDDS 配置 |
| `vendor/src/BehaviorTree.ROS2` | [BehaviorTree](https://github.com/BehaviorTree/BehaviorTree.ROS2) | 0.3.0 | 行为树 ROS2 节点（含 deadline 修复） |
| `vendor/src/elevation_mapping` | [Muhammad540](https://github.com/Muhammad540/elevation_mapping) | main @ 710fffc | 高程图（激光径向距离门 + Go2 配置） |
| `vendor/src/kindr` | [ANYbotics](https://github.com/ANYbotics/kindr) | master @ 2446444 | elevation_mapping 依赖（CMake 版本适配） |
| `vendor/src/kindr_ros` | [SivertHavso](https://github.com/SivertHavso/kindr_ros) | galactic @ af5667c | elevation_mapping 依赖（rosidl 依赖补齐） |
| `vendor/src/agx_arm_ros` | [agilexrobotics](https://github.com/agilexrobotics/agx_arm_ros) | ros2 @ b9ad14d | Piper 驱动 + MoveIt（park 位姿 / topic 后端 / TF 前缀） |

### 2.2 vendor 导入的包（未修改）

| 包 | 上游 | 版本 | 本仓库用途 |
|---|---|---|---|
| `vendor/src/Livox-SDK2` | [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK2) | v1.5.2 | 驱动底层 SDK |
| `vendor/src/realsense-ros` | [IntelRealSense](https://github.com/IntelRealSense/realsense-ros) | 4.58.3 | 相机驱动（感知链路） |
| `vendor/src/librealsense` | [IntelRealSense](https://github.com/IntelRealSense/librealsense) | v2.58.3 | 相机底层 SDK |

`grid_map` 不在 vendor 里 —— 直接用 apt 的 `ros-humble-grid-map-*`（2.0.1）。

## 3. 系统级依赖（apt）

| 依赖 | 本机版本 | 说明 |
|---|---|---|
| ROS 2 Humble | — | 容器 `self_l4t`（Ubuntu 22.04） |
| `ros-humble-gtsam` | 4.2.0 | 位姿图后端（simple_loop_closure 必需） |
| `ros-humble-pcl-ros` / `libpcl-dev` | 2.4.5 / 1.12.1 | 点云处理 |
| `libeigen3-dev` | 3.4.0 | 线性代数 |
| `libboost-all-dev` | 1.74 | `date_time`（Livox driver）、`filesystem`（SLC） |
| `ros-humble-tf2-eigen`、`ros-humble-message-filters` | — | SLC 依赖 |
| `ros-humble-moveit*` | 2.5.x | `lgw_bt_planner`、`lgw_servo_smoothing`、`servo_state_probe` |
| `ros-humble-navigation2`（含 `nav2_msgs`） | 1.1.x | `go2_transit` 的 FollowPath / Spin 与消息 |
| `ros-humble-ros2-control`、`ros-humble-ros2-controllers` | — | `agx_topic_hardware` 硬件插件 |
| `ros-humble-behaviortree-cpp` | **4.9.1** | `lgw_bt_planner`、`go2_transit`（用 v4，不是 v3） |
| `nlohmann-json3-dev` | 3.x | `lgw_bt_planner` |
| `can-utils` | — | `can_activate.sh` 检查并配置 can0（`can_activate.sh` 会拒绝在缺 `can-utils` 时运行） |
| `python3-numpy`、`python3-scipy` | — | `lgw_3d_tracker` |
| `ros-humble-rosbag2`（含 `ros2 bag`） | — | `tools/lgw/record/record_cameras.sh` 等录制脚本 |

安装：

```bash
sudo apt install ros-humble-gtsam ros-humble-pcl-ros ros-humble-tf2-eigen \
                 ros-humble-message-filters ros-humble-moveit ros-humble-navigation2 \
                 ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-behaviortree-cpp ros-humble-rosbag2 nlohmann-json3-dev \
                 can-utils python3-numpy python3-scipy \
                 libpcl-dev libeigen3-dev libboost-all-dev
```

### 3.1 Python 依赖（pip）

| 依赖 | 来源 / 版本 | 用途 |
|---|---|---|
| `pyAgxArm` | [agilexrobotics/pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)，本机 `1.0.0` | `agx_arm_ctrl` 的 CAN SDK（vendor 的 `agx_arm_ros` 运行时依赖） |

```bash
# 方式一：源码安装（可指定 commit/tag）
git clone https://github.com/agilexrobotics/pyAgxArm.git
pip install -e pyAgxArm

# 方式二：直接从仓库安装
pip install "git+https://github.com/agilexrobotics/pyAgxArm.git"
```

> 版本用 `python3 -c "import pyAgxArm; print(pyAgxArm.__version__)"` 核对，
> 与 vendor 的 `agx_arm_ros` 保持同一代（本机验证组合为 `pyAgxArm 1.0.0` +
> vendor `agx_arm_ros@b9ad14d`）。

### 3.2 配套外部仓库（不在本仓库、也不在 vendor）

| 仓库 | 用途 | 缺失后果 |
|---|---|---|
| `Lunar-Grotto-Walker-Team/lgw-yoloe-perception`（AGPL-3.0，独立仓库） | 宿主机 Foxy 的 YOLOE 两阶段检测 + Humble 侧双相机编排，发布 `/perception/two_stage/*/result` | `IsHeadDetectionConfirmed` / `IsArmDetectionConfirmed` / `ComputeGraspPosesFromTrack` 等节点失败 |
| 导航工作区（Nav2 bringup、costmap、地图） | `go2_transit` 复用 Nav2 的 FollowPath / Spin | `TransitToLocalPoint` 等不到 `/go2_transit/transit_to_pose` 动作服务器 |

> 感知链路是 **AGPL-3.0**，按许可边界独立成仓库，不并入本仓库（同 Point-LIO 的处理）。
> 缺失时本仓库仍可构建，只是依赖感知的 BT 子树会失败。

### 3.3 宿主工作区 / 容器镜像提供的依赖

| 依赖 | 谁用 | 说明 |
|---|---|---|
| `unitree_sdk2`（C++，2.0.0） | `lgw_vui_bridge` 的 `go2_vui_set_brightness_helper` | **不随本仓库 / vendor 分发**。构建前宿主机上需有一份**已编译**的 SDK 源码树（产出 `lib/<arch>/libunitree_sdk2.a` 与 `thirdparty/lib/<arch>/libddsc*.so`）；用 `UNITREE_SDK2_DIR` 或 `-DUNITREE_SDK2_SOURCE_DIR=` 指过去，默认回退 `/my_ws/foxy/unitree_sdk2` |
| `domain_bridge`（`ros-humble-domain-bridge`） | `tools/lgw/start/start_lgw_perception_bridge.sh`（感知 `--lo` 模式） | 由容器镜像提供（其他成员维护镜像构建），本仓库不在 apt 清单里重复 |
| `realsense2_camera`、`yoloe_perception`、`yoloe_humble_orchestration` | 感知链路脚本 | 由 vendor / 独立感知仓库提供，见 §2 与 §3.2 |

> `unitree_sdk2` 只被 VUI helper 链接（静态 `libunitree_sdk2.a` + SDK 自带的
> `libddsc/libddscxx`）。它与 rclcpp 的 `rmw_cyclonedds_cpp` 在同一进程里会发生
> ddscxx 冲突，所以 helper 以独立子进程运行——这是设计约束，不是可选依赖。

## 4. 手工安装到 `/usr/local` 的 SDK

以下三个不是普通 ROS 包（或需要特殊 CMake 参数），必须在 **vendor 仓库里**
`make install` 到系统目录，后面的 ROS 包才能链接到。

### 4.1 Livox SDK2

```bash
cd src/Livox-SDK2
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
cd src/librealsense
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

### 4.3 kindr（elevation_mapping 依赖）

```bash
cd src/kindr
mkdir -p build && cd build
cmake .. -DUSE_CMAKE=true
sudo make install && sudo ldconfig
touch ../../kindr/COLCON_IGNORE   # 不让 colcon 重复编译
```

## 5. 部署（两个独立仓库，先环境后应用）

**主仓库必须配一份 vendor**，且 vendor 要先构建好：

```
Lunar-Grotto-Walker-Vendor/   ← 环境（第三方驱动与基础库）
        install/setup.bash
                ↓  source
Lunar-Grotto-Walker/          ← 应用（团队代码 + Point-LIO）
        install/setup.bash
```

### 第一阶段：vendor 仓库（环境）

```bash
git clone https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-Vendor.git
cd Lunar-Grotto-Walker-Vendor

vcs import src < deps.repos      # 拉 Livox-SDK2 / librealsense / realsense-ros

# ① SDK 层（第 4 节，逐个 make install）
# ② livox_ros_driver2 的 ROS2 换名（构建步骤，不是源码改动）
cd src/livox_ros_driver2
rm -f package.xml && cp package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/
cd ../..

colcon build --symlink-install --parallel-workers 4
source install/setup.bash        # 之后所有编译本仓库的终端都要先 source 这一句
```

### 第二阶段：主仓库（应用）

```bash
git clone https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker.git
cd Lunar-Grotto-Walker

source ../Lunar-Grotto-Walker-Vendor/install/setup.bash   # ← 缺这句编译会失败
vcs import src < deps.repos                              # 只拉 point_lio_ros2

sudo apt install ros-humble-gtsam ros-humble-pcl-ros ros-humble-tf2-eigen \
                 ros-humble-message-filters ros-humble-grid-map-* \
                 ros-humble-moveit ros-humble-navigation2 ros-humble-ros2-control \
                 ros-humble-ros2-controllers ros-humble-behaviortree-cpp \
                 ros-humble-rosbag2 nlohmann-json3-dev \
                 can-utils python3-numpy python3-scipy \
                 libpcl-dev libeigen3-dev libboost-all-dev

python3 tools/preflight_deps.py                          # 检查环境是否就绪
colcon build --symlink-install --parallel-workers 4
source install/setup.bash
```

**运行时的环境**：vendor → 主仓库，顺序不能反。CycloneDDS 配置
（`eth10` + `AllowMulticast=spdp`）由 vendor 的 `src/unitree_ros2/setup.sh` 提供。

> **不要把 vendor 拉进本仓库的 `src/`**。那样 vendor 会被编第二遍，两个工作区
> 里同名包各有一份，source 顺序一反就跑到旧版本；而且 `deps.repos` 里也没写它。
> 本仓库单独 clone 下来是**不完整**的，必须先有 vendor——`preflight_deps.py`
> 会先报这一条。

> **不要用 livox 官方的 `./build.sh humble`**：它会删掉整个工作区的 `build/ install/`、
> 不带 `--packages-select` 编所有包、并在收尾删掉 `launch/`。
> vendor 的 `src/` 下还有 `librealsense`、`realsense-ros` 等需要特殊 cmake 参数的包，
> 会被拖进无参数编译。详见 vendor 仓库 `livox_ros_driver2/UPSTREAM.md`。

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
# ① vendor 跟上游更新（在 vendor 自己的 clone 里做，走 PR 到它的 develop）
cd Lunar-Grotto-Walker-Vendor
git subtree pull --prefix=src/<pkg> up-<name> <branch> --squash

# ② 主仓库里的 Point-LIO 升级
vcs pull src

# ③ vendor 升级后主仓库这边只需要一件事：
#    重新验证一遍，然后更新本页顶部「本版本验证环境」那一行。
#    deps.repos 不用动 —— vendor 不在里面。
```
