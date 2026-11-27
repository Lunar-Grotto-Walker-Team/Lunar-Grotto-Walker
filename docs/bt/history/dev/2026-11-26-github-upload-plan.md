# lg_walker_ws GitHub 上传方案

> 日期：2026-11-26
> 范围：`/home/unitree/my_ws/l4t/lg_walker_ws`（行为树主工程）及其必需的外部依赖。
> 状态：**方案待确认，未执行任何 `git init` / push**。
> 相关文档：`docs/dev/2026-11-06-vendor-modifications-and-rollback.md`（agx_arm_ros 的 V1–V8 改动）。

---

## 0. 结论摘要

0. **目标仓库已存在**：`Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker`（组织仓库，public，main/develop
   已有历史与 CI/Git Flow 规范）。方案由"新建仓库"改为"往既有 monorepo 提 PR"，详见 §2.5。
   本地 `l4t/Lunar-Grotto-Walker` 的 remote 目前指向个人仓库 `cbzZZZZ/...`，需先修正。
1. **主仓库 = 自写代码**：`lg_walker_ws` 的 8 个包 + `go2_transit` / `pointlio_cloud_relay`（目前在
   `nav_ws/go2-nav2-terrain-pointlio/src/` 下，漏了它 BT 跑不起来）。
2. **改过的上游仓库必须自持**（fork 到自己的账号）：`agx_arm_ros`、`BehaviorTree.ROS2`。
   这两处有实质性修改，不能简单指向官方仓库。
3. **没改过的上游仓库不进仓库**：用 `deps.repos`（vcstool）固定 URL + commit 引用，
   配 `scripts/bootstrap_deps.sh` 一键拉齐。理由：体积、可维护性、贡献归属清晰。
4. **当前目录里 64 GB 的 `bags/` 绝对不能提交**；`.superpowers/`、`.run/`、`build/ install/ log/`
   同样排除。
5. **本工作区曾经是 git 仓库**，`.git` 已丢失且无本地备份；3 个 worktree 目录
   （`lg_walker_ws_feature_detect_and_move`、`lg_walker_ws_yu`、`baseline_wt`）现在是孤儿，
   上传前需先处理。详见 §8.1。

---

## 1. 现状盘点（2026-11-26 实测）

### 1.1 工作区构成

| 路径 | 体积 | 处理 |
|---|---|---|
| `bags/` | **64 GB** | 排除（`/home/unitree/my_ws/l4t/nav_ws/bags` 另有同名目录，同样排除） |
| `build/` `install/` `log/` | 673 M / 176 M / 13 M | 排除 |
| `.run/` | 11 M | 排除（其中 8 个 probe 脚本先搬到 `scripts/diag/`） |
| `.superpowers/` | 2.0 M | 排除（SDD 草稿；有价值的内容迁到 `docs/history/`） |
| `src/` | 2.9 M | **进仓库** |
| `scripts/` | 588 K | 进仓库（需归类） |
| `docs/` | 1.2 M | 进仓库（需分层） |
| `config/` | 16 K | 进仓库（DDS lo 配置 + perception_bridge YAML） |
| `test/` | 112 K | 进仓库（YOLOE 端到端脚本） |
| `.lg_retained_C_group/` | 36 K | 排除（已 ignore，历史备份） |
| `frames_2026-10-19_15.35.07.gv`、`work.log` | 8 K | 排除 |

### 1.2 自有包（`src/` 下 8 个）

| 包 | 角色 | 产出 | 关键依赖 |
|---|---|---|---|
| `lgw_bt_planner`（2.3 M，核心） | BT 编排 + 几何/伺服 | `bt_runner`、`servo_node`、`select_arm_observation_pose_service_node`、`apply_head_camera_collision_node`、35 个 BT 节点插件、`msg/ServoState`、`srv/{SelectArmObservationPose,Takeover}`、22 棵 BT XML、`calibration/*.json` | behaviortree_ros2、moveit_msgs、agx_arm_msgs、unitree_go/api、yoloe_interfaces |
| `lgw_3d_tracker`（308 K） | 头/臂双相机统一 3D 跟踪 | `unified_track_node`（Python） | yoloe_interfaces、tf2、numpy/scipy |
| `lgw_vui_bridge`（64 K） | Go2 VUI 亮度（SDK 进程隔离） | `vui_bridge_node` + `go2_vui_set_brightness_helper` | Unitree SDK2 |
| `body_pose_bridge`（64 K） | Go2 姿态服务 | `body_pose_bridge_node` → `/set_body_pose` | unitree_api / unitree_go |
| `agx_topic_hardware`（44 K） | ros2_control 硬件插件 | `agx_topic_hardware/AgxTopicSystem` | hardware_interface、pluginlib |
| `lgw_servo_smoothing`（24 K） | MoveIt Servo 平滑插件 | 插件库 | moveit_core |
| `servo_state_probe`（36 K） | 独立诊断探针 | `servo_state_probe` | moveit_ros_planning |
| `yoloe_interfaces`（52 K） | 跨 distro 感知接口 | msg/srv | — |

**遗漏项（必须补进仓库）**：

| 包 | 当前位置 | 说明 |
|---|---|---|
| `go2_transit` | `l4t/nav_ws/go2-nav2-terrain-pointlio/src/go2_transit` | 自写，Apache-2.0。**注册 `TransitToLocalPoint`**，`track_collect_reactive_v2*.xml`、`transit_mission.xml` 依赖它 |
| `pointlio_cloud_relay` | 同上 | 自写，Apache-2.0。导航侧点云转发；若仓库只覆盖 BT 链，可留给导航仓库，但需在 README 声明 |

### 1.3 BT 节点清单（35 个，`src/bt_plugin_registration.cpp`）

| 类别 | 节点 |
|---|---|
| 机械臂 / MoveIt（4） | `MoveArmToPose` `MoveArmToNamed` `SetPoseStamped` `ControlGripper` |
| 感知与几何（9） | `IsHeadDetectionConfirmed` `IsArmDetectionConfirmed` `ComputeLookatPose` `VerifyHeadArmConsistency` `ComputeGraspPose` `ComputeHeadTargetWorld` `ComputeArmTargetWorld` `MakePoseFromPointAndReference` `SelectArmObservationPose` |
| 跟踪 / Servo（13） | `IsTargetAvailable` `IsTargetWithin` `ComputeGraspPosesFromTrack` `ComputeGraspPosesSampled` `WaitForTargetSelected` `WaitUntilTargetNear` `WaitForArmHits` `IsArmCameraAlive` `ServoTakeover` `ServoRelease` `MarkTrack` `RecordBasePose` `IsWithinBaseRadius` |
| Go2 运动 / VUI（8） | `StandUp` `BalanceStand` `StandDown` `SetBodyHeight` `SetBodyPose` `SwitchGait` `SetLedBrightness` `StartMapping` |
| 人机交互（1） | `WaitForUserInput` |
| 外部注册 | `TransitToLocalPoint` ← `go2_transit` |

BT XML 共 22 棵（`bt_trees/`），其中带导航的为 `track_collect_reactive_v2*.xml`、`transit_*.xml`、`go_to_target_and_back.xml`。

### 1.4 运行时依赖的外部仓库（实测是否改过）

| 仓库 / 包 | 本地位置 | 是否改过 | 上传策略 |
|---|---|---|---|
| `agx_arm_ros`（`agx_arm_ctrl` / `agx_arm_moveit` / `agx_arm_msgs`） | `l4t/agx_arm_ws/src/agx_arm_ros` | **是**：V1–V8（park home、`hardware_backend:=topic`、JTC 容差）+ `rsp.launch.py` 的 `frame_prefix` | **fork 自持**，见 §2.2 |
| `BehaviorTree.ROS2` v0.3.0（`behaviortree_ros2` / `btcpp_ros2_interfaces`） | `l4t/bt_ros2_ws/src/BehaviorTree.ROS2` | **是**：`tree_execution_server.cpp` 的 deadline 累加修复（树假死）、`bt_action_node.hpp` 补 `return onFailure(GOAL_REJECTED_BY_SERVER)` | **fork 自持** 或 `patches/` 补丁 |
| `unitree_ros2` | `l4t/unitree_ros2` | 否（仅 `setup.sh` 的 eth10 配置 + vendored nlohmann 版本差） | `deps.repos` 引用上游 + 本仓库保留 `config/` 里的 DDS 配置 |
| `pyAgxArm` | `l4t/pyAgxArm` | 否（核心源码逐字一致） | `pip install` / 引用上游 |
| `ros2_ws`（`realsense-ros` + `livox_ros_driver2`） | `l4t/ros2_ws/src` | 未核实（预计无改） | 引用上游或 apt，上传前用 `git clone --depth 1` 对比一次 |
| `nav_ws` 其余包（`go2_navigation2` / `go2_slam` / `go2_perception` / `go2_description` / `base`） | `l4t/nav_ws/go2-nav2-terrain-pointlio/src` | 上游 unitree-go2-slam-nav2 拷贝（package.xml 仍是 `TODO` 占位） | 引用上游 |
| `yoloe_perception`（宿主机 Foxy）+ `/opt/yoloe_transfer/humble_ws` | 宿主机 `$HOME/yoloe_deploy`、容器 `/opt` | 外部部署，非标准 ROS 包 | 文档说明 + 记录版本/权重 |
| `domain_bridge`、`moveit2`、`nav2`、`ros2_control`、`tf2` | apt（humble） | — | apt 清单写进 `docs/SETUP.md` |
| `qpmad` | `src/lgw_bt_planner/thirdparty/qpmad` | vendored 源码 | 保留，随附 `NOTICE` / `LICENSE` |

### 1.5 关键端点契约（BT ↔ 节点）

| 方向 | 端点 | 提供方 |
|---|---|---|
| 动作 | `/move_action` | `agx_arm_moveit`（MoveGroup） |
| 动作 | `/gripper_controller/joint_trajectory`、`/arm_controller/joint_trajectory` | `agx_arm_moveit`（ros2_control） |
| 动作 | `TransitToLocalPoint` | `go2_transit` |
| 服务 | `/set_body_pose`、`/set_vui_brightness`、`/start_mapping`、`/select_arm_observation_pose` | `body_pose_bridge`、`lgw_vui_bridge`、`lgw_bt_planner`、`lgw_bt_planner` |
| 服务 | `/lgw/servo/{takeover,release,pause,resume}`、`/lgw/servo/state` | `lgw_bt_planner`（servo_node） |
| 服务 | `/servo_node/{start,pause,unpause}_servo`、`/servo_node/delta_twist_cmds` | MoveIt Servo |
| 状态 | `/feedback/joint_states`、`/feedback/tcp_pose`、`/feedback/gripper_status` | `agx_arm_ctrl` |
| 指令 | `/control/joint_states` | `agx_topic_hardware`（topic 后端） |
| 感知 | `/perception/two_stage/camera_{head,arm}/result` | `yoloe_perception`（宿主机，经 `domain_bridge` 跨域） |
| 感知 | `/perception/tracks/unified`、`/lgw/tracker/mark_track` | `lgw_3d_tracker` |
| 机器人 | `/api/sport/request`、`/api/vui/request`、`/lf/sportmodestate` | `unitree_ros2` + Go2 固件 |
| 人工 | `/bt_user_input` | 操作者 |

---

## 2. 分类原则

### 2.1 三类处理

| 类别 | 判据 | 处理 |
|---|---|---|
| A. 自写 | 本仓库作者维护 | 进主仓库 `src/` |
| B. 改过的上游 | 有实质代码修改 | **在组织下建独立仓库**，主仓库用 submodule 挂 `src/<repo>` |
| C. 未改的上游 | 逐字一致 | 不进仓库，`deps.repos` 固定 commit 引用 |

### 2.2 B 类处理细则

- **`agx_arm_ros`**：按 `2026-11-06-vendor-modifications-and-rollback.md` 的 V1–V8 + `frame_prefix`，
  以上游 `b9ad14d`（2026-09-04）为基线重建仓库，改动做成一条提交，推到
  `Lunar-Grotto-Walker-Team/agx_arm_ros`（详见 §2.6）。之后 `upstream` remote 保留，便于 merge。
- **`BehaviorTree.ROS2`**：两处修复同上游现状不一致，建议：
  1. 优先向上游提 PR（deadline bug 是真 bug）；
  2. 短期在组织下建 `Lunar-Grotto-Walker-Team/BehaviorTree.ROS2`（`lgw-fixes` 分支），
     同样以 submodule 挂 `src/BehaviorTree.ROS2`；或放
     `patches/bt_ros2-deadline-fix.patch` 由 bootstrap 应用（仓库更干净，但会随上游 drift）。

### 2.3 为什么 C 类不复制进仓库

1. 体积与信噪比：`nav_ws` 是几十个 SLAM 仓库的试验场，`unitree_ros2` 带 cyclonedds，全拷会让仓库失控。
2. 维护双份：上游更新无法跟进；`agx_arm_ros` 现在"改过又无 git 历史"的困境不应重演。
3. 贡献归属：MIT/Apache 允许再分发，但混在一起会让 review 分不清哪些是本项目工作。
4. 可复现性不靠复制：`deps.repos` + bootstrap 脚本等价且干净。

### 2.4 感知链路（YOLOE）的处置

感知链路与前两类都不同：它是一套**独立部署在宿主机 Foxy** 的推理系统，同时依赖容器内
Humble 的相机编排，且**已触发第三方许可证约束**。

#### 现状（2026-11-26 实测）

| 资产 | 位置 | 体积 | 性质 |
|---|---|---|---|
| 核心推理代码 `yoloe_transfer/` | `~/yoloe_deploy/yoloe_transfer` | 164 K | 自写 Python（config/inference/geometry/schema/workflow/priority_decode/serial_io） |
| Foxy 节点 `yoloe_perception` | `~/yoloe_deploy/ros2_ws/src/yoloe_perception` | 892 K（含十余个 `.bak`） | 自写 ROS 包：`two_stage_node` `dual_camera_node` `serial_bridge_node` `success_frame_recorder` `trajectory_visualizer_node` |
| Humble 编排 `yoloe_humble_orchestration` | `~/yoloe_deploy/humble_ws/src/` | — | 自写，`two_realsense.launch.py`（两路 realsense） |
| 接口 `yoloe_interfaces` | 两处：`~/yoloe_deploy/ros2_ws/src/` 与主仓库 `src/` | 36 K / 52 K | **已漂移**（见下） |
| 权重 `models/` | `~/yoloe_deploy/models` | 106 M | 竞赛专用 `.pt` / `.onnx` / `.engine` + prompt 聚合 |
| 证据 `evidence/` | `~/yoloe_deploy/evidence` | 261 M | 评测图/视频/结果 |
| 备份 `backups/` `audit_envs/` `source_bundles/` `prompt_packages/` | 同目录 | 27 M / 20 M / 1.9 M / 11 M | 历史与素材 |
| 容器内副本 | `/opt/yoloe_transfer`（**镜像 overlay 层，非挂载**） | v5.0.0 | 手工 `docker cp` 进镜像，无构建来源 |

#### 两个必须先解决的问题

1. **许可证**：`requirements-nx.txt` 锁 `ultralytics==8.4.33`（**AGPL-3.0**），
   而 `yoloe_perception/package.xml` 自声明 `<license>Proprietary</license>`。
   因此**不能并入主仓库的 MIT/Apache 代码**（AGPL 传染），公开时也必须单独说明。
2. **接口漂移**：主仓库 `yoloe_interfaces` 是超集（多了 `UnifiedTrack` `UnifiedTrackArray`
   `srv/MarkTrack`），Foxy 部署侧只有 5 个旧 msg；共享 msg 目前仅差空行，**暂时兼容但无人在管**。

#### 处置方案（推荐 A）

**A. 拆成独立仓库 `lgw-yoloe-perception`（推荐，默认私有）**

- 进仓库（约 5 MB）：
  - `yoloe_transfer/`
  - `ros2_ws/src/yoloe_perception/`（删除所有 `.bak*`，保留 launch）
  - `humble_ws/src/yoloe_humble_orchestration/`
  - `config/*.yaml`（去 `.bak`）、`prompt_assets/`、`docs/`（docx 转 md）、`tests/`、
    `systemd/*.service.template`、`scripts/` 中保留 bench/eval 常用项
- 不进仓库：`models/*.{pt,onnx,engine}`、`evidence/`、`backups/`、`audit_envs/`、
  `prompt_packages/`、`source_bundles/`、全部 `.bak*`
- 权重策略：`models/README.md` + `export_models.sh`（从 ultralytics 导出）+ `SHA256SUMS`
- LICENSE：AGPL-3.0（受 ultralytics 约束），`NOTICE` 注明权重不随仓库分发
- 主仓库侧只保留：`yoloe_interfaces`（canonical）+ 接口契约文档 + 缺失时的降级说明

**B. 并入主仓库 `perception/` 子树** —— 不推荐：许可证混用、体积、Foxy/Humble 双环境
会把主仓库的 README 复杂度拉满。

**C. 完全不入库、只写文档** —— 最省事但他人无法复现；若仓库定位只是"展示 BT 逻辑"可选。

#### 配套动作（方案 A 下）

1. `yoloe_interfaces` 以主仓库为**唯一 canonical**；感知仓库删除自带副本，
   bootstrap 时从主仓库同步（submodule 或脚本复制），并在两侧 README 写兼容矩阵（v5.0.0）。
2. 容器 `/opt/yoloe_transfer` 目前无构建来源 → 增加 `docker/perception_install.sh`
   （`docker cp` humble_ws + `colcon build`）或 Dockerfile 片段，让镜像可重建。
3. `start_lgw_yoloe.sh` 的 `YOLOE_DEPLOY` 默认值改为必填/占位（`$HOME/yoloe_deploy` 只作示例）。
4. 主仓库 README 增加"感知链路（可选）"一节：说明它是独立私有仓库、接口契约是什么、
   缺失时哪些 BT 节点会失败（`IsHeadDetectionConfirmed` / `IsArmDetectionConfirmed` /
   `ComputeGraspPosesFromTrack` / `SelectArmObservationPose` 等）。

### 2.5 目标仓库现状与约束（2026-11-26 核查）

目标：`https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker`

| 项 | 实测 |
|---|---|
| 存在性 | 是（HTTP 200），**public**，默认分支 `main` |
| 组织下仓库数 | 1（即该仓库） |
| 现存内容 | `README.md`、`docs/project/`（Git Flow 指南、模板）、`src/README.md` 骨架，约 24 KB |
| 远端分支 | `main` = `3230632`，`develop` = `5deb355` |
| 本地克隆 | `l4t/Lunar-Grotto-Walker`（6.3 MB，工作区干净） |
| ⚠️ 本地 remote | 指向 **`github.com/cbzZZZZ/Lunar-Grotto-Walker`**（个人仓库，main=`dfbbb46`，**落后于组织仓库**） |
| 未推送分支 | `feature/1-pointlio-slam`（`b409a7f`，把 Point-LIO + simple_loop_closure 整栈放进 `src/`） |
| 分支流程 | Git Flow：`feature/<id>-<desc>` → `develop`（PR + 1 review）→ `main`（PR + 2 review），**禁止直接推 main** |
| CI | `c-cpp-ci`：仓库根 `cmake ..` + clang-tidy + gtest；`python-ci`：`black --check .` + `ruff check .` + pytest |
| 仓库约定 | `src/` **平铺 ROS2 功能包**（先例：`point_lio_ros2`、`simple_loop_closure_ros2`），文档放 `docs/<主题>/` |
| LICENSE | 无 |
| `.gitignore` | 已有 `build/ install/ log/`，缺 `bags/`、`*.bag`、`.run/`、`frames_*` |

**因此方案要改的 6 处：**

1. **不做 `git init`**：改为在既有仓库上从 `develop` 开 feature 分支提交，走 PR 流程。
2. **先修 remote**：本地 clone 指向个人仓库，必须改成组织仓库并 fetch 对齐；未推送的
   `feature/1-pointlio-slam` 要先 rebase 到最新 `develop` 再决定是否发 PR。
3. **目录改为 `src/` 平铺包**（不是嵌套工作区），文档按 `docs/<主题>/` 落位，与既有约定一致。
4. **CI 必须同步改造**，否则 PR 必红（见下）。
5. **vendor 策略与团队既有做法对齐**：团队已把 Point-LIO 整栈放进 `src/`。建议沿用该习惯，
   但只 vendor **用到且改过**的（`agx_arm_ros`、`BehaviorTree.ROS2`），未改的走 apt/文档。
6. **感知链路仍独立成仓库**（AGPL-3.0），放到同一组织下，主仓库只留接口与文档。

**CI 的具体风险（上传前必须处理）：**

- `c-cpp-ci` 在**仓库根**执行 `cmake ..`：本仓库根没有 `CMakeLists.txt`，且 8 个包依赖
  MoveIt / BT.ROS2 / unitree 消息，GitHub runner 上不存在 → 必失败。需要改为
  ROS 容器内 `colcon build`，或加 `paths` 过滤只对相关目录生效。
- `python-ci` 全局 `black --check .` / `ruff check .`：现有 Python 代码（`yoloe_*`、`scripts/`）
  风格不统一 → 必失败。需要先格式化或限定路径。

### 2.6 改过的官方驱动 `agx_arm_ros` 的落地方式

> **2026-11-27 修订**：团队已建立 Vendor 仓库机制（`docs/dependencies.md` + `deps.repos`），
> 实际落地改走 **vendor 路线**（见 §10 执行记录）。本节保留 fork+submodule 方案作为备选，
> 新依赖入库以 Vendor 仓库的 `docs/third-party-modes.md` 为准。

**原结论（已被 vendor 路线取代）：在组织下单独建仓库，monorepo 用 submodule 引用。**

理由：

1. `agx_arm_urdf` 子模块带 **205 MB meshes**。平铺要么把大二进制塞进 monorepo，要么丢 mesh
   让 rviz/MoveIt 碰撞几何退化。
2. 改过的 vendor 必须能跟上游 diff/merge；独立仓库保留完整历史和 `upstream` remote。
3. review 边界清晰：驱动的 V1–V8 补丁与 BT 业务代码分开评审。
4. monorepo 的 PR 只增加一个 gitlink + `.gitmodules`，符合 Git Flow 的小 PR 习惯。
5. CI 可按路径过滤，驱动相关 job 单独跑，不影响主 CI。

#### 步骤 A：本机重建驱动仓库

```bash
git clone https://github.com/agilexrobotics/agx_arm_ros.git ~/agx_arm_ros_org
cd ~/agx_arm_ros_org
git checkout -b main b9ad14d          # 2026-09-04 基线
git submodule update --init --recursive

# 覆盖 10 个本地改动文件（相对 b9ad14d 逐字核对过）
SRC=/home/unitree/my_ws/l4t/agx_arm_ws/src/agx_arm_ros
for f in \
  src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py \
  src/agx_arm_moveit/config/agx_arm.ros2_control.xacro \
  src/agx_arm_moveit/config/agx_arm.srdf \
  src/agx_arm_moveit/config/agx_arm.srdf.xacro \
  src/agx_arm_moveit/config/agx_arm.urdf.xacro \
  src/agx_arm_moveit/config/initial_positions.yaml \
  src/agx_arm_moveit/launch/_moveit_config_builder.py \
  src/agx_arm_moveit/launch/demo.launch.py \
  src/agx_arm_moveit/launch/rsp.launch.py \
  test/gripper/test_gripper_joint_states.yaml ; do cp "$SRC/$f" "$f"; done

git status --short                    # 必须恰好 10 个 M，无删除项
git commit -m "feat(go2): park home pose + topic hardware backend + TF frame prefix"
```

#### 步骤 B：推到组织仓库（两种方式，推荐 B1）

**B1. GitHub 网页 Fork（保留 "forked from agilexrobotics/agx_arm_ros" 关联）**

1. 打开 `https://github.com/agilexrobotics/agx_arm_ros` → **Fork** → *Create a new fork*；
2. **Owner** 选 `Lunar-Grotto-Walker-Team`，名称保持 `agx_arm_ros`；
3. `Copy the main branch only` 保持勾选（上游默认分支就是 `ros2`，够用）；
4. 本地：

```bash
git clone --recursive https://github.com/Lunar-Grotto-Walker-Team/agx_arm_ros.git ~/agx_arm_ros_org
cd ~/agx_arm_ros_org
git remote add upstream https://github.com/agilexrobotics/agx_arm_ros.git
git fetch upstream --tags
git checkout -b main b9ad14d
# ...（步骤 A 的 10 文件覆盖 + 提交）
git push -u origin main
git tag lgw-v1 && git push origin lgw-v1
```

5. 仓库 Settings → Branches 把默认分支设为 `main`（`ros2` 保留给上游同步）。
   - 上游默认分支是 `ros2`，fork 会继承；改成 `main` 只影响"直接 clone fork 时拿到什么"
     （不改则别人 clone 到的是**未打补丁**的上游代码）。
   - 不影响上游同步：`git fetch upstream && git merge upstream/ros2` 与默认分支无关，
     GitHub 的 Sync fork 也只更新 `ros2`。
   - 不影响给上游提 PR：base 仍是 `agilexrobotics/agx_arm_ros:ros2`。
   - **严禁把本地补丁推到 `ros2`**，该分支保持只读镜像。

> Fork 前提：`agx_arm_ros` 必须 public，且你在组织里有建仓库权限；
> 不满意可删除 fork 重来，不影响上游。

**B2. 手工等价（无 forked-from 关联，但 git 内容完全一样）**

```bash
git remote rename origin upstream
git remote add origin https://github.com/Lunar-Grotto-Walker-Team/agx_arm_ros.git
git push -u origin main
git tag lgw-v1 && git push origin lgw-v1
```

> 无论哪种方式，**本地改动要拆成按主题的多个提交**（park home / topic 后端 /
> frame prefix / 测试 yaml），而不是一个 blob —— 上游 merge 冲突时可以逐个
> rebase、cherry-pick 或丢弃。

#### 步骤 C：monorepo 接入（走 Git Flow）

```bash
cd /home/unitree/my_ws/l4t/Lunar-Grotto-Walker
git checkout -b feature/<issue-id>-agx-arm-driver origin/develop
git submodule add -b main \
  https://github.com/Lunar-Grotto-Walker-Team/agx_arm_ros.git src/agx_arm_ros
git commit -m "feat(driver): 引入 agx_arm_ros（park home + topic 后端补丁）"
```

- 驱动仓库自带的 `agx_arm_urdf` 子模块仍指向 `agilexrobotics/agx_arm_urdf`（public），
  克隆/CI 需要 **`--recursive`**；
- `.github/workflows/*` 的 `actions/checkout@v4` 必须加 `with: submodules: recursive`，
  否则驱动目录是空的、`colcon build` 静默少包。

#### 步骤 D：日常同步上游

```bash
cd src/agx_arm_ros
git fetch upstream && git merge upstream/ros2   # 在驱动仓库提 PR
cd ../..
git add src/agx_arm_ros                          # 更新 gitlink，再提 monorepo PR
```

#### 步骤 E：备选（平铺 vendor，不推荐）

仅在"团队明确禁止 submodule"时采用：只拷 4 个包 + `LICENSE` + `README` + `UPSTREAM.md`
（记录基线 `b9ad14d` 与补丁清单），`agx_arm_urdf` 仍需处理 205 MB meshes（不带则退化）。
代价：上游更新只能手工 diff，正是当前"改过又无历史"困境的来源。

#### 步骤 F：`agx_arm_ws` 里哪些东西**不属于**驱动仓库

| 路径 | 去向 |
|---|---|
| `bringup.sh`、`scripts/*.sh`、`test/` | monorepo `tools/lgw/`（或 `docs/setup/`） |
| `docs/`（容器/驱动笔记） | monorepo `docs/setup/` |
| `piper_bt_planner_old/`（带 `COLCON_IGNORE`） | 丢弃 |
| `build/ install/ log/ .tmp/ tmp_analyze.py` | 丢弃 |

### 2.7 修改第三方库的通用做法

| 改动规模 / 上游活跃度 | 推荐做法 |
|---|---|
| 几行、稳定、一次性的修补 | `patches/*.patch` + bootstrap 脚本 `git apply`；源码树与上游一致，补丁可审计 |
| 有实质改动、需要独立历史与 review、要长期跟上游 | **fork + submodule 固定 commit**（本方案） |
| 通用 bug / 缺失特性 | 先向上游提 PR；合并后删除本地补丁 |
| 整栈照搬、不打算跟上游 | 平铺 vendor（你们对 Point-LIO 的做法） |

三条不变纪律：

1. **许可与署名**：保留上游 `LICENSE`/`NOTICE`；`README` 注明"基于 `<repo>@<commit>`，
   修改清单见 `NOTICE`"（Apache-2.0 要求标注被修改的文件）。
2. **消费方钉死 commit**：submodule gitlink / `deps.lock`，不使用浮动分支。
3. **上游同步 runbook**：`git fetch upstream && git merge upstream/<branch>`，
   冲突固定落在已知的若干文件，合并后更新 monorepo 的 gitlink。

---

## 3. 目标仓库结构

目标仓库是既有团队 monorepo，因此结构按其约定（`src/` 平铺包 + `docs/<主题>/`）组织：

```
Lunar-Grotto-Walker/                # 团队 monorepo（已存在，public）
├── README.md                       # 团队维护，追加"行为树 / 机械臂"章节
├── .gitignore                      # 补 bags/、*.bag、.run/、frames_*（§4）
├── .github/workflows/              # 改造后的 CI（§2.5 风险项）
├── docs/
│   ├── project/                    # 既有：Git Flow 指南、PR/Issue 模板说明
│   ├── slam/                       # 既有
│   ├── bt/                         # 新增：ARCHITECTURE.md、TOPICS.md、启动手册
│   └── setup/                      # 新增：SETUP.md（apt、peer ws、CAN、eth10、相机序列号、YOLOE）
├── tools/
│   ├── ci/                         # 既有
│   ├── monitor_stack.py            # 既有
│   └── lgw/                        # 新增：原 lg_walker_ws 的 scripts/config/test
│       ├── start/  record/  bench/  diag/
│       ├── _common.sh  lgw_env.sh
│       ├── config/                 # dds_local_lo.xml、perception_bridge.{xml,yaml}
│       └── test/                   # YOLOE 端到端脚本
└── src/                            # colcon build 根
    ├── point_lio_ros2/             # 既有（feature/1-pointlio-slam，尚未合入 develop）
    ├── simple_loop_closure_ros2/   # 既有
    ├── lgw_bt_planner/
    ├── lgw_3d_tracker/
    ├── lgw_vui_bridge/
    ├── body_pose_bridge/
    ├── agx_topic_hardware/
    ├── lgw_servo_smoothing/
    ├── servo_state_probe/
    ├── yoloe_interfaces/           # canonical（超集版）
    ├── go2_transit/
    └── agx_arm_ros/                # vendored（改过的 fork；NOTICE 声明）
    └── BehaviorTree.ROS2/          # vendored（含 deadline 修复）
```

> 配套的第二个仓库：`Lunar-Grotto-Walker-Team/lgw-yoloe-perception`（宿主机 Foxy 推理 +
> Humble 相机编排，AGPL-3.0；公开/私有由团队定），不放在本仓库内，详见 §2.4。

> 若团队后续更倾向"引用而非 vendor"，可把新增的 `deps.repos` 放仓库根，与既有
> `src/point_lio_ros2` 的 vendor 做法并存会让人困惑 —— **建议二选一并在 `docs/project/` 写明**。

> 备选：若坚持"clone 即用"，可把 C 类仓库作为 `external/` 下的 submodule + bootstrap 建 symlink。
> 不推荐（ROS 工作区与 submodule 组合笨重，colcon 会误构建），本方案采用 `deps.repos`。

---

## 4. `.gitignore`（在现有基础上补齐）

```gitignore
# 构建产物
build/ install/ log/
# 运行期
.run/
bags/
*.bag *.bag.zstd
frames_*.gv frames_*.pdf
work.log
# 缓存
__pycache__/ *.pyc
# 临时/草稿
.superpowers/
.lg_retained_C_group/
.tmp/
*.tar *.7z *.zip
# 本地覆盖
config/local.env
```

现有 `.gitignore` 缺少 `bags/`、整个 `.superpowers/`（现在只忽略 `sdd/`）、`frames_*`、`work.log`、`*.tar`，
必须补齐后再 `git add`，否则首次提交会包含 64 GB 数据。

---

## 5. 依赖清单与引导脚本（草案）

### 5.1 `deps.repos`

```yaml
repositories:
  # B 类：改过的上游，已 fork 自持
  agx/agx_arm_ros:
    type: git
    url: https://github.com/<you>/agx_arm_ros.git
    version: go2-piper            # b9ad14d + V1–V8 + frame_prefix
  bt/BehaviorTree.ROS2:
    type: git
    url: https://github.com/<you>/BehaviorTree.ROS2.git
    version: lgw-fixes            # deadline 修复 + GOAL_REJECTED

  # C 类：未改，钉住上游 commit
  unitree/unitree_ros2:
    type: git
    url: https://github.com/unitreerobotics/unitree_ros2.git
    version: <commit-待核>
  camera/realsense-ros:
    type: git
    url: https://github.com/IntelRealSense/realsense-ros.git
    version: <tag-待核>
  camera/livox_ros_driver2:
    type: git
    url: https://github.com/Livox-SDK/livox_ros_driver2.git
    version: <tag-待核>
  nav/go2-nav2-terrain-pointlio:
    type: git
    url: https://github.com/<上游或你的导航仓库>
    version: <branch/commit-待核>
```

> 待核项：上传前逐个 `git clone --depth 1` 与本地对比，确认 commit，再填写。

### 5.2 `scripts/bootstrap_deps.sh` 要点

1. `vcs import --recursive < deps.repos` 到 `external/`；
2. 按 peer workspace 拓扑把 `external/*` 链到 `src/`（或在 `SETUP.md` 给出放置说明）；
3. 若使用 patch 方案：对 `external/BehaviorTree.ROS2` 执行 `git apply patches/*.patch`；
4. 打印当前各依赖的 commit，写入 `deps.lock`（便于复现）；
5. 缺 `vcs` 时提示 `pip install vcstool`。

---

## 6. 整理动作清单

### P0（不上传会缺功能 / 会传错东西）

- [ ] 补 `.gitignore`（§4），确认 `git add` 后不含 `bags/`
- [ ] 迁入 `go2_transit`（决定：进本仓库 `src/` 还是单独仓库）
- [ ] 决定 `pointlio_cloud_relay` 归属（BT 仓库 / 导航仓库）
- [ ] 选定 LICENSE，补 `NOTICE`（qpmad、上游仓库链接）
- [ ] 处理 3 个孤儿 worktree（§8.1）
- [ ] 发布 `agx_arm_ros` fork（`go2-piper` 分支），确认 `lgw_bt_planner` 能对新路径构建
- [ ] 处理 `BehaviorTree.ROS2` 修复：fork 或 `patches/`
- [ ] 感知链路：定仓库归属与可见性（§2.4 方案 A/B/C），清理 `.bak*`，
      权重/evidence 不入库，补 `models/README.md` + 导出脚本
- [ ] `yoloe_interfaces` 定 canonical（主仓库超集版），感知侧副本改为同步
- [ ] 修正本地 `l4t/Lunar-Grotto-Walker` 的 remote（cbzZZZZ → 组织仓库）并 fetch
- [ ] 处理未推送的 `feature/1-pointlio-slam`：rebase 到 develop，决定发 PR 或丢弃
- [ ] 改造团队仓库 CI（`c-cpp-ci` 的根 `cmake ..`、`python-ci` 的全仓库黑化/ruff）
- [ ] 补团队仓库 `.gitignore`（`bags/` `*.bag` `.run/` `frames_*`）
- [ ] 团队仓库 LICENSE 与 vendor 声明（`agx_arm_ros`、`BehaviorTree.ROS2`、qpmad）

### P1（别人能不能看懂 / 能不能跑）

- [ ] 重写 `README.md`：架构 mermaid 图 + 4 步启动 + 硬件清单
- [ ] `docs/ARCHITECTURE.md`：§1.2/§1.3/§1.5 表格 + BT XML ↔ 节点矩阵
- [ ] `docs/SETUP.md`：apt 清单、peer workspace 表、CAN/eth10、相机序列号、YOLOE 部署与版本
- [ ] `docs/TOPICS.md`：端点契约
- [ ] `scripts/` 归类到 `start/ record/ bench/ diag/`，`.run/` 的 probe 脚本迁入 `diag/`
- [ ] 相机序列号默认值（`_261622078621` / `_262422071246`）改为空默认 + 必填校验
      —— 出现在 12 个文件：`scripts/` 5 个、`test/` 6 个、`docs/2026-10-20-...` 1 篇
- [ ] `calibration/*.json` 注明是特定机器人标定，换机需重标

### P2（可后续）

- [ ] 硬编码路径收敛：`WS_ROOT=${WS_ROOT:-/my_ws/l4t/lg_walker_ws}`（现状 139 处 `/my_ws/l4t/lg_walker_ws`、114 处 `l4t_setup`）
- [ ] `docs/superpowers/specs|plans` 归档到 `docs/history/`，README 只链 3–5 篇
- [ ] `ros2_ws` 上游比对，确认无改
- [ ] 可选：GitHub Actions 仅跑 lint / 文档检查（重依赖不适合 CI）

---

## 7. 上传步骤（目标：既有团队仓库，Git Flow）

```bash
# 0) 修本地克隆的 remote（当前指向个人仓库 cbzZZZZ，落后于组织仓库）
cd /home/unitree/my_ws/l4t/Lunar-Grotto-Walker
git remote set-url origin https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker.git
git fetch origin
git branch -vv                      # 应看到 main=3230632、develop=5deb355

# 1) 未推送的 feature/1-pointlio-slam 先对齐最新 develop
git checkout feature/1-pointlio-slam
git rebase origin/develop
# 决定：SLAM 与行为树分两个 PR 还是合并成一个（建议分开 review）

# 2) 从 develop 开行为树工程的 feature 分支（命名遵循 feature/<issue-id>-<desc>）
git checkout -b feature/<issue-id>-lgw-bt-stack origin/develop

# 3) 按 §3 落文件（cp -a；不要带 build/ install/ log/ bags/ .run/）
#    src/<8 个包>、src/go2_transit、src/agx_arm_ros、src/BehaviorTree.ROS2
#    tools/lgw/{start,record,bench,diag,config,test}
#    docs/bt/{ARCHITECTURE,TOPICS}.md、docs/setup/SETUP.md

# 4) 提交前自查
git add -A
git status --short | wc -l
git ls-files -z | xargs -0 du -ch | tail -1   # 预期 < 15 MB

git commit -m "feat(bt): 导入行为树 + 机械臂控制栈（8 包 + go2_transit）"

# 5) 推送并开 PR
git push -u origin feature/<issue-id>-lgw-bt-stack
# PR: feature → develop（1 人 review）→ 合并后 develop → main（2 人 review）
```

注意事项：

- **不能直接推 `main` / `develop`**（有分支保护），必须走 PR。
- PR 会触发 CI；上传前先按 §2.5 完成 CI 改造（可与本 PR 同一批，也可先发一个 CI 修复 PR）。
- 感知链路若独立成仓库，同样在 `Lunar-Grotto-Walker-Team` 下建 `lgw-yoloe-perception`，
  走同样的 Git Flow。
- 本地 `lg_walker_ws` 之后退化为历史开发目录；日常改动应在 monorepo 的
  `src/` 里做，避免两边再次分叉。

---

## 8. 风险与遗留

### 8.1 `.git` 丢失与孤儿 worktree（阻塞项）

实测：

- `lg_walker_ws_feature_detect_and_move/.git`、`lg_walker_ws_yu/.git`、`baseline_wt/.git`
  内容均为 `gitdir: /home/unitree/my_ws/l4t/lg_walker_ws/.git/worktrees/<name>`；
- 主仓库 `lg_walker_ws/.git` **已不存在**，且未找到 `worktrees/`、objects 或 tar 备份。

处理：

1. 确认是否曾 `git push` 过（GitHub 上是否有旧仓库）；有则 clone 恢复历史；
2. 无则接受"历史从 2026-11-26 开始"，并处理孤儿 worktree：
   - `lg_walker_ws_feature_detect_and_move`（1.5 M）与 `lg_walker_ws_yu`（280 K）内容较少，
     用 `diff -rq` 与主目录对比后，把独有内容合并回来再删除；
   - `baseline_wt`（3.4 M）是某个基线快照，确认无需后删除或归档到仓库外。

### 8.2 硬件与外部部署不可复现

- YOLOE 推理在宿主机 Foxy（`$HOME/yoloe_deploy`），双相机 launch 在 `/opt/yoloe_transfer/humble_ws`，
  不属于本仓库 → `SETUP.md` 必须写清版本、权重路径、启动顺序，并说明缺失时的降级表现
  （`/perception/two_stage/*` 无数据，感知类 BT 节点会失败）。
- `calibration/*.json` 与相机序列号是单机数据，README 需提示换机重标/改序列号。
- `domain_bridge` 跨域（domain 7 → 0）依赖 `config/perception_bridge.{xml,yaml}`，属仓库内资产，勿漏。

### 8.3 许可

- 主仓库 LICENSE 待定；`thirdparty/qpmad` 的 `NOTICE`/`LICENSE` 必须随仓库保留。
- `agx_arm_ros`（MIT，版权人 aalicecc）、`BehaviorTree.ROS2`（MIT）、`unitree_ros2` 等
  在 `NOTICE` 中列明来源与许可证。

---

## 9. 验收清单

- [ ] `git ls-files | wc -l` 与 `du -sh` 符合预期（无 bags、无 build/install/log）
- [ ] `grep -rn "261622078621"` 只出现在文档示例或已被参数化（当前 12 处，见 §6 P1）
- [ ] 干净目录 `git clone` + `vcs import` + `bootstrap_deps.sh` 后，
      `colcon build` 能通过 8 个自写包（外部包按 `SETUP.md` 走容器/apt）
- [ ] `ros2 launch lgw_bt_planner lgw_chain.launch.py` 在目标机器上能拉起
      （有硬件时）或给出清晰缺失提示（无硬件时）
- [ ] README 的启动步骤与 `scripts/start/start_lgw_chain.sh` 实际行为一致
- [ ] 未部署感知链路时，主仓库按 README 能起来，且缺失的 BT 节点失败信息可读
- [ ] `yoloe_interfaces` 两侧版本一致（或 README 写明兼容矩阵）

---

## 10. 执行记录

### 2026-11-27：agx_arm_ros 已发布到组织（§2.6 步骤 A/B 完成）

| 项 | 值 |
|---|---|
| 组织仓库 | `Lunar-Grotto-Walker-Team/agx_arm_ros`（由网页 Fork，保留 forked-from） |
| 基线 | `b9ad14d`（2026-09-04） |
| 分支 | `main` = `cae02c9`（4 个提交） |
| Tag | `lgw-v1`（annotated） |
| 上游 remote | `upstream` → `agilexrobotics/agx_arm_ros`，`ros2` 分支未改动 |
| 子模块 | `agx_arm_urdf` → `f6642ce`（baseline 对应提交，meshes 已拉齐） |
| 校验 | `git status` 干净；`git diff --stat b9ad14d..main` = 10 文件 133+/25−；py/xml 语法通过 |

4 个提交：`fix(go2): park home pose` → `feat(moveit): topic hardware backend` →
`feat(moveit): TF frame_prefix` → `chore(test): gripper joint state sample`。

待办：仓库 Settings → Branches 把默认分支改为 `main`；随后按 §2.6 步骤 C 在 monorepo
以 submodule 接入（需要 Issue 编号）。

### 2026-11-27（下午）：改为 vendor 路线，已推 Vendor 仓库分支

团队 `develop` 已建立 Vendor 机制（`Lunar-Grotto-Walker-Vendor` + 根 `deps.repos` +
`docs/third-party-modes.md`），且 `BehaviorTree.ROS2` 的 deadline 修复已在其中。
据此改用 vendor 模式：

| 项 | 值 |
|---|---|
| 分支 | `feature/agx-arm-driver`（基于 `origin/develop` b4cf047） |
| 导入 | `git subtree add -P src/agx_arm_ros b9ad14d --squash` |
| 子模块 | `agx_arm_urdf`（205 MB）剔除 gitlink 与 `.gitmodules`，改由 `deps.agx-arm-urdf.repos` 钉 `f6642ce`，导入到 `agx_arm_description/` 内 |
| 补丁 | 4 个提交（park 位姿 / topic 后端 / frame prefix / 测试采样），SHA 见仓库 |
| 文档 | `UPSTREAM.md`、NOTICE、`docs/third-party-modes.md`、`src/README.md`、`README.md` 已同步 |
| 规模 | 102 文件 / +10060（绝大部分是导入的上游树），`.git` 8.1 MB |
| 状态 | 已推送，PR 待开：`.../Lunar-Grotto-Walker-Vendor/pull/new/feature/agx-arm-driver` |

早先建的 `Lunar-Grotto-Walker-Team/agx_arm_ros` fork 在 vendor 路线下不再必需
（vendor 用 subtree 直接跟上游），建议 PR 合并后删除，避免两套来源。
