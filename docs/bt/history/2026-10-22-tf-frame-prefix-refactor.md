# TF frame 前缀重构(Plan A)+ Round 4 诊断更正

**日期**:2026-10-22
**范围**:`lgw_3d_tracker` / `lgw_bt_planner` / `agx_arm_moveit` / `start_lgw_arm.sh` 的 TF 帧名治理
**状态**:代码与 launch 改动完成,container 内 build 通过,**真机全栈已验证**
(真双 realsense + 真 `robot_state_publisher` + 真 piper URDF + 真 tracker,2026-10-22)

> 📖 **只想知道"为什么做、怎么用"** → 看 [`tf-frame-prefix.md`](./tf-frame-prefix.md)(设计/使用说明)。
> 本文是**排查过程记录**(根因复现、官方仓库对照、裁剪依据),适合溯源。

> 📖 **只想知道"为什么做、怎么用"** → 看 [`tf-frame-prefix.md`](./tf-frame-prefix.md)(设计/使用说明)。
> 本文是**排查过程记录**(根因复现、官方仓库对照、裁剪依据),适合溯源。

## 1. 为什么做这件事

`docs/2026-10-20-lgw-3d-tracker-real-hw-verification.md` 里 Round 4 把
"head 相机静态 TF lookup 不到"归因于 **cyclonedds+eth10 下 TRANSIENT_LOCAL latch 不可靠**。
**该诊断经纯软件复现证明是错的**,本文件给出更正与后续重构。

### 1.1 纯软件复现(排除一切 DDS 因素)

单进程内用 `TransformBroadcaster` 周期发边,构造"同一 child frame 有两个 parent":

| 场景 | 结果 |
|---|---|
| C1 对照(单 parent) | `OK=20 FAIL=0` |
| C2 `base_link` 双 parent(world + odom) | `OK=16 **FAIL=4**` |
| C3 `camera_X_color_optical_frame` 双 parent | `OK=16 **FAIL=4**` |
| C4 两者叠加 | `OK=13 **FAIL=7**` |

失败信息与真机完全一致:
```
Could not find a connection between 'odom' and 'camera_head_color_optical_frame'
because they are not part of the same tree.Tf has two or more unconnected trees.
```

**根因**:tf2 的 `getParent()` 从 unordered_map 取,顺序不确定。同一个 child frame
有两条 parent edge 时,走链可能走到"错"的分支,撞到另一个 root → 报 unconnected trees。
**冲突边被周期性重发时失败持续**(每次重发都重新掷一次骰子);一次性 publish 的边只在到达瞬间扰动。

### 1.2 真机上的三处冲突

| # | 子 frame | parent A | parent B |
|---|---------|---------|---------|
| 1 | `base_link` | `odom`(动态,tracker 从 `/utlidar/robot_pose` 转) | `world`(机械臂 URDF 的 `world_to_base_link` fixed joint,identity) |
| 2 | `camera_head_color_optical_frame` | `base_link`(tracker 的头相机标定桥) | `camera_head_color_frame`(realsense2_camera_node 的 URDF) |
| 3 | `camera_arm_color_optical_frame` | `tcp_link`(CLI 标定桥) | `camera_arm_color_frame`(realsense2_camera_node 的 URDF) |

补充事实(查证):
- `realsense2_camera_node` 的 `base_frame_id` 参数**不是外部 parent**,而是相机树内部的
  link 名 —— `parameters.cpp:83` 给它加 `camera_name` 前缀:
  `_base_frame_id = (_camera_name << "_" << _base_frame_id).str()`。
  默认 `'link'` → `camera_head_link` / `camera_arm_link` 恒为**相机树的 root**,
  节点**没有任何"把相机挂到机器人本体上"的机制** → 静态桥是架构必需,不是 hack。
- `robot_state_publisher` 的**固定关节发一次 /tf_static**,动态关节周期发 /tf。
- `static_transform_publisher` (CLI) 是**周期重发**;rclpy 的
  `StaticTransformBroadcaster.sendTransform()` 只发一次。
- **一次性 publish 的 `/tf_static` 对 late-joining subscriber 是**可靠**的**
  (2026-10-22 更正):`TRANSIENT_LOCAL` 的 latch 机制本身没问题,唯一约束是
  **publisher 进程必须常驻**。`robot_state_publisher` / `realsense2_camera_node` /
  tracker 都常驻 → 它们的一次性 `/tf_static` 能被任意晚启动的消费者收到。
  之前的 `ONESHOT_LATE_LOOKUP_FAILED` 是**测试方法错误**:那次 publisher 1 秒后
  就退出了,latched 数据随 writer 消失,不是 latch 的问题。
  → **固定变换不需要周期重发**,`TransformBroadcaster` + timer 那套可以去掉。
- ⚠️ **fake realsense 不能代表真实节点**:真 `realsense2_camera_node` 每次
  `publishStaticTransforms()` 都 `restartStaticTransformBroadcaster()`
  (销毁旧 writer 重建,`tfs.cpp:250`),发 8 条边(color+depth);
  rclpy fake 是 4 条边、单次 `sendTransform`。二者行为不同,别用 fake 下结论。

## 2. 方案:官方 frame_prefix(Plan A)

用户决策:**保持规范** —— 采用上游 `agx_arm_description/launch/display.launch.py`
里的官方做法(ROS namespace → RSP 的 `frame_prefix` 参数),下游做补偿,
不改机械臂 URDF、不改 MoveIt 的 robot model。

上游原厂写法:
```python
# agx_arm_description/launch/display.launch.py:106-112
# Multi-arm: RSP needs frame_prefix (e.g. left/) so /tf frames differ; RViz RobotModel
frame_prefix = f'{ns}/' if ns else ''
if frame_prefix:
    rsp_params['frame_prefix'] = frame_prefix
```
`robot_state_publisher` (Humble) 原生支持 `frame_prefix` 参数(实测参数列表里有)。

### 2.1 为什么**不用** ROS namespace

`demo.launch.py` 有 `namespace` arg + `PushRosNamespace`,但**开了会破坏接线**:
- `move_group` 进 `/arm1` → action server 变 `/arm1/move_action`,
  而 `start_lgw_arm.sh:175` 检查的是 `^/move_action$` → 失败
- `move_group` 的 `joint_states` remap 到 `/arm1/feedback/joint_states`,
  而 driver 发在根 `/feedback/joint_states` → 对不上

RSP 的 TF publisher 用的是**绝对 topic**(`/tf`、`/tf_static`,实测 namespace 下仍在根),
所以 namespace 不影响 TF 投递 —— 但上面两条接线问题决定了不能用它。
**只加 `frame_prefix`,namespace 保持 `""`。**

### 2.2 重构后的 TF 树(实测)

```
odom → base_link                     (动态 /tf,tracker 转 SLAM pose)   ✓ 唯一 parent
     └── arm1/world                  (static, tracker 挂载桥)          ✓
          → arm1/base_link           (static, URDF fixed joint)        ✓
               ├── camera_head_link        (static, tracker 桥)        ✓
               │     ├── camera_head_color_frame → …_optical_frame     ✓
               │     └── camera_head_depth_frame → …_optical_frame     ✓
               → arm1/link1 → … → arm1/link6                          ✓
                    ├── arm1/tcp_link                                 ✓
                    │     → camera_arm_link    (static, tracker 桥)   ✓
                    │       ├── camera_arm_color_frame → …_optical_frame ✓
                    │       └── camera_arm_depth_frame → …_optical_frame ✓
                    └── arm1/flange_link → gripper_base → gripper_link{,1,2} ✓
```

真机实测:`/tf_static` 共 **15 条边**、`/tf` 只有 `odom → base_link` 1 条动态边,
每个 frame 恰好一个 parent。

**关键教训(踩过)**:挂载桥必须接在**链的 root** 上 —— `base_link → arm1/world`。
如果接 `base_link → arm1/base_link`,会给 `arm1/base_link` 加第二个 parent
(URDF 里它已有 parent `arm1/world`)→ 歧义原地复活,实测
`base_link → arm1/tcp_link` 变成 `OK=0 FAIL=25`。

同理,head 相机桥接 `arm1/base_link → camera_head_link`(相机**体** link),
不能接 `..._optical_frame`(后者已有 realsense 的 parent)。

⚠️ **head 桥的 parent 是机械臂基座,不是 Go2 躯干**(2026-10-22 修正)。
`head_calibration.json` 是 eye-to-hand 产物,其 `base_link` 指机械臂 URDF 的基座
——设计文档里的判据是"`base_link → world = identity`",而 `world→base_link` 的
fixed joint 属于机械臂。此前 tracker 把它当成 Go2 躯干的 `base_link` 用,只因
`arm_mount_offset` 还是单位阵而数值巧合一致。

### 2.3 消费端规避:分段查 + 矩阵乘

即使图修好了,tracker 也不再做整链 lookup:

```
head 相机:  T_cam_odom = T_base_odom ⊗ T_armbase_base ⊗ T_cam_armbase
arm  相机:  T_cam_odom = T_base_odom ⊗ T_tcp_base      ⊗ T_cam_tcp
```
- `T_base_odom` = tf lookup `odom → base_link`(base_link 唯一 parent=odom ✓)
- `T_armbase_base` = tf lookup `base_link → arm1/base_link`(head 标定的基准是机械臂基座)
- `T_tcp_base` = tf lookup `base_link → arm1/tcp_link`
  (lookup 的**上界是 base_link**,走链到 base_link 即停,base_link 自己的 parent 不参与判定 ✓)
- `T_cam_base` = `head_calibration.json`(camera→base_link),纯矩阵
- `T_cam_tcp`  = `arm_calibration.json`(camera→tcp_link),纯矩阵

**标定方向:一律不取逆。** 两份标定都是"相机 → 机器人侧",与 tf edge 语义
(parent=P, child=C 表示 `p_P = R·p_C + t`)同构;矩阵复合律
`[cam]_odom = [base]_odom ⊗ [cam]_base` 方向天然对上。
数值验证(随机位姿,含非平凡旋转):head/arm 复合误差均 < 1e-15;
若错误取逆,误差量级 0.6~5 m。

## 3. 改动清单(裁剪后:**7 个文件**)

初版实现改了 17 个文件,经"官方仓库对照 + 实测"后裁剪到 **7 个**(1 个机械臂侧
launch + 1 个启动脚本 + 5 个 tracker 文件)。下面是最终形态。

### 机械臂侧(agx_arm_ws,nested repo)—— 1 个文件
| # | 文件 | 改动 |
|---|---|---|
| A1 | `agx_arm_moveit/launch/rsp.launch.py` | 本文件内 `DeclareLaunchArgument("frame_prefix")` + 派生规则 + 传给 RSP。**这是整个前缀改造的全部** |

派生规则(与上游 `agx_arm_description/launch/display.launch.py:106-112` 语义同构):
```python
frame_prefix = LaunchConfiguration("frame_prefix").perform(context)
namespace = LaunchConfiguration("namespace").perform(context)
if not frame_prefix and namespace:
    frame_prefix = f"{namespace.strip('/')}/"
```

### 启动脚本 —— 1 个文件
| # | 文件 | 改动 |
|---|---|---|
| B1 | `scripts/start_lgw_arm.sh` | `demo.launch.py` 加一行 `frame_prefix:='arm1/'` |

### tracker(lgw_3d_tracker)—— 5 个文件
| # | 文件 | 改动 |
|---|---|---|
| C1 | `unified_track/calibration.py` | 重写:标定加载(head/arm)+ 4x4 构造 + `broadcast_arm_mount_tf`(base_link→`<prefix>world`)+ `broadcast_camera_mount_tf`(head 与 arm 共用)。删掉旧的 `broadcast_head_static_tf` |
| C2 | `unified_track/input_adapter.py` | 重写:分段查 + 矩阵乘;arm 链目标帧 `arm1/tcp_link` |
| C3 | `unified_track/config.py` | 新增 `arm_frame_prefix`、`arm_mount_offset` |
| C4 | `unified_track_node.py` | 接 **3 条**挂载桥(base_link→world / **arm1/base_link→head cam** / tcp_link→arm cam);建 buffer+listener 的时机提前到桥之前;给 InputAdapter 传标定矩阵 |
| C5 | `config/runtime.yaml` | 新增 `arm_frame_prefix: "arm1/"`、`arm_mount_offset: [0,0,0,0,0,0]` |

(`unified_track/odom_tf_broadcaster.py` 的改动是 Round 2 的 SLAM stamp drift
fallback,不属于本次前缀改造。)

### 裁剪掉的部分及依据
| 裁掉 | 依据 |
|---|---|
| `agx_arm_moveit/launch/_moveit_config_builder.py` | arg 声明挪进 `rsp.launch.py`(launch arg 会向下传播到 include 它的 launch) |
| `agx_arm_moveit/launch/move_group.launch.py` | 加的 `"planning_frame": "base_link"` **冗余** —— 实测 `ros2 param get /move_group planning_frame` 默认就是 `base_link`(MoveIt 2 从 SRDF 的 `<chain base_link="base_link" tip_link="tcp_link"/>` 推导,**不是**回落 URDF root) |
| `lgw_bt_planner` 全部 8 个文件 | MoveIt 的 planning pipeline 用 `Resolve constraint frames to robot links` adapter 把约束 frame_id 对 **robot model** 解析(不是 TF);model 里 `world` 仍在,且 `world→base_link` 是 identity → goal 标 `"world"` 数值等价,**不用改** |

### 明确**不需要**改
- `lgw_bt_planner` 里所有 `tcp_link` 引用(`pc.link_name` / `oc.link_name` /
  `ik_link_names`)—— 那是 **MoveIt 模型帧名**,模型不带前缀 ✓
- `arm_calibration.json` / `head_calibration.json` 的**数值** ✓
- `agx_arm_ctrl` driver 的 `feedback/tcp_pose` —— frame_id 为空字符串 ✓
- 机械臂 URDF 本身(Plan A 的核心就是不改它) ✓
- **BT planner 的 `world_frame` 默认值** —— 保持 `"world"`(见上)✓

## 4. 已完成的验证

| 项 | 结果 |
|---|---|
| `lgw_3d_tracker` build | ✓ |
| `lgw_bt_planner` build | ✓ 1m4s |
| `agx_arm_moveit` build | ✓ |
| `frame_prefix` arg 跨 include 传播 | ✓ `demo.launch.py frame_prefix:='arm1/'` → rsp.launch.py 收到 → RSP 发 `arm1/*` |
| TF 图单 parent 校验 | ✓ 手工拉全 `/tf` + `/tf_static` 的 edge 列表比(**`view_frames` 看不出多 parent**,见 §1.1) |
| tracker 启动 | ✓ 三条桥全部发布(一次性 `/tf_static`):`base_link → arm1/world`、`base_link → camera_head_link`、`arm1/tcp_link → camera_arm_link` |
| tracker 的两段 lookup | ✓ `odom→base_link` / `base_link→arm1/tcp_link` / `odom→arm1/tcp_link` 全 `OK=25 FAIL=0` |
| 旧无前缀链 | ✓ 已不存在(`base_link→tcp_link` 报 frame 不存在) |
| move_group 启动 | ✓ action server 就绪,无 frame 相关报错 |
| `namespace:=arm1` 派生路径 | ⚠️ **未实测**(仅验证了显式 `frame_prefix` 路径) |
| **真机全栈**(真双 realsense + 真 RSP + tracker) | ✓ `/tf_static` 15 条 + `/tf` 1 条(odom→base_link),每帧唯一 parent |
| **两个 late-joiner probe** | ✓ 10/11 lookup OK;唯一 FAIL 是裸名 `tcp_link`(预期) |
| **`/perception/tracks/unified`** | ✓ 9.6–10.0 Hz,`frame_id=odom`,3 tracks |
| **1.1 的多 parent 失败在真机复现了吗** | ❌ **没有**。全栈 10/10 OK —— 说明前缀隔离后该风险已消除;1.1 的复现仅证明机制存在,不代表现网仍暴露 |

## 5. 已知局限 / 待办

1. **机械臂安装偏移未实测**。`arm_mount_offset` 当前是单位阵,语义等价
   "机械臂安装座 ≡ Go2 base_link"。不补测则机械臂在 odom 系的**绝对**位置有
   系统偏差(相对位姿正确)。补测后只改 `runtime.yaml` 一个数组。
2. **MoveIt 侧隐含"机械臂安装座 ≡ Go2 base_link"的恒等假设**。MoveIt 的
   `planning_frame` 必须在 robot model 里存在 → 只能是 `base_link` 或 `world`,
   不能是 `arm1/base_link`(只存在于 TF)。与第 1 点同源。
3. **`world` 在 TF 里消失**(变成 `arm1/world`)。BT planner 的 `world_frame`
   保持 `"world"`(模型内有,MoveIt 用模型解析)→ 功能正常;但任何**直接对 TF
   查 `world`** 的外部代码会失败。
4. **两个"世界系"并存**:`world`/`base_link`(机器人躯干系,BT 侧)与 `odom`
   (SLAM 漂移系,tracker 侧)。`UnifiedTrackArray` 目前无下游消费者;等 BT 要
   消费 track 时需统一(建议 `base_link`)。
5. **RViz 配置待改**:`agx_arm_moveit/config/moveit.rviz` 的 RobotModel TF Prefix
   应设 `arm1/`、Fixed Frame 设 `base_link`。当前 `start_lgw_arm.sh` 用
   `use_rviz:=false`,暂不影响。
6. ~~**真机端到端回归未做**~~ → ✅ **2026-10-22 已完成**。剩余未覆盖项:
   **真实机械臂运动中的 arm 相机位姿正确性**,以及双相机 merge 的**数值一致性**
   (本次只验证了链路连通性与输出频率)。
7. **head 标定的 base_link 归属待核对**:`head_calibration.json` 里的 base_link
   是 Go2 的还是机械臂的?若标定时用的是机械臂安装座,head 相机链也要补安装偏移。

## 6. 相关引用

- `docs/2026-10-20-lgw-3d-tracker-real-hw-verification.md`(前 4 轮,Round 4 结论已更正)
- memory `feedback_tf2_multi_parent_ambiguity`(多 parent 歧义,本文核心)
- memory `feedback_cyclonedds_transient_local_unreliable`(一次性 /tf_static 的行为)
- 上游写法:`agx_arm_description/launch/display.launch.py:106-112`
- realsense 帧名机制:`ros2_ws/src/realsense-ros/realsense2_camera/src/parameters.cpp:83`
