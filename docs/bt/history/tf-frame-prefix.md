# TF 帧名前缀隔离:为什么做、怎么实现

**适用范围**:`lgw_walker_ws` 整机(Unitree Go2 底盘 + piper 机械臂 + 双 realsense)在同一张
ROS 2 tf2 图里协作时的帧名约定。
**状态**:已实现并**真机全栈验证**(容器内 build + 真机双 realsense + 真 `robot_state_publisher`
+ 真 piper URDF + 真 tracker,2026-10-22)。
**相关**:完整根因排查过程见 [`2026-10-22-tf-frame-prefix-refactor.md`](./2026-10-22-tf-frame-prefix-refactor.md);
更早的 tracker 真机验证见 [`2026-10-20-lgw-3d-tracker-real-hw-verification.md`](./2026-10-20-lgw-3d-tracker-real-hw-verification.md)。

---

## 1. 背景:一台机器上有三套独立的 URDF

| 来源 | 发布的 frame 链 | 谁 publish |
|---|---|---|
| Go2 底盘(SLAM) | `odom → base_link` | `lgw_3d_tracker` 的 `OdomTFBroadcaster` |
| piper 机械臂 | `world → base_link → link1 → … → link6 → tcp_link` | `robot_state_publisher`(MoveIt 的 RSP) |
| 双 realsense 相机 | `camera_head_link → camera_head_color_frame → …_optical_frame`(arm 相机同理) | `realsense2_camera_node` |

三套 URDF 各有各的 root,却全被塞进**同一个 tf2 buffer**。而 tf2 的 frame 名是
**全局唯一 key**,没有命名空间概念 —— 于是"两套 URDF 都叫 base_link"必然撞车。

---

## 2. 为什么必须做帧名隔离

### 2.1 tf2 没有 frame 命名空间,ROS namespace 也不管用

- tf2 的 frame 名是扁平全局 key,同一个名字只能有一份
- `robot_state_publisher` 的 TF publisher 发到**绝对 topic** `/tf` / `/tf_static`
  → **ROS namespace 不会改帧名**。实测:`-r __ns:=/arm1` 时,根 `/tf_static` 照样收到
  机械臂的 frame
- 所以隔离 TF 帧名只有一个官方手段:**`robot_state_publisher` 的 `frame_prefix` 参数**

> 官方 upstream(`agilexrobotics/agx_arm_ros`,ros2 分支)**只在
> `agx_arm_description/launch/display.launch.py` 接了这一环**(可视化路径),
> **MoveIt 路径(`demo.launch.py` → `rsp.launch.py`)没有** —— 也就是说官方
> "多臂 + MoveIt" 这个组合在 TF 层是撞名的。我们补的就是这一环。

### 2.2 本项目具体撞在哪

| # | 同一个 child frame | parent A | parent B |
|---|---|---|---|
| 1 | `base_link` | `odom`(底盘,动态 19 Hz) | `world`(机械臂 URDF 的 `world_to_base_link` fixed joint) |
| 2 | `camera_head_color_optical_frame` | `base_link`(我们发的标定桥) | `camera_head_color_frame`(realsense URDF) |
| 3 | `camera_arm_color_optical_frame` | `tcp_link`(我们发的标定桥) | `camera_arm_color_frame`(realsense URDF) |

### 2.3 撞名的真正后果:tf2 的多 parent 歧义

**机制**:tf2 的 `getParent()` 从 unordered_map 里取,同一个 child frame 有两条
parent edge 时,**取到哪条不确定** → 走链时可能走到"错"的分支,撞到另一个 root:

```
Could not find a connection between 'odom' and 'camera_head_color_optical_frame'
because they are not part of the same tree.Tf has two or more unconnected trees.
```

**实测(单进程周期发边,完全排除 DDS 因素)**:

| 场景 | 结果 |
|---|---|
| 单 parent(对照) | `OK=20 FAIL=0` |
| child 有 2 个 parent | `OK=16 **FAIL=4**` |
| 2 处冲突叠加 | `OK=13 **FAIL=7**` |

**冲突边被周期性重发时失败持续**(每次重发都重新掷一次骰子);一次性 publish 的边
只在到达瞬间扰动。

⚠️ **`view_frames` 看不出来**:它只打印"作为 child 出现过的 frame",而 root
(`odom` / `world`)只当 parent 不当 child → **根本不出现**;而且每个 frame 只显示
一个 parent。排查时必须手工把 `/tf` + `/tf_static` 的所有 edge 拉出来比。

### 2.4 只改消费者够不够?

不够,但能先止血。让每次 lookup 的**上界明确**(见 §4.3),tracker 就不再被歧义打挂;
但 `base_link` 仍是双 parent → **MoveIt、rviz、任何做 `lookup_transform("odom","base_link")`
的节点仍然暴露在 25% 随机失败里**。

所以本项目**两件事都做**:
1. **图上隔离** —— 机械臂的帧加前缀,和底盘不再重名(§4.1)
2. **消费者补偿** —— tracker 改分段查(§4.3)

---

## 3. 方案:采用官方 `frame_prefix`,不改 URDF / 不改 MoveIt model

官方 upstream 的写法(`agx_arm_description/launch/display.launch.py:106-112`):

```python
# Multi-arm: RSP needs frame_prefix (e.g. left/) so /tf frames differ; RViz RobotModel
frame_prefix = f'{ns}/' if ns else ''
if frame_prefix:
    rsp_params['frame_prefix'] = frame_prefix
```

**为什么选它**:
- 不改 vendored 的机械臂 URDF(5 个机型变体)
- 不改 MoveIt 的 `robot_description` / SRDF / kinematics —— **模型帧名保持规范**
  (`world` / `base_link` / `tcp_link`),所有 `ik_link_names` / `link_name` 引用零改动
- 参数化,一个 `arm1/` 就切掉

**为什么不用 ROS namespace**:namespace 会连带把 action server 变成 `/arm1/move_action`、
话题变成 `/arm1/...` → 波及 6 个启动脚本的 `^/move_action$` 检查 +
`move_arm_to_pose_action_node.cpp` 的客户端 + BT 树 XML。我们只有一条臂,话题隔离没有
实际收益,**只取"帧名隔离"那一半**。

---

## 4. 本项目怎么实现

### 4.1 机械臂侧:给所有 frame 加前缀

**唯一改动文件**:`agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/launch/rsp.launch.py`

```python
# 1) 声明 launch arg(放在 rsp.launch.py 内部,不进 declare_common_args)
DeclareLaunchArgument(
    "frame_prefix", default_value="",
    description="TF frame name prefix for the arm chain, e.g. 'arm1/'. ..."),

# 2) 派生规则:显式优先,否则按 namespace 派生(与官方 display.launch.py 同构)
frame_prefix = LaunchConfiguration("frame_prefix").perform(context)
namespace = LaunchConfiguration("namespace").perform(context)
if not frame_prefix and namespace:
    frame_prefix = f"{namespace.strip('/')}/"

# 3) 传给 RSP
parameters=[moveit_config.robot_description, {"frame_prefix": frame_prefix}],
```

**`frame_prefix:='arm1/'` 的传递链**(每一跳都有依据):

| # | 位置 | 机制 |
|---|---|---|
| 1 | `ros2 launch` CLI | `name:=value` 写进顶层 `context.launch_configurations` |
| 2 | `demo.launch.py` → `IncludeLaunchDescription(rsp.launch.py)` | included entity 在**同一个 context** 执行;我们没传 `launch_arguments` → 不生成 `SetLaunchConfiguration` 覆盖 |
| 3 | `DeclareLaunchArgument("frame_prefix", default_value="")` | 源码 `if self.name not in context.launch_configurations:` → **已在就不写默认值**,保留 CLI 的值 |
| 4 | `OpaqueFunction(_launch)` | `LaunchConfiguration("frame_prefix").perform(context)` |
| 5 | `Node(parameters=[...])` | 作为节点参数下发 |
| 6 | RSP(C++) | 构造时读它,给**所有发布的 frame** 加前缀 |

**效果**:
```
world → base_link → link1 → … → tcp_link
  ⇒  arm1/world → arm1/base_link → arm1/link1 → … → arm1/tcp_link
```

启动脚本 `scripts/start_lgw_arm.sh` 里一行:
```bash
ros2 launch agx_arm_moveit demo.launch.py ... frame_prefix:='arm1/' \
```

### 4.2 挂载桥:把三棵树连起来

加前缀后机械臂链自成一体(`arm1/world` 是它的 root),和底盘的 `odom → base_link`
**是两棵独立的树**;两台 realsense 的树(`camera_X_link` 为 root)也是独立的。
三条桥全部由 `lgw_3d_tracker` 补,否则 arm 相机的观测无法换算到 odom 系。

由 `lgw_3d_tracker` 发(`unified_track/calibration.py::broadcast_arm_mount_tf`):

```
base_link ──[挂载桥]──▶ arm1/world ──▶ arm1/base_link ──▶ … ──▶ arm1/tcp_link
```

⚠️ **桥必须接在链的 ROOT(`arm1/world`)上,不能接 `arm1/base_link`**:
URDF 里 `arm1/base_link` 已经有 parent `arm1/world`;再补一条
`base_link → arm1/base_link` 会让它变成**双 parent** → 歧义原地复活
(实测 `base_link → arm1/tcp_link` 从 `OK=25` 变成 `OK=0 FAIL=25`)。

**三条桥全部用 `StaticTransformBroadcaster` 一次性发到 `/tf_static`**
(不是 `TransformBroadcaster` + timer 周期重发):

| 桥 | 函数 | 数值来源 |
|---|---|---|
| `base_link → {prefix}world` | `broadcast_arm_mount_tf` | `arm_mount_offset`(未实测) |
| **`{prefix}base_link → camera_head_link`** | `broadcast_camera_mount_tf` | `head_calibration.json`(camera→**机械臂基座**)⊗ inv(realsense 内部边) |
| `{prefix}tcp_link → camera_arm_link` | `broadcast_camera_mount_tf` | `arm_calibration.json`(camera→tcp_link)⊗ inv(realsense 内部边) |

⚠️ **head 桥的 parent 是机械臂基座 `{prefix}base_link`,不是 Go2 躯干的 `base_link`**。
`head_calibration.json` 是 eye-to-hand 产物,其 `base_link` 指机械臂 URDF 的基座
(`world→base_link` 为 identity 的那个),佐证见
`docs/superpowers/specs/2026-10-01-lgw-head-to-arm-lookat-design.md`。
两个同名帧不是同一帧 —— 2026-10-22 曾因此挂错父节点,当时只因 `arm_mount_offset`
是单位阵而数值恰好一致。

`StaticTransformBroadcaster` 的 QoS 是 **depth=1 + KEEP_LAST + TRANSIENT_LOCAL**:
同一次 `sendTransform()` 里的多条边可以一起投递,但**分开调用只留最后一条** ——
所以 rclpy 侧要么批量发、要么每条边各用一个 broadcaster(tracker 是后者)。

**latch 可靠的前提:publisher 必须常驻。**

| publisher 行为 | 晚启动的订阅者 |
|---|---|
| 常驻(本节点 / `robot_state_publisher` / `realsense2_camera_node`) | ✅ **能**收到 |
| publish 后进程退出(一次性脚本) | ❌ 收不到(数据随 writer 消失) |

2026-10-22 实测:2-reader 实验(早/晚两个 reader)与真机全栈(两个独立 late-joiner probe)
全部通过。**因此不需要周期重发**,固定变换归 `/tf_static`,动态的只留 `odom → base_link`。

> ⚠️ **别用 fake realsense 验证相机链**。真实 `realsense2_camera_node` 每次
> `publishStaticTransforms()` 都先 `restartStaticTransformBroadcaster()`
> **销毁旧 writer 再重建**(见 `realsense2_camera/src/tfs.cpp:250`),并且发
> **8 条边**(color + depth 各一组);rclpy 写的 fake(4 条边、单次 `sendTransform`)
> 行为不同。我曾因 fake 的异常表现误判"相机光学帧 lookup 不到"是 latch 问题 ——
> 真机验证必须用真实节点。

**数值** = 机械臂在 Go2 躯干上的安装偏移,当前 `arm_mount_offset` 是单位阵
(未实测,等价"机械臂安装座 ≡ Go2 base_link")。

### 4.3 消费者补偿:分段查 + 矩阵乘

tracker(`unified_track/input_adapter.py`)**不再做整链 lookup**,改成:

```
head 相机:  T_cam_odom = T_base_odom ⊗ T_armbase_base ⊗ T_cam_armbase
arm  相机:  T_cam_odom = T_base_odom ⊗ T_tcp_base      ⊗ T_cam_tcp
```

| 段 | 来源 | 为什么无歧义 |
|---|---|---|
| `T_base_odom` | tf lookup `odom → base_link` | `base_link` 唯一 parent = `odom` ✓ |
| `T_armbase_base` | tf lookup `base_link → arm1/base_link` | 机械臂基座相对 Go2 躯干,即挂载偏移 ✓ |
| `T_tcp_base` | tf lookup `base_link → arm1/tcp_link` | lookup 的**上界是 base_link**,走链到 base_link 即停,base_link 自己的 parent **不参与判定** ✓ |
| `T_cam_armbase` | `head_calibration.json`(camera→**机械臂基座**) | 纯矩阵 |
| `T_cam_tcp` | `arm_calibration.json`(camera→tcp_link) | 纯矩阵 |

> head 相机多一段 `T_armbase_base`,是因为它的标定基准是机械臂基座而非 Go2 躯干。
> 这一段现在是 identity(`arm_mount_offset` 未实测),量值上等价于旧写法;
> **一旦实测填入偏移,旧写法就会整段偏掉**。

**标定方向:一律不取逆。** 两份标定都是"相机 → 机器人侧",与 tf edge 语义
(parent=P, child=C 表示 `p_P = R·p_C + t`)同构,矩阵复合律
`[cam]_odom = [base]_odom ⊗ [cam]_base` 方向天然对上。
数值验证(随机位姿含非平凡旋转):head/arm 复合误差均 `< 1e-15`;
若错误取逆,误差量级 0.6~5 m。

**两台相机共用 `broadcast_camera_mount_tf`**,数值由标定合成:

```
p_parent = CALIB · p_optical,   p_optical = inv(M1) · p_link
  ⇒  M = CALIB ⊗ inv(M1),       M1 = lookup(link_frame, optical_frame)
```

- **head**:`parent={prefix}base_link`(机械臂基座,不是 Go2 躯干),`link=camera_head_link`,
  `optical=camera_head_color_optical_frame`
- **arm** :`parent=arm1/tcp_link`,`link=camera_arm_link`,
  `optical=camera_arm_color_optical_frame`

桥一律接**相机体 link**,不能接 `..._optical_frame` —— 后者已有 realsense 的 parent。
`M1` 从 realsense 自己的链取(那棵树以 `camera_X_link` 为 root,lookup 无歧义)。

realsense 可能晚于 tracker 启动,所以函数内部用 **0.2 s timer 轮询**等待
`lookup(link, optical)` 成功;**成功即发一次并自停**(内容是常量,不需要周期重发)。

### 4.4 配置项

`lgw_3d_tracker/config/runtime.yaml`:
```yaml
arm_frame_prefix: "arm1/"                        # 必须与 start_lgw_arm.sh 的 frame_prefix 一致
arm_mount_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 机械臂安装偏移 [x,y,z,r,p,y],未实测
```

### 4.5 明确**不改**的东西

| 不改 | 原因 |
|---|---|
| 机械臂 URDF | Plan A 的核心就是不改它 |
| MoveIt 的 `robot_description` / SRDF / kinematics | 模型帧名保持规范(`world`/`base_link`/`tcp_link`) |
| `lgw_bt_planner` 里所有 `tcp_link` 引用 | 那是 **MoveIt 模型帧名**(`ik_link_names` / `link_name`),不是 TF 帧名 |
| BT planner 的 `world_frame` 默认值 | 保持 `"world"` —— MoveIt 用 `Resolve constraint frames to robot links` adapter 对 **robot model** 解析约束帧(非 TF);model 里 `world` 仍在,`world→base_link` 是 identity → 数值等价 |
| 两份标定 JSON 的**数值** | 只改语义引用,数值不变 |
| driver 的 `feedback/tcp_pose` | frame_id 是空字符串,不涉及 TF 帧名 |

---

## 5. 验证

| 项 | 结果 |
|---|---|
| `frame_prefix` 跨 include 传播 | ✓ `demo.launch.py frame_prefix:='arm1/'` → rsp.launch.py 收到 → RSP 发 `arm1/*` |
| namespace 派生路径 | ✓ `namespace:=arm1`(不给 frame_prefix)→ 自动派生出 `arm1/` |
| TF 图单 parent 校验 | ✓ 手工拉全 `/tf` + `/tf_static` 的 edge 列表比(⚠️ **不能靠 `view_frames`**,理由见 §2.3) |
| tracker 两段 lookup | ✓ `odom→base_link` / `base_link→arm1/tcp_link` / `odom→arm1/tcp_link` 全 `OK=25 FAIL=0` |
| 旧无前缀链已消失 | ✓ `base_link→tcp_link` 报 frame 不存在 |
| move_group 启动 | ✓ action server 就绪,无 frame 相关报错 |
| build | ✓ `lgw_3d_tracker` / `lgw_bt_planner` / `agx_arm_moveit` |
| **真机全栈**(真双 realsense + 真 RSP + 真 tracker) | ✓ `/tf_static` 15 条 + `/tf` 1 条,逐帧单 parent |
| **两个独立 late-joiner probe** | ✓ 10/11 lookup OK;唯一 FAIL 是裸名 `tcp_link`(预期,前缀隔离生效) |
| **`/perception/tracks/unified`** | ✓ **9.6–10.0 Hz**,`frame_id=odom`,3 个 track,坐标合理 |
| **三条桥数值与标定一致** | ✓ head `t=(0.311, 0.026, -0.035)` / arm `t=(-0.075, -0.015, -0.086)` |
| **head 桥父帧修正**(2026-10-22 真机复测) | ✓ `camera_head_link` 的 parent = **`arm1/base_link`**;`arm1/base_link→camera_head_link` = `(0.311, 0.026, -0.035)` |
| **分段合成 ≡ 整链 TF** | ✓ `T_base_odom ⊗ T_armbase_base ⊗ T_cam_armbase` 与 `lookup(odom, camera_head_color_optical_frame)` 最大偏差 **5.6e-16** |
| **track 输出未回归** | ✓ `/perception/tracks/unified` **~12 Hz**(170 条 / 14 s) |

---

## 6. 遗留 / 待办

1. **机械臂安装偏移未实测** —— `arm_mount_offset` 是单位阵。不补测则机械臂在 odom 系的
   **绝对**位置有系统偏差(相对位姿正确)。补测后只改 `runtime.yaml` 一个数组。
2. **MoveIt 侧隐含"机械臂安装座 ≡ Go2 base_link"的恒等假设** —— MoveIt 的
   `planning_frame` 必须在 robot model 里存在(默认从 SRDF 的
   `<chain base_link="base_link" tip_link="tcp_link"/>` 推导成 `base_link`),
   不能用 `arm1/base_link`(只存在于 TF)。与第 1 点同源。
3. **`world` 在 TF 里消失**(变成 `arm1/world`)。BT planner 侧靠 model 解析不受影响,
   但任何**直接对 TF 查 `world`** 的外部代码会失败。
4. **两个"世界系"并存**:`world`/`base_link`(机器人躯干系,BT 侧)与 `odom`
   (SLAM 漂移系,tracker 侧)。`UnifiedTrackArray` 目前无下游消费者;等 BT 要消费
   track 时需统一(建议 `base_link`)。
5. **RViz 配置待改**:`agx_arm_moveit/config/moveit.rviz` 的 RobotModel TF Prefix
   应设 `arm1/`、Fixed Frame 设 `base_link`。当前 `start_lgw_arm.sh` 用
   `use_rviz:=false`,暂不影响。
6. ~~**真机端到端回归待做**~~ → ✅ **2026-10-22 已完成**:真机双 realsense
   (`start_lgw_yoloe.sh` 的 `two_realsense.launch.py`)+ 真 RSP + tracker 全栈,
   TF 链全通、`/perception/tracks/unified` 稳定 9.6–10.0 Hz。
   **仍未覆盖**:真实机械臂运动中的 arm 相机位姿、以及双相机 merge 的**数值正确性**
   (目前只验证了链路与频率,未做动臂位姿一致性校验)。
7. ~~**head 标定的 base_link 归属待核对**~~ → ✅ **2026-10-22 已确认并修复**:
   `head_calibration.json` 的 `base_link` 是**机械臂基座**(`{prefix}base_link`),
   不是 Go2 躯干的 `base_link`。已修:head 桥 parent 改为 `{prefix}base_link`,
   消费端合成补 `T_armbase_base` 一段。原写法仅因 `arm_mount_offset` 当前是单位阵
   而数值巧合一致 —— **实测安装偏移后若不修就会整段偏掉**。

---

## 7. 给后续改动的两条规则

> **规则 1:做 TF lookup 时用带前缀的名字**(`arm1/tcp_link`、`arm1/link6` …),
> 裸名 `tcp_link` / `link1` 在 TF 里**不存在**。
>
> **规则 2:和 MoveIt 说话时用不带前缀的模型名**(`tcp_link`、`base_link`)——
> MoveIt 的 robot model 没加前缀,`ik_link_names` / `link_name` 写 `arm1/tcp_link`
> 会找不到。

判断准则:**这个名字是出现在 TF 里,还是出现在 MoveIt 的 robot model 里?**
