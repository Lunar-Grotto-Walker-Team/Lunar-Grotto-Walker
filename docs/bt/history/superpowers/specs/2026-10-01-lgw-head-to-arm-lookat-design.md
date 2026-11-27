# Head → Arm Camera Lookat BT for lgw_bt_planner

| 字段 | 值 |
|---|---|
| Spec 路径 | `l4t/lg_walker_ws/src/lgw_bt_planner/docs/superpowers/specs/2026-10-01-lgw-head-to-arm-lookat-design.md` |
| 创建日期 | 2026-10-01 |
| 状态 | Draft(待用户审阅) |
| 范围 | lgw_bt_planner:新增 `ComputeLookatPose` BT 节点(含纯函数单元测试)+ 加载 2 份手眼标定 JSON + `head_to_arm_lookat.xml` BT 树 + 配套启动脚本。迭代视觉伺服:head 看到目标 → 用标定算末端位姿使 arm camera 光轴对准 → 移到该位姿 → 检测 arm 是否也看到 → 是则回 home,否则再 refine |
| 不在范围 | 双 tcp 虚拟 link / MoveArmToPose `link_name` 端口(未来 spec);pick-and-place 抓取动作(沿用 pick_and_place_demo.xml);真机以外的 mock 集成测试 |

---

## 1. 背景与动机

现有 `lgw_bt_planner` 已经具备:
- `IsHeadDetectionConfirmed` / `IsArmDetectionConfirmed`(spec 2026-09-24):订阅 YOLOE 双相机 CONF 消息,返回 last_observation
- `MoveArmToPose` / `MoveArmToNamed` / `SetPoseStamped`:MoveIt 动作包装
- `WaitForUserInput`:user msg halt
- `user_gated_auto_pick_and_place.xml`:ReactiveFallback + Repeat num_cycles="-1" 套路(2026-10-01)

但**没有任何节点能从检测结果生成控制指令**。要做"head 发现目标 → arm 相机去看"的视觉伺服回路,需要一个:
1. 读 head 观测 → 用 **head 手眼标定 JSON** 把目标位置转到 base(world)系
2. 计算末端位姿(用 **arm 手眼标定 JSON** 的 arm camera 偏移补偿)使 arm camera 光轴对准目标
3. 驱动 MoveArmToPose 到该位姿
4. 检测 arm 是否看到(给回路终止条件)

`agx_arm_perception::ComputePickPose` 之前做过类似的事,但其 action server 已不可用(只剩 build artifacts 与 piper_bt_planner_old 引用)。

本 spec **不重写 action server**,而是在 BT 节点层做计算:
- **加载 2 份手眼标定 JSON**(用户已在 `my_ws/l4t/result/` 提供,本次 copy 进 lgw ws):
 - `head_calibration.json`:`T_base_link → camera_head_color_optical_frame`(eye_to_hand 模式产物,head 相机在 base_link 下的位置+姿态)
 - `arm_calibration.json`:`T_tcp_link → camera_arm_color_optical_frame`(eye_in_hand 模式产物,**用 `_tcp.json` 变体**,已叠加 `tcp_offset=[0,0,0.12,...]`,把 link6 系转到 tcp_link 系)
- 命名约定:JSON `position` = camera 光学系原点在参考系(base_link / tcp_link)下的位置;JSON `orientation` = 参考系 → camera 光学系的旋转四元数(w,x,y,z)。与 `agx_arm_ws` tf_static.launch.py 已确立约定一致
- **不依赖 TF lookup**(`tf_buffer` 完全用不到),标定 JSON 是真·手眼标定结果
- 复用现有检测节点 + MoveArmToPose
- 新增 SyncActionNode `ComputeLookatPose`,做"head obs + 标定 → tcp_link target pose"
- 新增 BT XML `head_to_arm_lookat.xml`,把所有节点串起来

---

## 2. 节点族

### 2.1 新增 1 个 BT 节点

| 节点 ID | 类型 | 父类 | 用途 |
|---|---|---|---|
| `ComputeLookatPose` | SyncActionNode | `BT::SyncActionNode` | 构造期加载标定 JSON → `onTick` 读 head 观测 + 末端位置参数 → 计算末端 PoseStamped → 写黑板 |

### 2.2 节点接口

#### InputPort

| 端口 | 类型 | 默认值 | 含义 |
|---|---|---|---|
| `head_observation` | `yoloe_interfaces::msg::ObservationResult` | (必填,通常 `{last_head_obs}`) | 头相机最近一帧 CONF 观测 |
| `ee_position_world` | `string` | `"0.2;0.0;0.3"` | arm camera **光学系原点**在 world 系下的目标位置(`x;y;z`,米) |
| `world_frame` | `string` | `"world"` | ee_position 所在的 frame(也是输出 pose 的 frame_id) |
| `head_calibration_path` | `string` | 自动取 `<pkg_share>/calibration/head_calibration.json` | head 手眼标定 JSON 绝对路径;覆盖用于单元/集成测试 |
| `arm_calibration_path` | `string` | 自动取 `<pkg_share>/calibration/arm_calibration.json` | arm 手眼标定 JSON 绝对路径;覆盖用于单元/集成测试 |
| `output_pose_key` | `string` | (空 → 直接用默认 key `"lookat_pose"`) | 黑板 key 覆盖 |

#### OutputPort

无。`ComputeLookatPose` 把结果写到黑板 `lookat_pose`(或 `output_pose_key` 指定的 key),下游 `MoveArmToPose` 用 `{output_pose_key 或 lookat_pose}` 引用。

#### 节点构造期行为(关键)

构造函数(每个节点实例化时执行一次,**只读一次 JSON**):

1. 解析 `head_calibration_path` InputPort(若未显式给 → `ament_index_cpp::get_package_share_directory("lgw_bt_planner") + "/calibration/head_calibration.json"`)
2. 解析 `arm_calibration_path` InputPort(同上逻辑)
3. 读 JSON → 解析 `position[3]` + `orientation[4]`(quat w,x,y,z)
4. 转 `tf2::Vector3` + `tf2::Quaternion` + `tf2::Matrix3x3`,存为成员变量(`head_R_/head_t_` 与 `arm_R_/arm_t_`)
5. **失败处理**:JSON 不存在 / 解析失败 / orientation 模近 0 → 日志 ERROR,标记 `initialized_=false`,后续 `onTick` 永远 FAILURE(节点无效,不抛)

构造期一次读完避免每 tick 重复 IO,BT 树 10Hz tick × Repeat num_cycles="-1" 不该每 tick 读文件。

#### 返回值语义

- `BT::NodeStatus::SUCCESS`:构造期 OK、head_observation 有 detections、lookat 数学成功、PoseStamped 写入黑板
- `BT::NodeStatus::FAILURE`:
 - 构造期失败(标定 JSON 问题)
 - `head_observation.detections` 为空(防御,正常路径下被 `IsHeadDetectionConfirmed` 过滤)
 - Lookat 数学异常:`direction` 范数过小(< 1e-3,目标与 ee_position 重合)、R 矩阵奇异(Gram-Schmidt 退化且 fallback 也退化,极小概率)
- `BT::NodeStatus::RUNNING`:不使用(SyncActionNode 同步决策)

---

## 3. 算法(ComputeLookatPose 6 步)

**记号**:
- `P_target_head` = `head_observation.detections[0].center_m`(Point,head_optical 系下)
- `P_ee_target_world` = 用户指定,arm camera 光原点应达到的 world 系位置,默认 `(0.2, 0, 0.3)`
- `T_head_to_base` = `(R_h2b, t_h2b)`(标定 JSON 解出,head 光学系→base_link 的旋转/平移)
- `T_arm_in_tcp` = `(R_a2t, t_a2t)`(标定 JSON 解出,arm 光学系在 tcp_link 系下的旋转/平移)

**Step 1 — Head 观测转到 base**:
```
P_target_base = R_h2b · P_target_head + t_h2b
```

**Step 2 — 方向向量**:
```
direction_base = (P_target_base - P_ee_target_world).normalize()
```
若 `||P_target_base - P_ee_target_world|| < epsilon` → FAILURE(目标与 ee_position 重合,几何退化)。

**Step 3 — 构造 arm_optical 在 base 下的旋转矩阵**:

约定:相机光学系 `z` 前向、`y` 下、`x` 右。
```
world_up = (0, 0, 1)
raw_x = direction_base × world_up
if (||raw_x|| < epsilon)            # direction ≈ ±world_up,Gram-Schmidt 退化
    raw_x = direction_base × (1, 0, 0)   # fallback
x_axis_base = raw_x.normalize()
y_axis_base = direction_base × x_axis_base
R_arm_in_base = Matrix3x3(x_axis_base, y_axis_base, direction_base)   # 3x3,columns
```

**Step 4 — 计算 tcp_link 在 base 下的旋转**:
```
R_tcp_in_base = R_arm_in_base · inverse(R_a2t)
```
其中 `R_a2t` 来自 arm 标定 JSON `orientation` 转 `tf2::Quaternion` 再转 Matrix3x3。

**Step 5 — 计算 tcp_link 在 base 下的位置**:

目标:arm optical 原点在 world 下 = `P_ee_target_world`,即 `R_tcp_in_base · t_a2t + P_tcp_in_base = P_ee_target_world`
```
P_tcp_target_world = P_ee_target_world - R_tcp_in_base · t_a2t
```

**Step 6 — 输出 PoseStamped**:
```
pose_stamped.header.frame_id = world_frame
pose_stamped.pose.position = P_tcp_target_world
pose_stamped.pose.orientation = quaternion_from_rotation_matrix(R_tcp_in_base)
blackboard->set("lookat_pose" or output_pose_key, pose_stamped)
return SUCCESS
```

---

## 4. 关键设计决策

### 4.1 使用手眼标定 JSON(本次核心)

**不**用 TF lookup / `tf_buffer`,直接加载用户提供的两份标定 JSON。

| 文件 | 数学约定 | 提供方式 |
|---|---|---|
| `head_calibration.json` | `T_base_link → camera_head_color_optical_frame`(eye_to_hand 产物,head 相机在 base_link 下的 position+orientation) | 用户提供 `my_ws/l4t/result/2026-09-30_12-54-33_calibration.json` |
| `arm_calibration.json` | `T_tcp_link → camera_arm_color_optical_frame`(eye_in_hand `_tcp.json` 变体产物,arm 相机在 tcp_link 下的 position+orientation) | 用户提供 `my_ws/l4t/result/2026-09-02_08-34-16_calibration_tcp.json`(已叠加 `tcp_offset=[0,0,0.12,...]`) |

**为什么 arm 必须用 `_tcp.json` 变体**:
- 原版 `2026-09-02_08-34-16_calibration.json` 来自 `cv2.calibrateHandEye` + piper `/piper_ctrl_node/end_pose` = link6 位姿,JSON 语义是 `T_link6 → camera_arm_color_optical_frame`
- 节点计算 lookat pose 用的是 `tcp_link` 在 world 下的位姿(`MoveArmToPose` 接 PoseStamped 默认走 `tcp_link`),需要的是 **tcp 系** 下的 arm camera 偏移
- `_tcp.json` 是预先把 `tcp_offset=[0,0,0.12,0,0,0]` 叠加上去的产物(`_meta.tcp_offset_applied` 字段明示),数值上 `pos.z` 从 `0.038` → `-0.082`(差 `-0.12`),与 `agx_arm_ws` tf_static.launch.py 已用约定一致
- **不要**用原版 link6 系 JSON 再让节点内补偿 tcp_offset(增加节点复杂度且无收益)

**JSON 格式**(来自源文件实测):
```json
{
  "position": [x, y, z],          // 米,camera 光学系原点在参考系下的位置
  "orientation": [w, x, y, z],    // 四元数,w-first ROS 约定,参考系 → camera 光学系
  "rpy": [[roll, pitch, yaw]]     // 仅参考,不参与计算
}
```

**位置**:`<src/lgw_bt_planner>/calibration/{head,arm}_calibration.json`,通过 CMakeLists `install(DIRECTORY calibration/ DESTINATION share/${PROJECT_NAME}/calibration)` 部署到 `install/lgw_bt_planner/share/lgw_bt_planner/calibration/`。节点构造期用 `ament_index_cpp::get_package_share_directory` 解析。

**数值实例**(供实施阶段 sanity check,不硬编码到 C++):
- head_calibration: `pos=[0.309, 0.029, -0.010]`, `quat=(0.417, -0.570, 0.579, -0.409)` —— ~30cm 偏移 = 头部相机装在机身前部
- arm_calibration(`_tcp.json`): `pos=[-0.076, -0.0005, -0.082]`, `quat=(0.709, -0.122, 0.123, -0.683)` —— ~7cm xy 偏移 + ~8cm z 偏移(相机在 tcp_link 下方 8cm)

**优势**(对比"近似"路径):
- 位置误差从 ±2cm → ±几 mm(标定残差)
- 姿态误差从 ±5° → ±几 mrad(标定残差)
- 视觉伺服首轮误差 < 5mm / 1°,YOLOE arm 相机直接命中,无需多轮 refine

**劣势**:
- 标定 JSON 必须存在且格式正确,节点构造期失败 → 节点整个生命周期 FAILURE(不影响 BT 树其他部分,但本次任务该子树不可用)
- 重新标定需替换文件 + 重启 bt_runner

### 4.2 不做运行时 tcp 偏置切换(节点零补偿)

详见 brainstorming §Q3。本次**未触及 xacro / MoveArmToPose**,**节点内不需要任何 tcp_offset 补偿逻辑** —— `arm_calibration.json` 直接来自 `_tcp.json` 变体,已经是 `T_tcp_link → camera_arm_color_optical_frame`,节点拿到的 `(R_a2t, t_a2t)` 直接就是 tcp 系下的偏移。算法 Step 4-5 不需要二次换算。

**对比"节点内补偿"方案**:如果用原版 `link6` 系 JSON,节点需要从 launch 读 `tcp_offset` 参数(默认 `[0,0,0.12,0,0,0]`)、构造期把 `R_a2t, t_a2t` 从 link6 系转到 tcp 系,徒增代码与测试负担,且 launch 参数变化(换 gripper / 改 tcp_offset)会让节点失效。`_tcp.json` 把这一层固化进文件,既沿用 `agx_arm_ws` tf_static 已有约定,也让节点保持纯算法。

**未来升级路径**(本次 spec 范围外):`tcp_link_align` 虚拟 link + MoveArmToPose `link_name` InputPort → 双 tcp 通用化。ComputeLookatPose 接口不变。

### 4.3 frame 选 `world` 而非 `base_link`

URDF `world → base_link` 是 identity fixed joint(无变换)。本次选 `world` 跟现有 `pick_and_place_demo.xml:28-29` 注释一致,保持 XML 字面值与现有 demo 风格一致。

`arm_calibration.json` 数据语义是"arm optical 在 tcp_link 系下",与 world/base_link 无关。`head_calibration.json` 语义是"head optical 在 base_link 下",`base_link → world = identity`,所以无需换算。

若 launch 改用 `base_link`,改 `world_frame` InputPort 默认值即可(无需改 C++)。

### 4.4 不需要 `head_camera_frame` / `arm_camera_frame` InputPort

算法只用标定 JSON 的 (position, orientation),不读 `head_observation.header.frame_id` 也不查 TF。简 YAG。

### 4.5 单元测试覆盖纯函数(本次)

详见 §8.1。

---

## 5. 错误处理

| 场景 | 节点行为 | BT 行为 |
|---|---|---|
| 标定 JSON 不存在 / 解析失败 / 字段缺失 | 构造期日志 ERROR,`initialized_=false`,`onTick` 永远 FAILURE | 内 ReactiveFallback child 2 Sequence FAILURE → inner RF SUCCESS → outer Repeat 下一轮 |
| 标定 orientation 不是合法四元数(模 ≠ 1) | 构造期 ERROR 同上 | 同上 |
| `head_observation.detections` 空 | `onTick` FAILURE | 同上 |
| `direction_base` 范数过小(< 1e-3) | `onTick` FAILURE | 同上 |
| Gram-Schmidt 退化(direction ≈ ±world_up) | `onTick` 用 world_x fallback 替代 world_up,正常 SUCCESS | — |
| 黑板 `head_observation` 为空(前一步 SUCCESS 后黑板被清) | `getInput` 抛 `BT::RuntimeError`,FAILURE | 同上 |
| `output_pose_key` 已存在于黑板 | 覆盖写 | — |
| BT halt 中 | BT.ROS2 标准行为(SyncActionNode 无订阅) | — |

---

## 6. XML 用法

### 6.1 新 BT 树 `head_to_arm_lookat.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--
  head_to_arm_lookat.xml — head 看到目标 → 计算 lookat pose → arm 移到 → 检测 arm 是否也看到

  结构:
    ReactiveFallback root
      child 1: WaitForUserInput       (user msg → SUCCESS,整树终止)
      child 2: Repeat num_cycles="-1"
          child: Sequence "refine_then_home"
            step 1: ReactiveFallback "arm_seen_or_refine"
              child 1: IsArmDetectionConfirmed    (arm 看到 → 跳过 refine)
              child 2: Sequence "refine_step"
                IsHeadDetectionConfirmed          (head 必须看到 → 拿到观测)
                ComputeLookatPose                 (head obs + 标定 JSON → tcp_link target pose)
                MoveArmToPose                    (MoveIt 到 tcp_link target)
            step 2: MoveArmToNamed "home"        (每次循环回到 home)

  行为:
    - 无 user msg → Repeat 无限循环
    - 每轮 Sequence { RF(arm_seen_or_refine) → home }
    - arm_seen_or_refine:
        * arm 已 CONFIRMED → SUCCESS → 直接 home
        * arm 未 CONFIRMED → 走 refine_step(头看到就算 → lookat → move)
    - refine_step 任一步 FAILURE → 内 Sequence FAILURE → inner RF SUCCESS(继续) → home → 下一轮
    - user msg → WaitForUserInput SUCCESS → 整 RF SUCCESS → 终止
-->
<root BTCPP_format="4" main_tree_to_execute="HeadToArmLookat">
  <BehaviorTree ID="HeadToArmLookat">
    <ReactiveFallback name="root">
      <WaitForUserInput name="wait_user_input" topic_name="/bt_user_input"/>

      <Repeat name="auto_loop" num_cycles="-1">
        <Sequence name="refine_then_home">
          <ReactiveFallback name="arm_seen_or_refine">
            <IsArmDetectionConfirmed name="check_arm"
              topic_name="/perception/two_stage/camera_arm/result"
              max_age_sec="0.3"
              last_observation="{last_arm_obs}"/>

            <Sequence name="refine_step">
              <IsHeadDetectionConfirmed name="check_head"
                topic_name="/perception/two_stage/camera_head/result"
                max_age_sec="0.3"
                last_observation="{last_head_obs}"/>

              <ComputeLookatPose name="compute_lookat"
                head_observation="{last_head_obs}"
                ee_position_world="0.2;0.0;0.3"
                world_frame="world"
                output_pose_key="lookat_pose"/>

              <MoveArmToPose name="move_to_lookat" goal="{lookat_pose}"/>
            </Sequence>
          </ReactiveFallback>

          <MoveArmToNamed name="return_home" named_target="home"
            named_targets="{@named_targets}"/>
        </Sequence>
      </Repeat>
    </ReactiveFallback>
  </BehaviorTree>
</root>
```

---

## 7. 代码改动清单

| 文件 | 操作 | 说明 |
|---|---|---|
| `include/lgw_bt_planner/compute_lookat_pose_node.hpp` | **新增** | `ComputeLookatPoseNode` 类声明(SyncActionNode)+) 纯函数 `build_lookat_orientation` + `compute_tcp_pose` 公开声明(供单元测试) |
| `src/compute_lookat_pose_node.cpp` | **新增** | 标定 JSON 加载器 + 算法 6 步 + quaternion↔R 矩阵辅助 |
| `src/bt_plugin_registration.cpp` | **修改** | 加 `factory.registerNodeType<lgw_bt_planner::ComputeLookatPoseNode>("ComputeLookatPose")` |
| `CMakeLists.txt` | **修改** | 加 `src/compute_lookat_pose_node.cpp` 到 `NODE_SOURCES`;加 `install(DIRECTORY calibration/ DESTINATION share/${PROJECT_NAME}/calibration)` |
| `tests/test_compute_lookat_pose.cpp` | **新增** | 单元测试,覆盖纯函数 `build_lookat_orientation` + `compute_tcp_pose` + JSON 解析器 |
| `calibration/head_calibration.json` | **新增** | 从 `my_ws/l4t/result/2026-09-30_12-54-33_calibration.json` 复制,内容不变(`T_base_link → camera_head_color_optical_frame`) |
| `calibration/arm_calibration.json` | **新增** | 从 `my_ws/l4t/result/2026-09-02_08-34-16_calibration_tcp.json` 复制,**必须用 `_tcp.json` 变体**(已叠加 tcp_offset,`T_tcp_link → camera_arm_color_optical_frame`) |
| `bt_trees/head_to_arm_lookat.xml` | **新增** | §6.1 XML |
| `launch/lgw_bt_planner.launch.py` | **修改** | `behavior_trees` 参数加 `head_to_arm_lookat.xml` 入口(让 TreeID 暴露给 send_goal) |
| `scripts/start_lgw_head_to_arm_lookat.sh` | **新增** | 复刻 `start_lgw_user_gated_auto_loop.sh`,只换 BT_XML 默认;同样 `LGW_INSTALL` 注入、`PID` 阶梯清理 |

### 不修改的文件

- `bt_runner_node.cpp`(`tf_buffer` 仍注入全局黑板,但本次 spec 不用)
- `move_arm_to_pose_action_node.cpp` / `move_arm_to_named_action_node.cpp` / `set_pose_stamped_node.cpp` / `wait_for_user_input_bt_node.cpp` / `is_detection_confirmed_node.cpp` / `evaluate_detection.cpp` / `pose_string_parser.cpp` / `named_target_loader.cpp`
- 现有 5 棵 demo XML
- xacro / URDF / SRDF

### 新增包依赖

无需新依赖。`tf2`、`tf2_geometry_msgs` 已在 CMakeLists 现有 `find_package` / `ament_target_dependencies` 中(`compute_tcp_pose` 用 `tf2::Matrix3x3` / `tf2::Quaternion`)。

### 单元测试 CMakeLists 修改

仿照现有 `test_evaluate_detection`(line 138-148),加:
```cmake
ament_add_gtest(test_compute_lookat_pose tests/test_compute_lookat_pose.cpp)
target_link_libraries(test_compute_lookat_pose lgw_bt_planner_nodes)
ament_target_dependencies(test_compute_lookat_pose
  rclcpp tf2 tf2_geometry_msgs geometry_msgs yoloe_interfaces)
```

---

## 8. 测试策略

### 8.1 单元测试(本次必做)

`tests/test_compute_lookat_pose.cpp` 覆盖以下纯函数与 JSON 解析:

**`build_lookat_orientation(Vector3 direction, Vector3 world_up)`**:
- **TC1**:direction=(0,0,-1), world_up=(0,0,1) → x_axis≈(-1,0,0), y_axis≈(0,1,0), R 列向量验证
- **TC2**:direction=(1,0,0), world_up=(0,0,1) → x_axis≈(0,-1,0), y_axis≈(0,0,-1) (Gram-Schmidt 正常)
- **TC3**:direction=(0,0,-1), world_up=(0,0,1) → 退化,函数 fallback 用 world_x,验证 x_axis≠0
- **TC4**:direction=(1,0,0) 且其范数为 0 → 抛异常或返回 FAILURE(防除零)

**`compute_tcp_pose(P_ee_target, R_arm_in_base, R_arm_in_tcp, t_arm_in_tcp)`**:
- **TC5**:R_arm_in_base=I, R_arm_in_tcp=I, t_arm_in_tcp=(0,0,-0.12), P_ee_target=(0.2,0,0.3) → P_tcp=(0.2,0,0.42), R_tcp=I (零偏移校)
- **TC6**:R_arm_in_base=绕 z 旋转 90°, R_arm_in_tcp=I, t_arm_in_tcp=(0,0.1,0), P_ee_target=(0.5,0,0.3) → 手算 tcp_pose 验证 R 与 P

**`parse_calibration_json(path)`**:
- **TC7**:合法 JSON(从 fixture inline 字符串构造临时文件) → position/quat 正确解出,模验证 quat 合法
- **TC8**:JSON 缺 `orientation` 字段 → 抛异常
- **TC9**:orientation 模 = 0 → 抛异常
- **TC10**:文件不存在 → 抛异常

**集成冒烟**(不依赖 ROS runtime):
- **TC11**:喂 head_observation fixture + head/arm calibration fixture → 跑完整 6 步算法 → 输出 PoseStamped 数值 sanity check(P_ee_target=(0.2,0,0.3) ± 几 mm,P_tcp 在合理范围)

测试方式:独立 `.cpp`,**不依赖 ROS runtime**(标定 JSON 解析用 `nlohmann/json` 或自写最小解析器,见 §8.3)。跑 build/ 下 gtest 二进制(`ament_cmake_test` 本容器缺,直接执行)。

### 8.2 集成测试(本次必做,真机)

**冒烟测试**(真机环境,**不**使用 mock):

1. 真机:Go2 driver / 机械臂硬件 / head+arm realsense / YOLOE workflow 全部 up
2. 启动 `start_lgw_head_to_arm_lookat.sh` → bt_runner + MoveIt up
3. `ros2 action send_goal /lgw_bt_execution ... "{target_tree: HeadToArmLookat}"` → 树 RUNNING(等 WaitForUserInput FAIL)
4. 真实目标(实物 / print marker)放在机械臂前 ~30cm:
 - 启动 YOLOE detection → `/perception/two_stage/camera_head/result` 自动周期发 CONF + detections
5. Groot2 + 节点 log 观察 BT 状态机:
 - IsHeadDetectionConfirmed SUCCESS → ComputeLookatPose SUCCESS → MoveArmToPose RUNNING → 机械臂按目标位姿移动
 - 机械臂到位 → arm 相机视野出现目标 → IsArmDetectionConfirmed SUCCESS → 下轮 Sequence 跳过 refine → MoveArmToNamed home → 下一轮 idle
6. **halt 路径**:`ros2 topic pub --once /bt_user_input std_msgs/Bool "data: true"` → WaitForUserInput SUCCESS → 树终止
7. **FAILURE 路径**:用布盖住 head 相机 → IsHeadDetectionConfirmed FAILURE → Sequence FAILURE → Repeat 下一轮重试(不应崩)
8. **compute sanity check**:从 launch log 抓 `ComputeLookatPose` debug 输出(类似 `MoveArmToPose` 的 `BT_PLANNER_DEBUG=1` 机制),核对 P_tcp_target 与 P_ee_target 几何关系正确

**回归测试**:
- 现有 5 棵 demo XML(`pose_via_set_pose_stamped.xml` / `go_to_target_and_back.xml` / `pick_and_place_demo.xml` / `detection_condition_test.xml` / `user_gated_auto_pick_and_place.xml`)继续能跑

### 8.3 JSON 解析依赖选择

实现 `parse_calibration_json(path)`:
- **方案 A**:`nlohmann::json`(`#include <nlohmann/json.hpp>`,apt 包 `nlohmann-json3-dev`)。最干净。
- **方案 B**:手写只读 `position` / `orientation` 字段的最小解析器(~30 行),无新依赖。

本次倾向 **方案 A**(标准做法,依赖少,字段加多时不需改解析器)。如 apt 不可用,fallback B。实施阶段先 `docker exec self_l4t bash -c 'dpkg -l | grep nlohmann'` 验证可用性。

---

## 9. 已知约束与注意事项

1. **`head_observation` 必须来自前一步 SUCCESS 的 `IsHeadDetectionConfirmed`**(ReactiveFallback + Sequence 保证顺序执行)。XML 用 `{last_head_obs}` 黑板引用是 BT 标准模式。
2. **arm calibration JSON(`_tcp.json` 变体)z=−0.082**:意味着 arm camera 在 tcp_link **下方** 8.2cm。原版(非变体)`z=+0.038` 是 link6 系下的偏移,tcp_offset=[0,0,0.12,...] 已叠加进 `_tcp.json`(差正好 −0.12)。**不要混淆两份 JSON**:`head_calibration.json` = `T_base_link → camera_head_color_optical_frame`;`arm_calibration.json` = `T_tcp_link → camera_arm_color_optical_frame`(不是 link6,也不是 base_link)。
3. **head calibration 语义**:head camera 在 base_link 下 ~30cm。**不依赖** tf_static.launch.py 是否启动 — 标定 JSON 是离线手眼标定的结果,TF 是否连接不影响本次计算。
4. **节点零 tcp_offset 补偿**(对应 §4.2):`arm_calibration.json` = `_tcp.json` 变体,**(R_a2t, t_a2t)直接是 tcp 系下偏移**,算法 Step 4-5 不做 link6→tcp 换算。launch 改 `tcp_offset` 或换 gripper 必须**重新生成 `_tcp.json` 并重启 bt_runner**,否则 lookat pose 会系统性偏掉。
5. **max_age_sec = 0.3s**(基于 2026-10-01 视觉伺服讨论结论):YOLOE TensorRT 5Hz 下,head/arm 检测 0.3s 内有 1.5 帧,足够新一次循环读到。
6. **`Repeat num_cycles="-1"` 无限循环**:必须 `WaitForUserInput` 终止,**不要 Ctrl-C 直接 kill bt_runner**(CLAUDE.md 禁忌)。
7. **构造期 vs onTick 加载 JSON**:本 spec 节点构造期一次加载,避免每 tick 重复 IO。ReactiveFallback 在 Repeat 切换时**不会重建子节点**,所以一份 JSON 一辈子用。如未来要支持热替换,改成带版本号的 mutex 保护 load-or-reuse。
8. **真机测试时物理安全**:参见 `user_physical_robot_safety` memory。机械臂移动到 lookat pose 需确保目标周围无障碍,操作员手不要伸入机械臂工作空间。`WaitForUserInput` 是显式的紧急停止手段。

---

## 10. 范围外(未来 spec)

| 任务 | 范围 |
|---|---|
| 双虚拟 link (`tcp_link_align`) + MoveArmToPose `link_name` 端口 | 未来 spec,本次不动 xacro / MoveArmToPose |
| 手眼标定矩阵严格化(多目标点 re-calibration 工具) | 未来 spec,本次沿用现有标定 JSON |
| YOLOE arm 检测不到时回退策略(改 head-only 决策 / 多目标选择) | 未来 spec |
| 视觉伺服终止条件改为"arm 中心化误差 < 阈值" | 未来 spec |
| 整机 mission 树(head → lookat → approach → grasp → place) | 后续 spec |
| 标定 JSON 运行时热加载 | 未来 spec |

---

## 11. 验收标准

### 单元测试
- [ ] `colcon build --packages-select lgw_bt_planner --cmake-args -DBUILD_TESTING=ON` 无 warning 通过
- [ ] `build/lgw_bt_planner/test_compute_lookat_pose` 跑通,11 个 TC 全 PASS

### 构建与集成
- [ ] `colcon build --packages-select lgw_bt_planner` 无 warning 通过
- [ ] `nm -D liblgw_bt_planner_nodes.so` 看到 `ComputeLookatPose` 注册符号
- [ ] `ls install/lgw_bt_planner/share/lgw_bt_planner/calibration/{head,arm}_calibration.json` 存在
- [ ] `ls install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/head_to_arm_lookat.xml` 存在
- [ ] bt_runner log `Loaded BehaviorTree: head_to_arm_lookat.xml` 出现

### 真机集成测试
- [ ] 启动 `start_lgw_head_to_arm_lookat.sh` → 启动无错
- [ ] send_goal HeadToArmLookat → 树 RUNNING
- [ ] 真实目标(head 视野内)→ ComputeLookatPose SUCCESS,MoveArmToPose 收到合理 PoseStamped,机械臂按位姿移动
- [ ] arm 相机视野出现目标 → IsArmDetectionConfirmed SUCCESS → 后续 refine 跳过
- [ ] user msg → WaitForUserInput SUCCESS → 树终止
- [ ] head 相机被遮挡 → IsHeadDetectionConfirmed FAILURE → 树不崩,Repeat 下一轮重试
- [ ] 现有 5 棵 demo XML 继续能跑(回归)

### 清理与安全
- [ ] cleanup 信号 trap 正确工作(TERM → KILL 阶梯,不 `pkill`)
- [ ] 真机测试期间操作员不离开控制台,周围 1.0m 范围无障碍