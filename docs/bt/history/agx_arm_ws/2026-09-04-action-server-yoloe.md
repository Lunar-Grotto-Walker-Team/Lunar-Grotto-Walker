# Verification Report — Action servers × live YOLOE 集成验证

**日期**: 2026-09-04
**对象**: `agx_arm_perception` 3 个 action server(`/wait_stage1_request`、`/wait_stage2_result`、`/compute_pick_pose`)与 self_l4t 容器内真实运行的 YOLOE 两阶段流程
**目的**: 在 BT 树合成(Task 8)前先验证 server 与 YOLOE 真实交互正常,提前暴露 TF / 接口 / 状态问题
**结论**: **3/3 server `success=true`**,发现并绕过 4 个 TF 链问题(F1–F4)。修复均为环境层 workaround,无源码改动。

---

## 1. 一句话总结

Server 端代码工作正常。问题全部出在**环境层**——DDS 域、URDF 缺失 frame、launch 链不全、以及 realsense 与 calibration TF 形成的 disjoint tree。本次验证让 4 个 latent issue 全部显形,每个都有显式 workaround 与 carry-forward。

---

## 2. 验证结果一览

| Server | 输入 goal | 结果 | 关键数值 |
|---|---|---|---|
| `/wait_stage1_request` | `timeout_sec=8.0` | ✅ success | `candidates[0]="wood_block"` score=0.9687 |
| `/wait_stage2_result` | `timeout_sec=8.0` | ✅ success | `detections[0]="wood_block"` score=0.97 |
| `/compute_pick_pose` | `timeout_sec=10.0, max_retries=3, pre_grasp_offset_z=0.12, lift_offset_z=0.12` | ✅ success | `grasp=(0.485,-0.065,0.171)` base_link,pre_grasp/lift 在 grasp 上方 0.12m |

完整 send_goal 输出 + 中间状态见 `.superpowers/sdd/2026-09-03-yoloe-bt-integration/task-7-report.md`。

---

## 3. 发现的 4 个问题与修复

### F1 — 容器内 server 走默认 DDS,与 host 不通

- **症状**: `ros2 action list` 看不到 server,或 send_goal 一直 hang
- **根因**: 容器内 `ros2 run` 默认走 fastrtps,host Go2 + yoloe_transfer 走 cyclone DDS,两套 DDS 不互通
- **修复**: 启动脚本第一行 `source /my_ws/l4t/l4t_setup.sh`(export `CYCLONEDDS_URI` 等)
- **Carry-forward**: Task 8/9 正式 launch 必须保留这个 source。已固化到 memory `feedback_l4t_unitree_dds.md`

### F2 — ComputePickPose 报 "could not find base_link"

- **症状**:
  ```
  TF lookup failed: "base_link" passed to lookupTransform argument target_frame does not exist
  ```
- **根因**: `tf_static.launch.py`(Task 3 产物)只广播 `tcp_link → camera_arm_color_optical_frame` 一段,机械臂 `base_link → ... → tcp_link` 这一截无人发布
- **修复**: 启动 `agx_arm_description/display.launch.py arm_type:=piper gui:=false`,robot_state_publisher 按 URDF 自动广播 `world → base_link → link1..6`
- **Carry-forward**: Task 8/9 正式 launch 必须把 display.launch.py 纳入

### F3 — URDF 缺 `tcp_link`

- **症状**: 启动 display 后 `base_link..link6` 出现,但 `tcp_link` 不在链上。`tf_static.launch.py` 单独跑时 `tcp_link` 是孤立节点
- **根因**: `piper_description.urdf` 末尾是 `link6`,**没有** `tcp_link`,但 launch 链期望 `link6 → tcp_link → camera_arm_color_optical_frame`。**URDF 缺陷**
- **本次 workaround**: 临时手起 identity static TF
  ```bash
  ros2 run tf2_ros static_transform_publisher \
    --frame-id link6 --child-frame-id tcp_link \
    --translation 0 0 0 --rotation 0 0 0 1
  ```
- **Carry-forward(应升级为新 Task)**:
  - **Option A**(改 URDF): 在 `piper_description.urdf` 末尾追加
    ```xml
    <link name="tcp_link"/>
    <joint name="link6_to_tcp" type="fixed">
      <parent link="link6"/>
      <child link="tcp_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    ```
  - **Option B**(改 xacro): 在 `agx_arm.urdf.xacro` 里 `<xacro:include piper>` 之后追加 link6→tcp_link joint
  - Option A 更彻底,推荐

### F4 — realsense 与 calibration TF 形成 disjoint tree

- **症状**:
  ```
  [ERROR] [tf2_buffer]: Detected two or more unconnected trees ...
  ```
- **根因**: `realsense2_camera_node` 默认根据 camera info 发 `camera_link → camera_optical_frame` + 命名空间变体(`camera_arm_color_frame → camera_arm_color_optical_frame`),只要 base frame 不同就互不连接。`tf_static.launch.py` 又独立发 `tcp_link → camera_arm_color_optical_frame`,两条链在 `camera_arm_color_optical_frame` 汇合前互不连通
- **本次 workaround**: **不**删任何 TF。Buffer 在 `lookupTransform(camera_arm_color_optical_frame → base_link)` 时会优先看机械臂这条链;**禁止反向 lookup**(否则形成回环,某些 lookup 卡死)
- **Carry-forward**: Task 8/9 写 BT 节点 / 下游逻辑时,**只**允许单向 `base_link → camera_arm_color_optical_frame`,不要反向

---

## 4. 最终 TF 链(可工作状态)

```
world
 └─ base_link                              ← display.launch.py (URDF)
     └─ link1 → link2 → link3 → link4 → link5 → link6
                                                  └─ tcp_link       ← identity static TF (F3 workaround)
                                                      └─ camera_arm_color_optical_frame  ← tf_static.launch.py (calibration)

另起 disjoint 树(无害,不要碰):
camera_arm_color_frame
 └─ camera_arm_color_optical_frame         ← realsense2_camera_node (F4)
```

---

## 5. 取舍记录

### 5.1 为什么不在本次 Task 7 固化 F3 修复

F3 根因是 URDF 缺陷,**正确修复**是改 `piper_description.urdf` 或 `agx_arm.urdf.xacro`。但本次任务定义是"验证而非实现",修 URDF 会:
- 影响 `agx_arm_ros` 嵌入式 git repo(不是 agx_arm_ws 的本地 src)
- 需要同步改 display.launch.py / 任何引用 tcp_link 的下游代码
- 范围远超"验证任务"定义

因此选 **workaround + carry-forward**,F3 留待后续独立 Task 修。

### 5.2 为什么 ComputePickPose 输出 base_link 系而不是 tcp_link 系

`compute_pick_pose_server.cpp` 把 `result.header.frame_id`(YOLOE 输出,camera_optical 系)→ `base_link`。这是 spec §6 的设计:**机械臂控制命令在 base_link 系**,downstream MoveArmToPose 节点直接吃这个 pose。

数值 sanity check:
- wood_block 在 camera 系下约 (0, 0, 0.252)
- calibration tcp→camera offset: `[-0.07597, -0.00052, -0.08153]` quat `[-0.122, 0.123, -0.683, 0.709]`
- 反算到 base_link(URDF 零位): grasp ≈ (0.485, -0.065, 0.171) ✓
- pre_grasp / lift 在 grasp 上方 0.12m,符合 `pre_grasp_offset_z=0.12, lift_offset_z=0.12` ✓

### 5.3 为什么不用 `tf_static.launch.py` 集成所有修复

`tf_static.launch.py` 的设计是**只**承载 calibration TF,**不**是 TF 链的"总管"。如果把它扩成 launch 链总管,会:
- 让 calibration 维护者难改 JSON-only 数据
- 让 Task 8/9 整合 launch 时多一层间接
- 违反单一职责

正确做法:Task 8/9 写一个 `tf_full_chain.launch.py`(或同等),**include** `tf_static.launch.py` + `display.launch.py` + link6→tcp_link identity TF。`tf_static.launch.py` 保持只做一件事。

---

## 6. Cleanup / 待办

### 6.1 本次验证遗留进程(需要清理时按此顺序)

| PID | Process | 操作建议 |
|---|---|---|
| 2643, 2645, 2767, 2769, 5287, 5409 | `static_transform_publisher`(F3 workaround 重复启动的多份) | INT → TERM → KILL 阶梯 |
| 5660 | 一次反向 calibration 实验 | 已 TERM |
| (S1/S2/compute server) | 用户决定 | 取决于是否进入 Task 8 |

YOLOE workflow(realsense、mock_camera*_coordinator、two_stage_node、orchestration_launch)**未动**,保持运行。

### 6.2 待升级的独立 Task

- **Task A**: 修 `piper_description.urdf` 或 `agx_arm.urdf.xacro` 添加 link6→tcp_link joint(F3 root fix)
- **Task B**: 把 F1+F2+F3 workaround 整合进 Task 8/9 的正式 launch
- **Task C**: 验证 joint_state_publisher / 真机 driver 是否在 publish joint_states(本次 base_link→tcp_link 链来自 URDF 零位,真机执行前必须确认)

---

## 7. 经验沉淀(下次类似验证时直接复用)

1. **先 source `l4t_setup.sh`** — 见 F1,任何 self_l4t 容器内 ros2 节点都先做
2. **TF 链诊断三件套**: `all_frames_as_dot_ascii` 查 tree 结构,`lookup_transform(target=base_link, source=camera)` 查正向,反向查前必须确认不会成环
3. **`display.launch.py arm_type:=piper gui:=false`** 是补 base_link 的最便宜手段
4. **`pgrep -af '[w]ait_stage'` 在嵌套 docker exec 中可能为空**,改用 `ps -ef | grep -E '[w]ait_stage'`
5. **container 内 `docker exec -d + bash -lc heredoc` 会吞 echo 输出**,改用 Write + `docker cp` + `docker exec -d bash /tmp/script.sh`
6. **YOLOE 两阶段是周期自发**,`/perception/two_stage/observation_request` 与 `/observation_result` 一直在跑,不需要手动喂 — 直接 send_goal 就能等到

---

## 8. 相关文件

- 详细 task brief: `.superpowers/sdd/2026-09-03-yoloe-bt-integration/task-7-brief.md`
- 详细 task report(含完整 send_goal 输出): `.superpowers/sdd/2026-09-03-yoloe-bt-integration/task-7-report.md`
- 设计文档: `docs/superpowers/specs/2026-09-03-yoloe-bt-integration-design.md`
- 实施计划: `docs/superpowers/plans/2026-09-03-yoloe-bt-integration.md`
- SDD ledger: `.superpowers/sdd/2026-09-03-yoloe-bt-integration/progress.md`
