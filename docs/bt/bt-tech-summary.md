# lgw_bt_planner — BT 决策框架技术总结

> 范围:`l4t/lg_walker_ws/src/lgw_bt_planner/`
> 文档定位:介绍本项目所用的自主决策规划技术(BehaviorTree.CPP + BT.ROS2),
> 并梳理"当前实现 → 月球洞穴 walker 完整任务"的差距与推荐扩展节点。
> 配套图:`bt-decision-framework.drawio.xml`(同目录)。

---

## 1. 技术栈

| 层 | 组件 | 角色 |
|---|---|---|
| 核心库 | BehaviorTree.CPP v4 (`behaviortree_cpp`) | 树形状态机:`Sequence` / `Fallback` / `Decorator` 等控制节点 + 异步 Action 节点 |
| ROS2 桥 | BT.ROS2 (`behaviortree_ros2`) | 把 BT 节点挂到 ROS2 节点 / Action / Service / Topic;提供 `TreeExecutionServer` 基类 |
| 主机节点 | `bt_runner` (本包) | `TreeExecutionServer` 的薄子类,启动时一次性加载 SRDF 命名姿态表,每次建树时把 `shared_ptr<TargetTable>` 和 `shared_ptr<tf2::Buffer>` 写入**全局黑板** |
| 编辑器 | Groot2 | BT.ROS2 自动 attach 一个 ZMQ publisher,实时观察树状态、当前 tick 节点、端口值 |

关键设计点:
- **节点注册走 .so plugin**:三个 leaf 节点编译成 `lgw_bt_planner_nodes` SHARED 库,装到 `share/<pkg>/bt_plugins/`;BT.ROS2 启动时按 `plugins` ROS 参数 glob `*.so` 并 `dlopen`,符号 `BT_RegisterRosNodeFromPlugin` 把节点注册进 Factory。`bt_runner` **不直接 link 这个 .so**(避免与 dlopen 路径冲突)。
- **全局黑板 + `@{key}` 端口重映射**:每个 BT 实例的根黑板都把全局黑板当作 parent;XML 写 `named_targets="@{named_targets}"` 表示这个端口值要从全局黑板取。类型匹配走 `typeid(T)` 精确匹配,所以 `shared_ptr<const Foo>` 要 cast 成 `shared_ptr<Foo>` 才能 set/get。
- **Action 包装而非 ROS callback 拼装**:`RosActionNode<MoveGroup>` 把 moveit 的 action 客户端封装成 BT 异步节点 —— BT tick 时发 goal,BT halt 时 cancel goal,result 回调自动转 `SUCCESS` / `FAILURE`。本包 BT 节点**绝不在自己的 `tick()` 里写 while 循环或 `rclcpp::Rate`**。

---

## 2. 四层架构(决策框架图的核心)

```
┌─────────────────────────────────────────────────────────────────┐
│ L1  BT 引擎  ───  常驻进程                                       │
│      BT.CPP v4  +  TreeExecutionServer  +  Groot2 publisher     │
│      tick_frequency=10Hz,  action_name=/<pkg>_bt_execution       │
└─────────────────────────────────────────────────────────────────┘
                            │  按 behavior_trees ROS 参数加载
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ L2  BT 树 (XML)  ───  行为编排                                    │
│      当前: 3 个 Sequence(纯顺序)                                  │
│      规划: 探索 / 采样 / 折返 / 卸货 任务树                        │
└─────────────────────────────────────────────────────────────────┘
                            │  按 plugins ROS 参数 dlopen
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ L3  BT 节点 plugin (.so)  ───  执行原语                          │
│      当前: 3 个 leaf 节点                                         │
│      规划: 导航 / 感知 / 折返 / 卸货 节点族                        │
└─────────────────────────────────────────────────────────────────┘
                            │  调用
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ L4  下层动作服务器 / 话题  ───  与 MoveIt2 / Nav2 / 感知 / 硬件  │
│      当前: /move_action (MoveGroup), /feedback/joint_states     │
│      规划: /navigate_to_pose, /detected_targets, /map, /tf ...  │
└─────────────────────────────────────────────────────────────────┘
```

横切:**全局黑板**贯穿 L2 / L3。`bt_runner` 在 `onTreeCreated()` 时把 SRDF 表、`tf_buffer` 等不可变共享指针注入,所有 leaf 节点通过 `@{key}` 端口读取 —— 这避免了每个节点都重新解析 SRDF,以及让 tf 在跨节点查询时复用同一个 buffer。

---

## 3. 当前 BT 节点语义(抽象,不写类名)

| 节点 | 类型 | 作用 | 输入 | 输出 |
|---|---|---|---|---|
| **Pose 字面量生成器** | SyncAction | 把 `frame;x;y;z;qx;qy;qz;qw` 字符串解析成 typed `PoseStamped`,写黑板 | `value`(字符串字面量)、`output_key`(黑板键名) | 黑板上命名键下的 `PoseStamped` |
| **绝对姿态规划器** | RosActionNode\<MoveGroup\> | 把黑板上的 `PoseStamped` 转成 MoveGroup goal(position box 1e-5 m + orientation tol 1e-3 rad),送到 `/move_action` | `goal`(`PoseStamped`)、可选 `planner_id` / `planning_time` / `plan_only` | `SUCCESS` / `FAILURE` |
| **SRDF 命名姿态规划器** | RosActionNode\<MoveGroup\> | 把 SRDF `group_state` 名查表后构造 joint constraint goal,送 `/move_action` | `named_target`(字符串,如 `home` / `gripper_open` / `gripper_close`)、`named_targets`(全局黑板 SRDF 表) | `SUCCESS` / `FAILURE` |

---

## 4. 当前 3 个 BT XML 树

### 4.1 `pose_via_set_pose_stamped.xml`(最小烟雾测试)

`Sequence` 2 步:
1. Pose 字面量生成器 → 把硬编码的 observation pose(2026-09-04 记录)写黑板
2. 绝对姿态规划器 → 走到该 pose
3. SRDF 命名姿态规划器(`home`) → 回 home

用途:验证"字面量 → typed pose → move_group"链路是否通,作为其他树的构建块。

### 4.2 `go_to_target_and_back.xml`

`Sequence` 2 步:
1. SRDF 命名姿态规划器(`raised`)→ 抬起到预设姿态
2. SRDF 命名姿态规划器(`home`)→ 落回 home

用途:验证"双 named_target 切换 + IK 收敛稳定性"。

### 4.3 `pick_and_place_demo.xml`(当前主 demo)

`Sequence` 7 步:

| # | 节点 | 目的 |
|---|---|---|
| 1 | SRDF 命名姿态规划器(`gripper_open`) | 张开夹爪(走 move_group → gripper_controller 路径,避开自定义 20Hz /control/joint_states 与 joint_state_broadcaster 200Hz 同 topic 多源冲突) |
| 2 | Pose 字面量生成器 + 绝对姿态规划器 | 移到目标抓取位姿(2026-09-04 记录) |
| 3 | SRDF 命名姿态规划器(`gripper_close`) | 闭合夹爪(靠 gripper 自身 stall 力抓取) |
| 4 | SRDF 命名姿态规划器(`home`) | 回 home |
| 5 | Pose 字面量生成器 + 绝对姿态规划器 | 移到放置位姿 |
| 6 | SRDF 命名姿态规划器(`gripper_open`) | 张开夹爪(释放) |
| 7 | SRDF 命名姿态规划器(`home`) | 回到 home |

注:步骤 0 和 4.5 的 `WaitForUserInput` 节点在 2026-09-08 race fix 阶段临时禁用,目前完全自动跑。

---

## 5. 当前栈与"月球洞穴 walker 完整任务"的差距

### 5.1 任务分解(目标架构)

| 阶段 | 描述 | 当前栈覆盖? |
|---|---|---|
| **导航探索** | 在未知洞穴里走到目标点附近(可能用 frontier exploration 或直接给定目标) | ❌ 无 Nav2 节点 |
| **目标识别** | 通过 RGB / 点云 / 深度检测目标(岩石 / 矿物 / 标记物) | ❌ 无感知节点 |
| **接近** | 把机器人底盘精确导航到"机械臂可达"的位姿 | ❌ |
| **采样** | 用当前包已实现的 7 步 pick-and-place,把目标抓到夹爪里 | ✅(机械臂部分已通) |
| **原路折返** | 把探索时记录的 waypoint list 反向回放,回起点 / dock | ❌ |
| **卸货** | 把夹爪里的样本放进 dock 上的容器(同样是 pick-and-place 的 place 步骤) | ✅(复用) |

**结论**:机械臂部分(采样 + 卸货)已就绪;**底盘导航 / 感知 / 折返路径记录**这三大块完全缺失。

### 5.2 推荐扩展节点族(保持抽象层级)

按 L3 plugin 层加,不动 L1 / L2 框架:

| 节点 | 语义 | 对应的下层接口(L4) |
|---|---|---|
| **导航目标点发送器** | 把 `PoseStamped` 转 `/navigate_to_pose` action goal,SUCCESS 时认为到达 | Nav2 stack |
| **探索触发器** | 启动 frontier exploration 直到满足停止条件(覆盖 / 步数 / 目标识别触发) | Nav2 + frontier_exploration |
| **目标识别监听器** | 订阅 `/detected_targets`,持续 spin 直到发现指定语义类别 | YOLO / segmentation 节点输出 |
| **接近位姿计算器** | 给定检测到的目标 3D 坐标,结合可达性分析算出"机械臂可达"的底盘位姿 | MoveIt2 reachable workspace 查询 |
| **路径记录器** | 周期采样当前 `base_link` 在 `map` 下的位姿,写入 waypoint list | tf2 |
| **路径回放器** | 把 waypoint list 反向,逐点送"导航目标点发送器" | 同上 |
| **任务完成判定器** | 综合"已卸载 + 已回到 dock + 通讯心跳"判定 mission SUCCESS,挂到根 `Sequence` 末尾 | 自定义 |

### 5.3 推荐的 L2 树族(每棵一个子任务)

- `explore_until_target.xml`:`Sequence` { 探索触发器 → [目标识别监听器 (Fallback)] → SUCCESS }
- `sample_at_target.xml`:**复用现有 `pick_and_place_demo.xml`**(无需新写)
- `return_to_dock.xml`:`Sequence` { 路径回放器 → 卸货(pick_and_place 子树) → 任务完成判定器 }

整棵根树 `mission_grotto_walker.xml` 用一个 `Sequence` 串起来,中间加 `Fallback` 处理失败重试:

```
Sequence
├── explore_until_target          (L2 子树)
├── sample_at_target              (L2 子树,复用 pick_and_place)
├── return_to_dock                (L2 子树)
└── mission_done                  (L3 节点,判定收尾)
```

失败分支可挂 `RetryUntilSuccessful` 装饰器,N 次失败后转 `FAILURE` 触发安全停机。

---

## 6. 与同 workspace 内其它包的关系

| 包 | 关系 |
|---|---|
| `agx_arm_moveit` | 提供 `/move_action` (move_group) 和 SRDF(被本包 regex 解析) |
| `agx_arm_ctrl` | 真机 / 仿真硬件驱动,本包不直接依赖,通过 MoveIt 间接联动 |
| `go2_bt_ws`(同 workspace 不同根) | 提供 `bt_runner` 基类 vendored 的 BT.ROS2 库(本包 link 那个 .so) |
| `l4t/agx_arm_ws/piper_bt_planner` | **不同** BT 框架项目,只用作 API 兼容参考,不直接复用代码 |

---

## 7. 文件清单(供协作者跳读)

源码:
- `bt_trees/*.xml` — 3 个 BT 树
- `src/bt_runner_node.cpp` — TreeExecutionServer 子类
- `src/bt_plugin_registration.cpp` — 节点 plugin 入口
- `src/{set_pose_stamped_node,move_arm_to_pose_action_node,move_arm_to_named_action_node}.cpp` — 3 个 leaf 节点实现
- `src/named_target_loader.cpp` + `include/lgw_bt_planner/named_target_loader.hpp` — SRDF 命名姿态解析
- `src/pose_string_parser.cpp` + `include/lgw_bt_planner/pose_string_parser.hpp` — PoseStamped 字符串解析
- `launch/{lgw_bt_planner,bt_with_lgw_moveit}.launch.py` — 启动入口

文档:
- 本文件
- `bt-decision-framework.drawio.xml` — 配套结构图

---

## 8. 已知约束 / 注意事项

- **BT 默认 SUCCESS 后保持 child 状态**:重启树需重启 `bt_runner` 或显式 reset;不要期望 tickOnce() 自动重跑。
- **joint drift**:1e-5 m position box + 1e-3 rad orientation tol 强制单 IK basin,但需 IK seed 才能稳定收敛(目前 IK seed 在调试中被刻意禁用,真机部署前要恢复,见 `move_arm_to_pose_action_node.cpp` 注释)。
- **j4/j6 关节范围**:从 ~2.3 rad 收紧到 < 0.11 rad 后 IK 稳定;SRDF 里的关节约束是关键。
- **本容器 `colcon test` 不可用**:单元测试改跑 build/ 下 gtest 二进制。
- **真机 + mock 绝对禁忌同启**:会出现 `/move_action` 多源竞争,BT 收到的 SUCCESS/FAILURE 不可信。
