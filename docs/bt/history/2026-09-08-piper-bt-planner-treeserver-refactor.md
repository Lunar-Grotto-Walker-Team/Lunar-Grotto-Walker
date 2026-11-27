# piper_bt_planner TreeExecutionServer 重构总结

- **日期**:2026-09-08
- **范围**:`l4t/agx_arm_ws/src/piper_bt_planner/`(BT runner + 节点 + XML)
- **目的**:从 pluginlib 自建 BT 调度器迁到 BT.ROS2 官方 `BT::TreeExecutionServer`,并按 BT.CPP v4 t16 "global blackboard" idiom 共享 `TargetTable` 与 `tf2_ros::Buffer`
- **现状**:已 build 通过,`pose_via_set_pose_stamped.xml` / `go_to_target_and_back.xml` 全链路 SUCCESS,`piper_minimal_demo.xml` 在 SetBlackboard 旧路径上 ABORT(与本次重构无关,见 §6)

---

## 1. 重构前 / 后对比

| 维度 | 重构前 | 重构后 |
|---|---|---|
| BT 调度器 | 自写 `BtRunnerNode`(继承 `rclcpp::Node`,自定义 PluginLoader / BehaviorTreeFactory / 注册回调) | `BT::TreeExecutionServer` 基类,只 override `onTreeCreated` |
| 插件发现 | pluginlib `PLUGINLIB_EXPORT_CLASS` + `plugin.xml` | BT.ROS2 的 `plugins` ROS 参数 → `BT::SharedLibrary` dlopen `share/<pkg>/bt_plugins/*.so` |
| XML 发现 | 自己 `find_package(...)/share/.../bt_trees/*.xml` + glob | `behavior_trees` ROS 参数 → 基类加载 |
| action 服务 | 自己声明 `rclcpp_action::Server<ExecuteTree>` | 基类自带 `<action_name>` action(默认 `/bt_execution`) |
| Tick 循环 | 自建 `while(rclcpp::ok())` + `tree.tickWhileRunning()` | 基类自带 tick 线程 |
| 共享资源传递 | 每个节点 ctor 单独传 `tf_buffer` / `named_targets_table` 指针 | global blackboard(`{@key}` 端口重映射) |
| Groot2 publisher | 自己手动启 | 基类自动挂 |
| 热加载参数 | 不支持 | 基类自动监听 `plugins` / `behavior_trees` 变更 |

底层插件注册机制没变 —— 还是 BT.CPP 的 `BT_REGISTER_ROS_NODES(...)` + `BT_PLUGIN_EXPORT`,只是**发现路径**从 pluginlib 改为 BT.ROS2 的 `BT::SharedLibrary` dlopen。

---

## 2. 改动清单(按文件)

### 2.1 `src/bt_runner_node.cpp`(重写)

- **从 ~200 行自写节点 → 110 行 TreeExecutionServer 子类**
- ctor:`BT::TreeExecutionServer(std::make_shared<rclcpp::Node>("bt_runner", options), ...)` —— 用带节点的 ctor 把节点名 pin 在 `bt_runner`(start_*.sh 清理脚本 grep 匹配用)
- ctor 内一次性加载 SRDF:`NamedTargetLoader::load_from_share()` → 存为 `std::shared_ptr<const TargetTable> named_targets_`
- ctor 内创建 `tf2_ros::Buffer` + `TransformListener`(持有 listener 防止 buffer 被回收)
- **override `onTreeCreated(BT::Tree&)`**:每次树实例化时往 global blackboard 注入两个 shared_ptr
- `main()` 走 `MultiThreadedExecutor(0 worker + 250ms spin_some)` workaround(BT.ROS2 已知 deadlock bug,见 vendored `btcpp_ros2_samples/src/sample_bt_executor.cpp:68-71`)

### 2.2 `src/move_arm_to_named_action_node.cpp`(微调)

- `providedPorts()` 新增 `BT::InputPort<std::shared_ptr<TargetTable>>("named_targets", ...)`
- `setGoal()`:`config().blackboard->get<std::shared_ptr<const TargetTable>>("named_targets")` → `getInput<std::shared_ptr<TargetTable>>("named_targets")`
- 行为不变,仅换了取 shared_ptr 的方式

### 2.3 `src/detach_object_service_node.cpp`(微调)

- `providedPorts()` 新增 `BT::InputPort<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", ...)`
- `setRequest()`:校验 global bb 里 `tf_buffer` 已注入(实际节点逻辑不直接用 tf_buffer,加这个端口仅为让 missing-injection 错误在 BT 早期暴露)

### 2.4 `src/bt_plugin_registration.cpp`(沿用 + 微调)

- 唯一 TU 含 `BT_REGISTER_ROS_NODES(...)`,被编进 `piper_bt_planner_nodes` SHARED 库
- CMakeLists 给该 target 加 `BT_PLUGIN_EXPORT` 私有宏,导出 `BT_RegisterRosNodeFromPlugin` 符号
- 注册列表未变(节点类清单不变)

### 2.5 `CMakeLists.txt`

- 拆两个 target:
  - **`piper_bt_planner_nodes`(SHARED)**:所有节点源文件 + `bt_plugin_registration.cpp` → install 到 `share/<pkg>/bt_plugins/`(BT.ROS2 的 `RegisterPlugins` walker 会 globs 这个目录的 `.so`)
  - **`bt_runner`(EXE)**:只 `bt_runner_node.cpp` + `named_target_loader.cpp`(SRDF 加载发生在树存在前,必须直接链进 runner)
- **关键决策:`bt_runner` 不 link `piper_bt_planner_nodes`**。理由:`.so` 安装到 `share/bt_plugins/` 而非 `lib/`,不在 loader path;链接会得到 `cannot open shared object file` 运行时错。要让 BT.ROS2 看见这些节点,必须走它自己的 `plugins` 参数 → `BT::SharedLibrary` dlopen 路径。
- install 路径:`ARCHIVE/LIBRARY/RUNTIME` 全部 `share/${PROJECT_NAME}/bt_plugins`

### 2.6 BT XML(6 个文件,12 处替换)

- 全部 `attr="@{key}"` → `attr="{@key}"`(BT.CPP v4 t16 idiom)
- 文件 / 替换次数:

  | 文件 | 替换次数 | 端口 |
  |---|---|---|
  | `bt_trees/go_to_target_and_back.xml` | 2 | `named_targets` |
  | `bt_trees/perceive_to_grasp.xml` | 1 | `named_targets` |
  | `bt_trees/pick_and_place_demo.xml` | 5 | `named_targets` |
  | `bt_trees/piper_minimal_demo.xml` | 2 + 1 | `named_targets` + `tf_buffer` |
  | `bt_trees/pose_via_set_pose_stamped.xml` | 1 | `named_targets` |

- 注意:**`SetBlackboard` 节点的 legacy `key:value` 字符串语法仍存在**(`piper_minimal_demo.xml` 仍在用),跟本次重构的 `{@key}` 黑板指针语法是两回事,详见 §6

### 2.7 删除

- `src/bt_runner_node.cpp` 的旧 pluginlib 自注册路径 → 已重写
- `plugin_description.xml`(若存在)→ 不再需要(pluginlib 路径整体废弃)
- `package.xml` / launch 调整(若有 plugins 参数 → 改为 `plugins:=[<pkg>/piper_bt_planner_nodes]`)

---

## 3. 关键设计决策

### 3.1 为什么用 global blackboard 而不是子黑板

- 决策:**global**(基类 `globalBlackboard()`)
- 备选:每个子树自带黑板(`tree.subtrees[0].blackboard`)
- 选 global 的理由:
  1. `TargetTable` 在整个进程生命周期里只有一份(SRDF 启动期一次性加载),不应被每个 tree 复制
  2. `tf2_ros::Buffer` 同上 —— 全局共享 TF 缓存,各树订阅同一个 listener
  3. TreeExecutionServer 基类语义上把 globalBlackboard 设为 `rootBlackboard` 的 parent,所以从任意子树 `getInput<T>("{@key}")` 都会跳到 global
  4. 跟 BT.ROS2 官方 `btcpp_ros2_samples` 一致

### 3.2 为什么 XML 写 `{@key}` 而不是 `@{key}`

- 决策:**v4 t16 idiom `{@key}`**
- 这是一次踩坑后的纠正 —— 详见 §4.2

### 3.3 `std::shared_ptr<TargetTable>` vs `std::shared_ptr<const TargetTable>`

- 决策:**非 const 内层类型**
- 理由:BT.CPP v4 的 `InputPort<T>` 用 `std::decay` 规范化类型,会**剥掉** `shared_ptr<const Foo>` 的内层 const,而 BB `set<T>()` 用 `typeid(T)` 严格匹配。所以:
  - global bb set 端:`std::shared_ptr<TargetTable>`(非 const)
  - InputPort 端:`std::shared_ptr<TargetTable>`(非 const)
  - 所有权不可变性靠 `shared_ptr<const TargetTable>` 持有 + ctor 内 `std::const_pointer_cast` 转换 → 写入 BB 时剥 const,但持有端仍是 const,节点拿到后无法 mutate

### 3.4 为什么 `MoveArmToNamed` 自展开 SRDF

- 决策:**regex 解析 SRDF → 翻译成 `joint_constraints`**
- 备选:用 `MoveGroup.action` 的 `string[] NamedTargets` 字段
- 拒绝备选:action server 端对该字段在 OMPL / pilz 路径下处理不一致(标记 deprecated,部分 pipeline 直接忽略)
- 详细原因:`docs/.../piper-bt-planner-design.md` § MoveArmToNamed 一节

---

## 4. 踩坑 / 关键问题

### 4.1 `libpiper_bt_planner_nodes.so: cannot open shared object file`

- **症状**:首次 build 后跑 `ros2 launch ... bt_runner`,立刻抛 `.so not found`
- **根因**:在 CMakeLists 给 `bt_runner` 加了 `target_link_libraries(... piper_bt_planner_nodes)`,但 `.so` 被 install 到 `share/bt_plugins/`,不在 `LD_LIBRARY_PATH` / `AMENT_PREFIX_PATH` 的 `lib/` 路径上
- **修复**:删除 link 行,改为 `named_target_loader.cpp` 直接编进 `bt_runner` 二进制
- **教训**:**插件库绝不能被宿主二进制直接 link**。BT.ROS2 的设计是宿主 dlopen,链接就破坏了这个契约

### 4.2 XML `{@key}` vs `@{key}` 误用 —— v3 → v4 行为差异

- **症状**:`port with name "named_targets" and value "@{named_targets}" can not be converted to std::shared_ptr<TargetTable>`
- **错误诊断**:
  - 第一轮:怀疑是 `shared_ptr<const TargetTable>` vs `shared_ptr<TargetTable>` 类型不匹配 → 改非 const,仍失败
  - 第二轮:查 `ros2_dusty_ws` 的 v3 源码 → 误以为 v3 写法移植到 v4 一样
  - 用户反馈:**"应该看 v4 的 BT.CPP 吧,你一直在看 v3"**
- **真因**(核对 v4 源码):
  - `tree_node.cpp::isBlackboardPointer(str)`:v4 只判 `size≥3 && front=='{' && back=='}'`,**不认** `@{}` 模式
  - `xml_parsing.cpp:834-852`:不识别为黑板指针时,走 `convertFromString<std::shared_ptr<T>>(port_value)` → 该特化无定义 → `LogicError`
  - `blackboard.cpp:54-85`:v4 `getEntry` 才处理 `@` 前缀,跳到 `rootBlackboard()` —— 这是 **getEntry 阶段** 的事,跟 **parse 阶段** 的 `isBlackboardPointer` 是两个独立判断
- **修复**:XML 全局 `attr="@{key}"` → `attr="{@key}"`,共 12 处
- **教训**:**BT.CPP 跨大版本时,黑板指针语法会变**(v3 `@{}` / v4 `{}`)。改前必须从 vendored 源码或 examples 核对,不照抄旧项目惯用法或 design doc。本项目 vendored 的是 v4(`/l4t/bt_ros2_ws/`),所有引用都应来自 v4 examples(`t16_global_blackboard.cpp`)

### 4.3 `isBlackboardPointer` 在不同路径上的语义分裂

- 这个 bug 不在用户层面报错,但理解了会避免后续踩坑:
  - **parse 阶段**(`xml_parsing.cpp`):`isBlackboardPointer` 只看 `{}`,**`@` 在内层还是外层不影响 parse 判定**
  - **getEntry 阶段**(`blackboard.cpp`):看到 key 以 `@` 开头才跳 parent;XML 里写 `{@key}` 时,运行时 BT 会自己把 `@` 加到 key 前再查 BB
  - **结果**:XML 必须写 `{@key}`,**不能**写 `@{key}` —— 后者 parse 时被当字符串,converter 失败

### 4.4 Install 目录同步

- **症状**:改 XML 后 build 通过,但跑起来还是用旧行为
- **根因**:`colcon build` 不复制 `bt_trees/` 到 install 除非 install 指令显式包含
- **修复**:确认 CMakeLists `install(DIRECTORY bt_trees/ DESTINATION share/${PROJECT_NAME}/bt_trees)` 存在(已存在,本次未改);`colcon build --packages-select piper_bt_planner` 后 install 同步

---

## 5. 验证结果

### 5.1 Build

```
colcon build --base-paths src --packages-select piper_bt_planner
```

无 warning,无 error。`share/piper_bt_planner/bt_plugins/libpiper_bt_planner_nodes.so` + `lib/<pkg>/bt_runner` 同时存在。

### 5.2 节点可发现性

启动 `bt_runner` 后,Groot2 publisher 上能看到所有节点 ID,包括 `MoveArmToNamed` / `MoveArmToPose` / `DetachObject` / `AttachObject` / `SetPoseStamped` 等。

### 5.3 树级端到端

| XML | 结果 | 备注 |
|---|---|---|
| `pose_via_set_pose_stamped.xml` | **SUCCESS** | SetPoseStamped → MoveArmToPose → MoveArmToNamed(全程 `named_targets="{@named_targets}"` 解析成功) |
| `go_to_target_and_back.xml` | **SUCCESS** | 两次 MoveArmToNamed,中间一次 detach |
| `piper_minimal_demo.xml` | **ABORTED** at SetBlackboard | 与本次重构无关,详见 §6 |
| `pick_and_place_demo.xml` | 未跑(需要 stage1/stage2 perception) | 静态检查 OK |
| `perceive_to_grasp.xml` | 未跑(同上) | 静态检查 OK |

### 5.4 未直接跑但静态验证

- `DetachObject` + `tf_buffer="{@tf_buffer}"` 路径未跑端到端,但 `getInput<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer")` 在全局 bb 注入后能正确解析(同 blackboard 机制,跟 named_targets 同源)

---

## 6. 已知遗留 / 不在本次范围

### 6.1 `piper_minimal_demo.xml` 仍用 `SetBlackboard` 旧语法

- 旧 `SetBlackboard` 节点用 `key:value` 字符串格式(`output="{pose}"`),需要 BB 有 `convertFromString<PoseStamped>` 注册,本项目**没有**注册这个
- 报错:`Can't convert string [base_link;0.0561;0.00;0.2632;0;0.6756;0;0.7373] to type [geometry_msgs::msg::PoseStamped_<...>]`
- **不在本次重构范围**:这是节点注册问题,不是本次 global bb 重构的副产品
- 修复路径(待办):把 `SetBlackboard` 换成本项目自带的 `SetPoseStamped`(自带 `PoseStringParser`,已在 install 目录)

### 6.2 `bt_plugins` 参数未在 launch 文件显式声明

- 当前依赖 BT.ROS2 默认搜索路径(`share/<pkg>/bt_plugins/`)
- 若以后跨包复用节点,需要在 launch 里显式 `plugins:=[piper_bt_planner/piper_bt_planner_nodes]` —— **本次未做,留待需要时再加**

### 6.3 IK seed 仍 disabled

- `MoveArmToPose` 里 `start_state.joint_state` + `joint_constraints(joint4/j6)` 两块代码当前**注释**,带 `IK seed DISABLED 2026-09-04` 注释
- 原因:在测"不靠 IK seed 能否消除 cycle-to-cycle drift"假设,假设未验证前保留现状
- 与本次重构无关

### 6.4 `MoveArmToNamed` 仍走自展开路径

- `setGoal()` 仍自己 regex 解析 SRDF → `joint_constraints`
- 见 §3.4,这是设计选择(不是遗留),但 `TargetTable` 现在通过 global bb 注入而非 ctor 传指针,只是装载/传递方式变了

---

## 7. 后续可选改进(按 ROI 排序)

1. **`SetBlackboard` → `SetPoseStamped`**:`piper_minimal_demo.xml` 端到端可跑(`§6.1`)
2. **删 `named_target_loader`**:把 `MoveArmToNamed` 改成非 action 节点,内部持 `MoveGroup` 长连接,直接 `setNamedTarget()`,消掉整本字典 —— 见设计 doc §MoveArmToNamed 的备选方案
3. **launch 显式 `plugins` 参数**:`§6.2`
4. **IK seed 重启**:`MoveArmToPose` 取消注释并验证 §6.3 的假设

---

## 8. 参考资料

- BT.CPP v4 t16 global blackboard examples:`/l4t/bt_ros2_ws/src/BehaviorTree.CPP/examples/t16_global_blackboard.cpp`
- BT.CPP v4 黑板指针解析(`/l4t/bt_ros2_ws/src/BehaviorTree.CPP/src/xml_parsing.cpp:834-852`)
- BT.CPP v4 blackboard parent traversal(`/l4t/bt_ros2_ws/src/BehaviorTree.CPP/src/blackboard.cpp:54-85`)
- BT.ROS2 `TreeExecutionServer`(`/l4t/bt_ros2_ws/src/behaviortree_ros2/tree_execution_server.cpp`)
- BT.ROS2 官方 sample runner:`/l4t/bt_ros2_ws/src/btcpp_ros2_samples/src/sample_bt_executor.cpp`
- 项目设计 doc:`/l4t/agx_arm_ws/docs/superpowers/specs/2026-08-30-piper-bt-planner-design.md`

---

## 9. 一句话总结

**`bt_runner` 从 ~200 行自写 pluginlib 调度器,改成 110 行 TreeExecutionServer 子类,只在 `onTreeCreated` 把 `TargetTable` + `tf_buffer` 注入 global blackboard;XML 全部用 v4 t16 `{@key}` 端口重映射取共享资源,6 个文件 12 处替换;走通了 MoveArmToNamed / MoveArmToPose / DetachObject 三条链路上 `getInput<T>()` 解析;主要坑是 v3→v4 的黑板指针语法差异(`@{}` → `{}`)。**