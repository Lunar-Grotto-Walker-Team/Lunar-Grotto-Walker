# servo_node 瘦身重构:范围、顺序与验收标准

- 日期:2026-10-31
- 状态:**待排期**(前置条件见 §5:必须在"接管内摆位"真机验证之后)
- 作者:Claude Code(与用户共同评审)
- 对象:`src/lgw_bt_planner/src/servo_node.cpp`(1831 行)
- 相关:`docs/superpowers/specs/2026-10-30-lgw-pinned-posture-lookat-design.md`、
  `docs/superpowers/plans/2026-10-31-lgw-preposition-in-takeover.md`

---

## 1. 现状:一个类扛了四件事

`servo_node.cpp` 1831 行。按区块:

| 区块 | 行范围 | 行数 | 职责 |
|---|---|---|---|
| 参数声明/加载 | 376–548 | 173 | 90+ 个 `declare_parameter`,无范围/无描述 |
| 订阅回调 | 549–724 | 176 | 轨迹、关节、Servo 状态、action 状态 |
| 控制拍 + 跟踪 | 725–1014 | 290 | `update_tracking_locked` 单函数 **237 行** |
| Servo 生命周期 | 1031–1147 | 117 | start/pause/unpause 的同步调用与重试 |
| 服务 | 1148–1400 | 253 | takeover / release / pause / resume |
| 辅助 | 1401–1661 | 261 | TF、复位、状态发布、饱和判据 |
| 成员 | 1662–1831 | 170 | |

**状态机本身不臃肿**:`servo_state_machine.cpp` 153 行,纯 `handle(event)`,六个状态
(IDLE / PREEMPT / TRACKING / HOLD / RELEASING / FAULT),有独立单测。问题在于节点类
把状态机、控制律、Servo 生命周期、ROS 接线四件事装进了同一个类。

## 2. 具体问题(逐条可核对)

| # | 问题 | 证据 | 影响 |
|---|---|---|---|
| 1 | `update_tracking_locked` 一个函数干八件事 | 778–1014,237 行 | 任何改动都要通读 |
| 2 | "失败 → HOLD" 样板重复 **7** 次 | `sm_.handle(TARGET_LOST); geometry_source_="none"; return true;` ×7 | 改语义要改七处 |
| 3 | lookat 模式用两个 bool 表示 | `lookat_target_relative_` / `lookat_pinned_posture_` | 加第三个模式时启动日志漏改(2026-10-31 已发生) |
| 4 | 三套模式各自一份错误处理 | 867 / 896 / 915 段 | 逻辑重复 |
| 5 | 会"炸"的死配置 | `force_cancel_moves := true` → 一律拒绝 takeover,提示 "not implemented in v1" | 误设即瘫痪 |
| 6 | 参数无校验 | 全部 `declare_parameter` 直取 | 非法值运行中静默异常 |
| 7 | 未接 lint | 无 `ament_lint_auto` | 只写不读的字段无人提醒(如曾经的 `posture_last_status_`) |
| 8 | 长锁 | 整个控制拍持 `mutex_` | 未来任何阻塞调用都会停摆 100 Hz |

## 3. 已有的良好实践(不要动)

- 多 callback group + 局部 `SingleThreadedExecutor`(为规避 handler 内同步 subcall 饿死执行器);
- `_locked` 后缀约定;
- `RELEASE` 幂等、`HOLD` 必须等新的 `TARGET_OK`;
- 纯函数外置(`geometry_source`、`lookat_setpoint`、`lookat_standoff_setpoint`、`control_law`);
- 状态机独立成文件并有单测。

## 4. 目标结构

```
servo_node.cpp        ~200 行   rclcpp::Node 子类,只做接线
servo_params.{h,cpp}           参数结构体 + declare_and_load() + 范围校验
servo_session.{h,cpp}          会话:状态机驱动 + Servo pause/unpause + 降级
tracking_loop.{h,cpp}          每拍:源选择 → 位姿求解 → 控制律 → 发布(不依赖 Node)
lookat/*                       纯几何(已存在,不动)
preposition/*                  判据 + goal 构造(摆位功能引入)
```

`tracking_loop` 接受"当前状态 + 目标 + 参数",返回"要发的 twist + 新状态",可以用合成
数据直接单测,不需要 ROS 图。

## 5. 顺序与前置条件

**前置条件:必须在"接管内摆位"完成真机验证之后**。理由:

- `servo_node` 是唯一在真机上跑通过视觉伺服的路径,重构期间没有替代验证物;
- 摆位功能与重构会同时触碰 takeover 与 `update_tracking_locked`,叠在一起出问题无法归因。

排期顺序(风险从低到高,每步独立可回退):

1. **删 `force_cancel_moves`**(或改成加载时抛错)——零风险
2. **`hold_and_return()` helper**,收敛 7 处样板——低风险
3. **拆 `update_tracking_locked`** 为 `pick_source()` / `solve_setpoint()` /
   `apply_control_law()` / `publish_status()`——纯搬家,有既有单测兜底
4. **`enum class LookatMode`** 替换两个 bool,三套分支各自收成函数——低风险
5. **参数结构体化 + 范围校验**(`servo_params.{h,cpp}`)——纯搬家
6. **接 `ament_lint_auto`**(clang-tidy / uncrustify)——零行为风险
7. **组件化**(`rclcpp_components`)——可选,收益是进程内组合与零拷贝

第 1–6 项完成后预计 1600 行左右,但价值主要在**可读性**(237 行函数变成 4 个 60 行
函数),不是行数。

## 6. 验收标准(逐条可检验)

1. 每个文件只有一个"为什么会被改"的理由;
2. 纯逻辑不依赖 rclcpp,可用合成数据毫秒级单测;
3. 函数不超过约 50 行,超出必须有注释说明理由;
4. 状态用类型表达,不用 bool 组合或裸字符串;
5. 参数有范围校验,非法值在启动时失败;
6. "失败 → 停机"只有一条代码路径;
7. 新增一个 lookat 模式只需改一处(不动控制环、状态机、日志)。

## 7. 与"接管内摆位"的关系

摆位计划**不因本重构而改写**,但加三条约束,避免给重构添阻力:

1. 新逻辑一律进纯模块(`preposition_decision`、`preposition_goal`),节点侧只保留
   "调用 + 分支";
2. 节点侧新增代码集中在 `handle_takeover` 和一个自包含的
   `bool maybe_preposition_locked(...)` 里,便于将来整体搬进 `servo_session`;
3. 不新增"模式类"的 bool;`preposition_enable` 是开关不是模式,可以保留。
