# Servo 接管 / 视觉伺服

本目录存放"用 MoveIt Servo 接管 MoveIt 规划器、并接入 3D tracker 做视觉伺服"的调研与设计文档。

| 文档 | 内容 | 状态 |
|---|---|---|
| [2026-10-26-servo-takeover-design.md](2026-10-26-servo-takeover-design.md) | 现状链路调研、接入方式取舍、四层切分、状态机、3D tracker 改造清单、风险与待验证项 | 调研记录(含证据与推导) |
| [`../superpowers/specs/2026-10-26-lgw-servo-takeover-design.md`](../superpowers/specs/2026-10-26-lgw-servo-takeover-design.md) | **正式设计方案**:目标/边界/验收标准、L0–L3 设计、接口定义、测试策略、决策记录、文件清单 | 设计基线 |

## 一句话结论

`/control/joint_states` 被 JointStateBroadcaster 以 200Hz 持续 echo 占用,因此驱动只能有**一个写者**。
最终方案采用 **MoveIt 官方提供的 JTC 话题接缝**:Servo 把 `trajectory_msgs/JointTrajectory`
发到 `/arm_controller/joint_trajectory`,与 MoveGroup 的 FollowJointTrajectory action 汇入同一个
JointTrajectoryController —— 不新增 controller、不切换 controller、释放时无回跳。
整套系统**单栈启动**(一个驱动进程);BT 侧通过一个带状态机的服务节点完成接管/归还。

## 先读哪几节

- 想了解"为什么不能直接 publish 控制关节":§2.2
- 想了解"接缝选在哪、为什么":§5.1
- 想了解"单栈与开发期两栈":§5.2
- 想了解"怎么接管/归还":§5.3
- 想了解"驱动模式(move_j / move_js / CPV)":§5.4
- 想了解"3D tracker 要改什么":§8
- 实施前必须实测的项:§10
