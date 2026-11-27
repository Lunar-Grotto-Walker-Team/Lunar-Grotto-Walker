# 行为树 / 机械臂文档

| 文档 | 内容 |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | 包与节点边界、35 个 BT 节点、22 棵树、启动拓扑、设计约束 |
| [TOPICS.md](TOPICS.md) | 跨包端点契约（动作 / 服务 / 话题、提供方与使用方） |
| [bt-tech-summary.md](bt-tech-summary.md) | 设计摘要（BT.CPP 工作方式、节点分层、测试策略） |
| [design/](design/) | 决策框架图（drawio）与检测字段说明 |
| [history/](history/) | 开发记录（54 篇，按日期命名） |

## 怎么用

- 想知道**某棵树调用了什么**：先看 `src/lgw_bt_planner/bt_trees/<tree>.xml`，
  再对照 TOPICS.md 查端点由谁提供；
- 想知道**新加一个 BT 节点**要遵守什么：看 `bt-tech-summary.md` 与
  `src/lgw_bt_planner/src/bt_plugin_registration.cpp` 里的注册注释；
- 想知道**为什么这么改**：在 `history/` 里按日期或关键词搜（例如
  `0620` 相机、`servo`、`transit`、`tracker`）。

## 维护约定

- 新增/删除 BT 节点时同步更新 ARCHITECTURE.md 的节点表；
- 端点新增或语义变化时更新 TOPICS.md；
- 一次性调试记录放 `history/`，不要放在包目录里。
