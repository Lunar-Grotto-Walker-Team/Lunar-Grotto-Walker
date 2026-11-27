# 3D 跟踪修复小结(2026-10-26)

**范围**:`lgw_3d_tracker`(Humble,容器)+ `yoloe_deploy`(Foxy,host)
**场景**:单木块,head(D435i)+ arm(D435)双相机均可见
**结论**:同一个目标在两相机下稳定合并为**唯一一条 3D 轨迹**,track_id 全程不变;
status 不再抖动;同帧重复检测不再产生影子轨迹。

---

## 1. 修复前的问题

| # | 现象 | 实测证据(2026-10-23) |
|---|---|---|
| P1 | 轨迹一进入 `coasting` 就永不回 `confirmed` | run1:`id=1` 的 1945 个 tick 中 **99.4% 显示 coasting**,而目标每帧都被观测 |
| P2 | status 在 confirmed↔coasting 之间抖动 | run2:2049 个 tick 中 **36% 显示 coasting**;head 每 3 帧就有 1 帧零检测(p50 空档 63 ms) |
| P3 | 同帧重复检测 → 影子轨迹 → **ID 切换** | head 一帧输出两条同类框(小框完整落在大框内,IoU 0.415 / IoM 1.0,3D 相差 1.8 cm);run1 中 **~155 s 时 track_id 从 1 变成 4** |
| P4 | 同一物体被拆成"顶面/正面"两个实例 | 2026-10-26 实测:两条 bbox 不构成包含(IoU 0.03 / IoM 0.06),3D 中心间距 **p50 3.03 cm** —— P3 的去重(阈值 3 cm)只覆盖 11% |
| P5 | YOLOE 的 2D tracker 在丢弃真实检测 | 它只输出"本帧匹配上且已激活"的轨迹,lost 不输出;新轨迹要连续 2 帧命中才输出 → 单帧检测被丢掉 |

---

## 2. 修复内容

| # | 修复 | 说明 |
|---|---|---|
| F1 | coasting → confirmed 恢复 | 重新观测到目标时把状态置回 `confirmed`(coasting 的语义是"暂时看不到") |
| F2 | `coast_enter_ms` 迟滞 | 缺失持续 ≥ 300 ms 才对外报 coasting;单帧空档不再翻状态 |
| F3 | 检测端 **IoM containment 去重** | 同类且 `intersection/min(area) > 0.8` 视为包含,保留更贴合(面积更小)的框。**不能用调低 NMS 的 IoU 代替**(实测那对框 IoU 0.415,且会误伤相邻目标) |
| F4 | 合并半径 3 cm → **4 cm** | 覆盖"顶面/正面"拆分(p50 3.03 cm,97.3% ≤3.5 cm);木块宽 ~4.8 cm,真实相邻同类目标中心距 ≥4.8 cm,不会误合并 |
| F5 | **建轨前抑制** | 未匹配观测若落在既有轨迹的合并半径内,直接丢弃(`AssociationReport.suppressed`),避免"每 tick 新建一条再被去重删一条"的 id 空转 |
| F6 | 追踪日志 | 每 tick 一条 JSONL(输入/观测/关联/抑制/去重/轨迹快照)+ 5 s 汇总 |
| F7 | 关闭 YOLOE 2D tracker | `enable_2d_tracker: false` → `track_batch()` 回落 `predict_batch()`;identity 只由 3D tracker 负责 |

**为什么关掉 2D tracker**:它的 `track_id` 在本项目里没有任何功能消费者(只有诊断打印);
它不提高召回(不输出 lost),反而要求 2 帧确认、丢掉单帧检测;跨相机 identity 它本来就做不到。

---

## 3. 涉及文件

### 3.1 `l4t/lg_walker_ws/`(3D tracker)

| 文件 | 改动 |
|---|---|
| `src/lgw_3d_tracker/lgw_3d_tracker/unified_track/track.py` | F1 coasting→confirmed;F2 `coast_enter_ms` 迟滞 |
| `src/.../unified_track/tracker.py` | `AssociationReport`;F5 建轨前抑制;3D 影子轨迹去重 `dedupe_tracks()`;新增 `coast_enter_ms` / `duplicate_track_distance_m` 参数 |
| `src/.../unified_track/config.py` | 新增 `coast_enter_ms`(300)、`duplicate_track_distance_m`(0.04) |
| `src/.../unified_track_node.py` | 传递新参数;F6 逐 tick 日志;`shutdown()` 幂等 |
| `src/.../unified_track/trace_logger.py` | **新增**:JSONL + 周期汇总 |
| `src/lgw_3d_tracker/config/runtime.yaml` | `coast_enter_ms: 300`、`duplicate_track_distance_m: 0.04` |
| `src/lgw_3d_tracker/launch/unified_track_node.launch.py` | 新增 `trace_log_dir` / `trace_tag` |
| `src/lgw_3d_tracker/test/` | **新增** `test_tracker_lifecycle.py`、`test_tracker_dedupe.py`;更新 `test_trace_logging.py` |
| `scripts/analyze_tracker_trace.py` | **新增**:离线性状分析(出检率/关联/抑制/去重/status/跨相机一致性) |

### 3.2 `yoloe_deploy/`(检测侧)

| 文件 | 改动 |
|---|---|
| `yoloe_transfer/inference.py` | F3 `box_iom()` / `suppress_contained()` / `_postprocess_candidates()`;F7 `enable_2d_tracker` 开关 |
| `config/runtime.yaml` | `containment_dedup_enabled: true`、`containment_dedup_iom: 0.80`、`enable_2d_tracker: false` |
| `tests/test_containment_dedup.py` | **新增** 7 条单测 |

### 3.3 测试

```
容器内   python3 -m pytest l4t/lg_walker_ws/src/lgw_3d_tracker/test/ -q   → 17 passed
宿主机   cd /home/unitree/yoloe_deploy && python3 -m pytest tests/ -q     → 19 passed
```

---

## 4. 修复前后性能对比

### 4.1 检测侧(YOLOE)

| 指标 | 修复前 | 修复后 |
|---|---|---|
| head 出检 | **1.43 条/帧**(2D tracker 过滤后) | **1.79 条/帧**(关 2D tracker) |
| arm 出检 | 1.00 条/帧 | 1.00 条/帧 |
| 同帧重复(包含关系) | 906 帧出现 15 次,均未被过滤 | 未再观测到(IoM 去重生效) |
| 同帧"顶面/正面"拆分 | 每帧 2 条,均进入 tracker | 仍检测出 2 条,但第 2 条在 3D 层被抑制(22%~39% 的 tick) |

> head +25% 出检率来自关掉 2D tracker(修复前它丢掉了只出现一帧的检测)。
> 注意这是**不同轮次**的两次测量,场景光照/姿态相同但非同一时刻。

### 4.2 跟踪侧(lgw_3d_tracker)

| 指标 | 修复前(10-23 run1/run2) | 修复后(10-26) |
|---|---|---|
| 主轨迹 coasting 占比 | **99.4% / 36%** | **0.1%** |
| 输出轨迹数 | 2~3 条,含影子轨迹 | **1 条** |
| `created` 轨迹(169~217 s) | 6 条 | **1 条** |
| track_id 稳定性 | **~155 s 时 1 → 4 切换** | **全程不变**(单条 id=1) |
| 同帧近邻抑制 | — | 406 次(22%)/ 953 次(39%) |
| 影子轨迹去重 | — | 0 次(建轨前抑制已覆盖) |
| 关联距离 p50 | 2.8 cm | **3.2~3.5 cm**(观测变多后仍在门控内) |
| 跨相机位置一致性 | p50 3.44 cm(仅位置对比) | p50 **3.99~4.37 cm**,100% ≤5 cm |
| 跨相机合并 | 成立,但会被影子轨迹打断 | **稳定成立**,`source_cameras=[head, arm]` 全程 |

> 单目标跨相机合并:169.4 s / 1884 tick @ 11.1 Hz,head 1349 条 + arm 942 条观测
> **全部配到同一个 id=1**。

### 4.3 A/B:关闭 2D tracker 的影响

| 指标 | 2D on | 2D off |
|---|---|---|
| head 出检 | 1.43 条/帧 | **1.79 条/帧**(+25%) |
| created 轨迹 | 1 | 1 |
| 关联距离 p50 | 3.53 cm | 3.24 cm |
| 跨相机一致性 p50 | 4.37 cm | **3.99 cm** |

---

## 5. 验证方法与日志

```bash
# 启动 tracker(带逐 tick 日志)
docker exec -d self_l4t bash -lc "source /my_ws/l4t/l4t_setup.sh; \
  source /my_ws/l4t/lg_walker_ws/install/setup.bash; \
  exec ros2 launch lgw_3d_tracker unified_track_node.launch.py \
    trace_log_dir:=/my_ws/l4t/lg_walker_ws/.run trace_tag:=_cross"

# 分析
python3 l4t/lg_walker_ws/scripts/analyze_tracker_trace.py \
  l4t/lg_walker_ws/.run/lgw_3d_tracker_<ts>_cross.jsonl
```

| 轮次 | 日志 |
|---|---|
| 修复前基线 | `.run/lgw_3d_tracker_20261023_085224_crosscam.jsonl`(283 s) |
| 修复后 head 单通道 | `.run/lgw_3d_tracker_20261026_092831_fix2.jsonl`(134 s) |
| 修复后跨相机 | `.run/lgw_3d_tracker_20261026_094004_cross.jsonl`(169 s) |
| 关 2D tracker | `.run/lgw_3d_tracker_20261026_095722_no2d.jsonl`(217 s) |

---

## 6. 配置项与回退

全部是配置,无需改代码;Python 包是 symlink/egg-link 安装,**改完重启节点即可**。

| 配置 | 当前值 | 回退值 | 作用 |
|---|---|---|---|
| `unified_track.coast_enter_ms` | 300 | 0 | coast 迟滞;0 = 一个空 tick 就翻 |
| `unified_track.duplicate_track_distance_m` | 0.04 | 0 | 重复观测/影子轨迹合并半径 |
| `inference.containment_dedup_enabled` | true | false | 检测端包含关系去重 |
| `inference.containment_dedup_iom` | 0.80 | — | 包含判据阈值 |
| `inference.enable_2d_tracker` | false | true | YOLOE 内 2D 跟踪开关 |

重启注意:`kill -15 <launch pid>` **不会**带走节点子进程,必须同时 kill 节点 pid,
并用 `ros2 node list | grep -c unified_track_node` 确认只有 1 个(同名双实例会静默丢消息)。

---

## 7. 遗留

1. **head 与 arm 的 3D 估计存在 ~4 cm 系统性偏差**(p50 4.37 cm → 关 2D tracker 后 3.99 cm)。
   不影响关联与合并(门控 15 cm),但若用于抓取,需要分别校准两个相机的 depth→3D 偏移。
2. `trajectory_visualizer_node` 依赖 YOLOE 的 `instance_id`(2D track_id),关掉 2D tracker 后失效(`instance_id=0`)。
   属调试工具,需要时把 `enable_2d_tracker` 临时改回 true。
3. `yoloe_deploy/` 不是 git 仓库,其改动只存在于工作目录,没有版本历史。
