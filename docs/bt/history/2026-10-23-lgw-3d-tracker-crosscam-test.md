# lgw_3d_tracker 双相机同目标(单物体)跟踪测试

**日期**:2026-10-23
**场景**:两相机(head D435i / arm D435)视野内有且仅有一个目标物(wood_block)
**链路**:YOLOE 双相机(Foxy,host)→ `/perception/two_stage/camera_{head,arm}/result`
→ `unified_track_node`(Humble,容器)→ `/perception/tracks/unified`
**状态**:主结论成立(同一轨迹合并);另有 2 个待决策问题(见 §4)

---

## 1. 测试方法

新增逐 tick 跟踪日志(trace logging),每 tick 一条 JSONL:
输入(dets/adapted)、观测(head/arm 的 odom 位置与置信度)、关联明细
(match 距离 / tentative / 新建 / 未匹配)、轨迹快照(id/status/hits/cams/位置)。
每 5s 输出一条汇总到 console + `.summary.log`。

启动:

```bash
docker exec -d self_l4t bash -lc "source /my_ws/l4t/l4t_setup.sh; \
  source /my_ws/l4t/lg_walker_ws/install/setup.bash; \
  exec ros2 launch lgw_3d_tracker unified_track_node.launch.py \
    trace_log_dir:=/my_ws/l4t/lg_walker_ws/.run trace_tag:=_crosscam"
```

分析:

```bash
python3 l4t/lg_walker_ws/scripts/analyze_tracker_trace.py \
  l4t/lg_walker_ws/.run/lgw_3d_tracker_<...>_crosscam.jsonl
```

日志(保留):

| 轮次 | JSONL / summary | stdout |
|---|---|---|
| run1(修复前,283s) | `.run/lgw_3d_tracker_20261023_085224_crosscam.{jsonl,summary.log}` | `.run/lgw_3d_tracker_crosscam_20261023_165212.log` |
| run2(修复后,273s+) | `.run/lgw_3d_tracker_20261023_085747_crosscam2.{jsonl,summary.log}` | `.run/lgw_3d_tracker_crosscam2_20261023_165735.log` |

注:容器时钟为 UTC(trace 内 `t_ms` 是节点相对时间,不受影响)。

---

## 2. 结论:双相机检测合并到同一轨迹 ✅

**run2(修复后,273.0s / 3587 ticks @ 13.1 Hz)**

| 指标 | 结果 |
|---|---|
| arm 观测 | 1793 / 1793 帧(100%) |
| head 观测 | 518 / 1794 帧(29%,其余为 0 检测) |
| 关联结果 | **2305 条观测全部配到 id=1**,`source_cameras=[head, arm]` |
| 新建轨迹 | 1 条主轨迹 + 5 条 1-hit 影子轨迹(均 ≤1.4s 自动删除) |
| match 距离 | head p50 **3.37 cm** / arm p50 **0.17 cm**,max 4.2 cm(< gate 15 cm) |
| 跨相机位置差(≤300ms 内 head/arm 观测对,2049 对) | p50 **3.44 cm**、p90 3.47 cm、max 4.2 cm;**100% ≤ 5 cm** |

**run1(修复前,283s)**:4404 对跨相机观测,同样 100% ≤ 5 cm;
主轨迹 id=1 同时含 arm+head。合并本身从始至终成立。

输出消息(`ros2 topic echo --once`):

```
track_id: 1
class_name: wood_block
observation_count: 1544
last_source_camera: camera_arm_color_optical_frame
source_cameras: [camera_arm_color_optical_frame, camera_head_color_optical_frame]
position: x=0.5705 y=-0.0648 z=-0.0705
```

**解读**:head 与 arm 的 3D 估计存在约 **3.4 cm 的固定系统偏差**
(head 观测在 KF 状态上方 3~4 cm),远小于 15 cm 门控,因此跨相机合并稳定。
该偏差更可能来自两相机对木块表面/中心点取点差异,不构成合并障碍。

---

## 3. 已修复的 bug:coasting 永不回到 confirmed

**现象(run1)**:一条每帧都在被观测的轨迹,状态 100% 显示 `coasting`
(1436 tick 中 1427 次),`hits` 正常累加。

**根因**:`Track.apply_observation()` 只更新状态量,不恢复 `status`;
`tick_lifecycle()` 里 confirmed→coasting 是单向的,一旦进入 coasting,
即使持续被观测也永远停在 coasting。

**修复**(`unified_track/track.py`):`apply_observation` 中若 `status == COASTING`
→ 置回 `CONFIRMED`(coasting 的语义是"暂时看不到、保留轨迹")。补单测
`test_track_returns_to_confirmed_after_gap`。

run2 验证:`confirmed 2306 / coasting 1282 / tentative 2`。

---

## 4. 待决策问题(未改行为)

### 4.1 同帧重复检测 → 影子轨迹 → ID 切换(run1 实测)

YOLOE head 有时一帧输出 **2 条同类检测**(同一 wood_block,odom 相距 ~1.8 cm,
一真一偏)。关联是 Hungarian **一对一**,第二条观测必然匹配失败 →
新建 tentative track(影子轨迹)。

run1 中影子轨迹存活足够久后**抢走主轨迹的关联**:

| 轨迹 | arm 匹配窗口 | head 匹配窗口 |
|---|---|---|
| id=1(主) | 0.2 → **154.7s** | 0.1 → 161.4s |
| id=4(影子) | **154.9s** → 282.8s | 104.1s → 282.7s |
| id=6(影子,head-only) | — | 239.4 → 251.0s(存活 12s) |

即 **~155s 发生 track_id 切换**(同目标、同位置,ID 从 1 变 4)。
run2 中同帧双检测只出现 5 次,影子轨迹都只有 1 hit 便过期,未发生切换。

**候选修复**(需确认):
- A. 同一 tick 内,未匹配观测若距"本 tick 已被匹配的轨迹" < 3~5 cm,则
  合并进该轨迹(不新建),不改变多目标语义;
- B. 关联前按相机做同类检测聚类/去重(保留置信度最高者)。

### 4.2 status 在 confirmed↔coasting 间抖动

run2 中 1281/1794 个 head 帧为 0 检测(单帧空档,p50 63 ms)。空 tick 时
`time_since_last_obs` 跨过 1 个 tick 即转 coasting,下一帧 arm 观测又转回
confirmed → **36% 的 tick 状态显示 coasting**(下游如果按 confirmed 过滤会看到闪烁)。

**候选修复**:新增 `coast_enter_ms`(如 300 ms),`time_since_last_obs > max(dt, coast_enter_ms)`
才进入 coasting。属行为变更,未擅自改。

### 4.3 head 检测置信度贴近阈值(非 tracker 问题)

head 置信度 p50 **0.155~0.167**,紧贴 `retention_confidence=0.15`;
arm 为 p50 0.78~0.80。head 只有 29%~66% 的帧出检测,跨相机合并因此
主要依赖 arm 维持轨迹、head 间歇纠偏。

---

## 5. 改动文件

| 文件 | 说明 |
|---|---|
| `lgw_3d_tracker/unified_track/trace_logger.py` | **新增**:JSONL 逐 tick 日志 + 5s 汇总 |
| `lgw_3d_tracker/unified_track/tracker.py` | `associate_and_update` 返回 `AssociationReport`(不改跟踪结果) |
| `lgw_3d_tracker/unified_track/track.py` | **修复** coasting→confirmed |
| `lgw_3d_tracker/unified_track_node.py` | trace 参数、逐 tick 记录、退出时写最终汇总;shutdown 幂等 |
| `launch/unified_track_node.launch.py` | 新增 `trace_log_dir` / `trace_tag` 参数 |
| `test/test_trace_logging.py` | **新增**:5 条单测(含 coasting 恢复、同帧双检测 characterization) |
| `scripts/analyze_tracker_trace.py` | **新增**:JSONL 离线分析 |

Python 包是 symlink/egg-link 安装,**改完无需 colcon build,重启节点即可**。

单测(容器内):

```bash
source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/lg_walker_ws/install/setup.bash
cd /my_ws/l4t/lg_walker_ws && python3 -m pytest src/lgw_3d_tracker/test/test_trace_logging.py -q
# 5 passed
```
