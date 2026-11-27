# 3D tracker:status 抖动 + 同帧重复检测 修复进度

**日期**:2026-10-23
**关联**:`docs/2026-10-23-lgw-3d-tracker-crosscam-test.md`(发现问题的那轮测试)
**状态**:代码与单测全部完成;真机联合验证**未完成**(进行中被打断,见 §6)

---

## 1. 背景:要修的两个问题

2026-10-23 双相机单目标测试(run1/run2)暴露:

| # | 问题 | 实测证据 |
|---|---|---|
| A | status 在 confirmed↔coasting 抖动 | run2:2049 tick 中 697 次显示 coasting,而目标一直在视野内。head 每 3 帧就有 1 帧零检测(p50 空档 63 ms),一个空 tick 就翻状态 |
| B | 同帧重复检测 → 影子轨迹 → ID 切换 | head 一帧输出两条同类框(小框完整落在大框内,IoU 仅 0.415、IoM 1.0,3D 中心差 1.8cm);Hungarian 一对一 → 新建影子轨迹;run1 中 ~155s 时影子轨迹抢走关联,`track_id` 从 1 变 4 |

采用的修复顺序(已与用户确认):**先修生命周期与去重,最后再关闭 YOLOE 的 2D tracker**(否则噪声会掩盖效果)。

---

## 2. 已完成的改动(逐层)

### 第 0 层:coasting 永不回到 confirmed(已修)

`unified_track/track.py::apply_observation()`:重新观测到目标时 `COASTING → CONFIRMED`。
旧实现只在 lifecycle 里单向 `confirmed→coasting`,导致一条每帧都被观测的轨迹
100% 显示 coasting。

### 第 1 层:status 迟滞(新增 `coast_enter_ms`)

| 项 | 值 |
|---|---|
| 参数 | `coast_enter_ms`(默认 **300 ms**) |
| 规则 | 缺失持续 `≥ coast_enter_ms` 且 `> dt_ms` 才转 coasting;重新观测立刻回 confirmed |
| 参数链 | `config.py` → `config/runtime.yaml` → `Tracker.__init__` → `Track.tick()` |
| 效果 | run2 中那种单帧空档(63~75 ms)不再翻状态 |

`coast_enter_ms: 0` 可退回旧行为(一个空 tick 就翻)。

### 第 2 层:同帧重复检测抑制(IoM,检测端)

| 文件 | 改动 |
|---|---|
| `yoloe_deploy/yoloe_transfer/inference.py` | 新增 `box_iom()` / `suppress_contained()` / `_postprocess_candidates()`;`predict_batch()` 与 `track_batch()` 都改为先 containment 抑制、再 `class_aware_nms` |
| `yoloe_deploy/config/runtime.yaml` | `containment_dedup_enabled: true` / `containment_dedup_iom: 0.80` |

**为什么用 IoM 而不是调低 NMS 的 IoU**:
- 实测那对框 IoU 0.415 < `iou=0.65`,NMS 放行;
- `class_aware_nms` 只合并**不同 prompt 组**,这里两条同组,管不到;
- 调低全局 IoU 会误伤相邻的真实目标;IoM > 0.8 只在"一方被另一方基本包含"时生效。
- 保留**面积更小**的框:实测大框多出的 ~94 px 高度是木块下方阴影,会把 3D 中心拉偏 1.8 cm。

### 第 3 层:影子轨迹去重(3D 端,移植 BoT-SORT 思路)

| 文件 | 改动 |
|---|---|
| `unified_track/tracker.py` | 新增 `dedupe_tracks()`,在 `tick_lifecycle()` 内调用;结果存 `last_dedupe_removed` 供日志 |
| `unified_track/config.py` + `config/runtime.yaml` | `duplicate_track_distance_m: 0.03` |

规则:同类别 + 3D 距离 < 阈值 → 视为同一目标,保留更成熟的一条
(`CONFIRMED/COASTING` 优先 → `hits` 多者 → 更老者 → id 小者)。
与 BoT-SORT 的 `remove_duplicate_stracks` 同构,只是距离度量从 2D IoU 换成 3D 米制。

阈值 3 cm 的依据:重复检测实测相差 1.8 cm;真实同类木块宽约 4.8 cm,中心距至少 ~5 cm。

### 第 4 层(已实现,**尚未启用**):关闭 2D tracker

| 项 | 值 |
|---|---|
| 开关 | `inference.enable_2d_tracker`(当前 runtime.yaml 仍为 **true**) |
| 行为 | false 时 `track_batch()` 直接回落 `predict_batch()`(track_id 全 0,现有代码已对 `instance_id=0` 兜底) |

**依据**(读 ultralytics 8.4.33 源码):
- `BYTETracker.update()` 只返回 `tracked && is_activated`(`byte_tracker.py:395`),**lost 轨迹不输出** → 不提高召回;
- `activate()` 仅在 `frame_id == 1` 时立即激活,否则新轨迹要**连续 2 帧命中**才输出 → 丢掉所有单帧检测;
- 它的 `track_id` → 消息 `instance_id`,全项目只有 `verify_head_arm_consistency.cpp`(打印)和 1 个单测在用,**无功能消费者**;
- 3D tracker 的 `InputAdapter` 根本不读它,跨相机 identity 完全由 3D 层重做。

副作用提醒:关闭后 `trajectory_visualizer_node`(按 `instance_id` 画轨迹)会失效,属调试工具,可接受。

---

## 3. 测试证据

容器内(3D tracker):

```bash
source /my_ws/l4t/l4t_setup.sh && source /my_ws/l4t/lg_walker_ws/install/setup.bash
cd /my_ws/l4t/lg_walker_ws
python3 -m pytest src/lgw_3d_tracker/test/ -q
# 15 passed
```

| 测试文件 | 覆盖 |
|---|---|
| `test/test_trace_logging.py` | 日志格式、汇总 cross/single、coasting 恢复、同帧双检测(characterization) |
| `test/test_tracker_lifecycle.py` | 单帧空档不翻 coasting、≥300ms 才翻、重新观测回 confirmed、超时删除、`coast_enter_ms` 配置读取 |
| `test/test_tracker_dedupe.py` | 影子轨迹被清、阈值生效(1cm 不删 1.8cm)、不同类别不合并、保留更成熟轨迹、`duplicate_track_distance_m` 配置读取 |

宿主机(yoloe_deploy):

```bash
cd /home/unitree/yoloe_deploy && python3 -m pytest tests/ -q
# 19 passed
```

| 测试 | 覆盖 |
|---|---|
| `tests/test_containment_dedup.py`(新增 7 条) | 真机那对框被抑制、相邻目标保留、不同类别保留、阈值生效、后处理开关、`enable_2d_tracker=false` 回落 predict |

每个新功能都是先写失败测试、确认失败原因、再实现(red→green 全程执行)。

---

## 4. 当前真机状态(18:50 快照)

| 项 | 状态 |
|---|---|
| YOLOE(host) | 18:47 由本会话重启以加载新配置(此前 18:25 曾被他处重启过一次)。容器内 realsense 未动 |
| `unified_track_node` | PID 9473,**仍是 16:57 启动的旧代码**,未加载第 1/3 层修复 → **需重启** |
| 场景 | **已不是单目标**:head 60s 内 **0 检测**;arm 稳定 2 条检测,两条**相距很远**(IoU=0、IoM=0,bbox `(163,242,249,317)` conf≈0.89 与 `(20,122,86,168)` conf≈0.43)→ 是两个真实的不同物体,不是重复框 |

**结论**:containment 去重在 60s 探针里没有被触发(因为没有出现包含关系),
既不能证明有效也没有回归证据。要验证第 2 层,必须先把场景恢复成
"两相机视野里只有一个目标",或人为构造包含关系。

---

## 5. 复现 / 观测工具

```bash
# 逐 tick 跟踪日志(JSONL + summary)
docker exec -d self_l4t bash -lc "source /my_ws/l4t/l4t_setup.sh; \
  source /my_ws/l4t/lg_walker_ws/install/setup.bash; \
  exec ros2 launch lgw_3d_tracker unified_track_node.launch.py \
    trace_log_dir:=/my_ws/l4t/lg_walker_ws/.run trace_tag:=_fix"

# 离线分析
python3 l4t/lg_walker_ws/scripts/analyze_tracker_trace.py \
  l4t/lg_walker_ws/.run/lgw_3d_tracker_<ts>_fix.jsonl

# 重复检测探针(只读):打印一帧内 >=2 条检测的 IoU/IoM
docker exec self_l4t bash -lc "source /my_ws/l4t/l4t_setup.sh >/dev/null; \
  python3 /my_ws/l4t/lg_walker_ws/.run/probe_dup.py both 60"
```

---

## 5.5 2026-10-26 真机验证(head 单通道,场景:木块只在 head 视野)

机器重启后重跑。场景:head 看到木块,arm 只看到纸箱(木块不在 arm 视野)。
运行 133.7 s / 1355 tick @ 10.1 Hz,日志
`.run/lgw_3d_tracker_20261026_092831_fix2.jsonl`。

### 新发现的第三类重复:同一物体被拆成两个"面"

YOLOE 把一个木块拆成**顶面 + 正面**两个实例:两条 bbox 上下相邻但不包含
(IoU 0.03、IoM 0.06),3D 中心间距 **p50 = 3.03 cm**(97.3% ≤ 3.5 cm)。
既有的 containment 去重(2D/IoM)**管不到**,3D 去重阈值 3 cm 也只覆盖 11%。

处理:
1. `duplicate_track_distance_m` 3 cm → **4 cm**(木块宽 ~4.8 cm,两个真实同类
   木块中心距 ≥ 4.8 cm,仍不会误合并);
2. 新增**建轨前抑制**:未匹配观测若落在既有轨迹的合并半径内,直接丢弃
   (`AssociationReport.suppressed`),不再"每 tick 新建一条再删一条"。

### 验证结果

| 指标 | 修复前(10-23 run2) | 本轮(10-26) |
|---|---|---|
| head 轨迹数 | 2(两个面各一条)+ 影子 | **1**(顶面/正面合并) |
| 同帧近邻观测抑制 | — | 625 条,占 46% 的 tick |
| `created` 轨迹 | 6 | 8(其中 6 条是 arm 纸箱的 1-hit 误检) |
| coasting 占比 | **36%** | id=1 **0.1%**、id=3 **0.3%** |
| 主 track_id | 1 → 4(**~155s 切换**) | **全程不变**(id=1 679 hits / id=3 677 hits) |
| 输出轨迹数 | 2~3,含影子 | 稳定 2(木块 + 纸箱),无影子 |

arm 的 id=3 是**纸箱误检**,与 head 的木块相距 20 cm(z=-0.06 vs +0.28),
未被误合并 —— 说明 4 cm 阈值对真实不同目标仍然安全。

**跨相机合并本轮未验证**:arm 视野里没有木块。

### 5.6 2026-10-26 跨相机合并验证(单木块,两相机均可见)

169.4 s / 1884 tick @ 11.1 Hz,日志
`.run/lgw_3d_tracker_20261026_094004_cross.jsonl`。

| 指标 | 结果 |
|---|---|
| 输出轨迹 | **1 条**,`id=1`,`source_cameras=[head, arm]`,全程不变 |
| `created` 轨迹 | **1**(仅启动时的 id=1),无影子、无 ID 切换 |
| 关联 | head 1349 条 + arm 942 条观测,**全部配到 id=1** |
| 同帧近邻抑制 | 406 条(22% 的 tick),顶面/正面拆分未再产生第二轨迹 |
| track 级去重 | 0 次(建轨前抑制已覆盖) |
| status | `confirmed` 99.9%,无 coasting 抖动 |
| 关联距离 | p50 3.5 cm(head 3.6 / arm 3.5),max 4.4 cm(< 门控 15 cm) |
| **跨相机位置一致性** | 4892 对(≤300 ms),p50 **4.37 cm**、max 4.46 cm,**100% ≤ 5 cm** |

**结论:单个目标在两相机下稳定合并为同一条 3D 轨迹,算法通过。**

遗留:head 与 arm 对同一木块的 3D 估计存在 ~4.4 cm 的系统性偏差
(大于 4 cm 合并半径但小于 15 cm 关联门控),不影响合并;若后续要做
抓取级精度,需要分别标定两个相机的 depth→3D 偏移。

### 5.7 2026-10-26 A/B:关闭 YOLOE 2D tracker

同一场景(单木块、两相机可见),只改 `inference.enable_2d_tracker`,
其余配置与算法完全一致。

| 指标 | 基线(2D on) | 本轮(2D **off**) | 变化 |
|---|---|---|---|
| 时长 / tick | 169.4 s @ 11.12 Hz | 217.2 s @ 11.14 Hz | — |
| head 观测 | 1349(**1.43 条/帧**) | 2164(**1.79 条/帧**) | **+25%** |
| arm 观测 | 942(1.00 条/帧) | 1210(1.00 条/帧) | 持平 |
| `created` 轨迹 | 1 | **1** | 无退化 |
| 关联距离 p50 | 3.53 cm | 3.24 cm | 略好 |
| coasting 占比 | 0.1% | 0.1% | 持平 |
| 同帧近邻抑制 | 406(22%) | 953(39%) | 观测变多 |
| 跨相机一致性 p50 | 4.37 cm | **3.99 cm** | 略好 |
| 跨相机 ≤5 cm | 100% | 100% | 持平 |

**结论:关闭 2D tracker 在所有维度上不劣于、部分优于基线**,主要是 head
出检率提升 25%(2D tracker 会丢掉只出现一帧的检测)。`created` 仍为 1,
说明新增检测没有变成误轨迹 —— 检测端的 containment 去重 + 3D 端建轨前
抑制已经兜住了重复。

**决定:`enable_2d_tracker: false` 保留。**

日志:`.run/lgw_3d_tracker_20261026_095722_no2d.jsonl`。

## 6. 未完成事项(下一步)

1. **重启 tracker** 加载第 1/3 层修复(当前 PID 9473 是旧代码):
   `docker exec self_l4t kill -15 9363`(按 PID,不要 pkill)。
2. **恢复单目标场景**,再验证:
   - 影子轨迹:`created` 全程 = 1、`dedupe_removed` 基本为空;
   - `track_id` 全程不变;
   - coasting 占比 < 2%(对照 run2 的 36%)。
3. **第 2 层 A/B**:先确认场景里能复现包含关系的双检测(探针 `multi > 0`),
   再对比 `containment_dedup_enabled: true/false`;通过后保留 true。
4. **第 4 层灰度**:把 `enable_2d_tracker` 改 false → 重启 YOLOE → 对比
   - head 每帧出检率(当前 0%,需在单目标场景下重测)
   - `created tracks` / 影子轨迹数(不应增加)
   - tick 频率(应略升)
   若误检明显增多,回退为 true 并保留 2D tracker 作噪声过滤器。
5. 更新 `docs/2026-10-23-lgw-3d-tracker-crosscam-test.md` 的结论与改动清单。

---

## 7. 回退方式

| 层 | 回退 |
|---|---|
| 第 1 层 | `runtime.yaml: coast_enter_ms: 0`(恢复"一个空 tick 就翻") |
| 第 2 层 | `runtime.yaml: containment_dedup_enabled: false` |
| 第 3 层 | `runtime.yaml: duplicate_track_distance_m: 0` |
| 第 4 层 | `runtime.yaml: enable_2d_tracker: true` |

全部是配置项,无需改代码;Python 包是 symlink/egg-link 安装,**改完重启节点即可**
(3D tracker 重启节点,YOLOE 侧重启 host 上的 `dual_camera_foxy.launch.py`)。
