# tracker 时间轴校正(Plan A)+ 缺帧 lookup 快速失败

**日期**:2026-10-23
**范围**:`lgw_3d_tracker`(`odom_tf_broadcaster.py` / `input_adapter.py` / `unified_track_node.py`)
**状态**:已实现并在真机全栈验证(YOLOE 双相机 + 机械臂 + tracker)

---

## 1. 问题一:缺帧 lookup 阻塞执行器,把 TF 从 19 Hz 拖到 3.9 Hz

### 现象

在机械臂未启动(或 `frame_prefix` 配错)的条件下:

```
/utlidar/robot_pose        = 18.8 Hz
/tf (odom→base_link)       =  3.9 Hz      ← 丢 79%
/perception/tracks/unified =  0   Hz
```

日志里满屏:
```
TF lookup base_link←arm1/base_link failed at both stamp and latest:
"arm1/base_link" passed to lookupTransform argument source_frame does not exist. ; skip batch
```

### 根因

`InputAdapter._lookup` 直接调 `lookup_transform(..., timeout=0.05)`。**tf2 对"帧不存在"
这种确定性失败也会完整等满 timeout**。实测:

| 操作 | 耗时 |
|---|---|
| 缺失帧 `lookup_transform(timeout=0.05)` | **62.7 ms** |
| 缺失帧 `can_transform(timeout=0)` | **0.0 ms** |
| 存在帧 `lookup_transform(timeout=0.05)` | 0.2 ms |

旧代码对每个缺失帧做两次(检测 stamp + latest)≈ **125 ms**;arm 相机换算需要 2 段,
两个相机各 ~5.9 Hz → 单线程执行器被占满 → pose 回调排不上队 →
`odom→base_link` 掉到 3.9 Hz,`tracks` 归零。

### 修复

`_lookup` 改为**先零超时探测,再查**:

```python
zero = Duration(seconds=0.0)
if not buf.can_transform(target, source, latest, timeout=zero):
    return None                                   # 确定性失败,立即返回
if buf.can_transform(target, source, t_stamp, timeout=zero):
    return transform_to_matrix(buf.lookup_transform(..., timeout=zero))
try:                                              # 只有"数据还没到"才等一小会
    return transform_to_matrix(buf.lookup_transform(..., timeout=self._timeout))
except Exception:
    return None
```

同时 `lookup_timeout_sec` 默认值 **0.05 → 0.01**(19 Hz 位姿半周期才 26 ms,10 ms 足够)。

### 验证

```
head 相机(需缺失帧): 0.299 ms/次    (旧代码 ~125 ms)
arm  相机(需缺失帧): 0.266 ms/次
```

arm 链齐全时 TF 稳定 18.8 Hz;CPU 从 20.1% 降到 16.8%。

---

## 2. 问题二:TF 时间轴用的是"回调时刻",不是"测量时刻"

### 背景

Go2 内置主机没有对时,其 `/utlidar/*` 的 stamp 与 Jetson 本地时钟相差一个**固定偏移**
(实测 10 s 内漂移 0.000 s;数值会随主机重启/NTP 步进而变,见 §2.4)。

旧代码的处理是:

```python
if drift_sec > 1.0:
    t.header.stamp = self._node.get_clock().now().to_msg()   # 丢弃原 stamp
```

这会让 TF 的时间轴变成**回调处理时刻**,而不是位姿被测量的时刻。带来的后果:

- 位姿被"贴晚"了一个传输+排队延迟;
- 时间轴间隔被回调调度抖动污染(而不是 Go2 稳定的 53 ms 节拍);
- 按检测 stamp 插值时,取到的是"更早"的位姿。

### Plan A:偏移估计 + 校正

```python
offset = min over 窗口(now_local - remote_stamp)     # 延迟恒为正,最小值最接近真值
corrected_stamp = remote_stamp + offset              # 测量时刻(本地时基)
```

实现要点(`odom_tf_broadcaster.py`):

| 设计 | 理由 |
|---|---|
| 滑动窗口 **100 帧(≈5.3 s)** 取最小值 | 自动跟踪慢漂移;无需人工填常量。实测 100 与 200 帧估计值完全相同(差值 0.000 ms),但短窗口让跳变/坏样本的恢复时间从 10.7 s 减半到 5.3 s |
| 冷启动 `MIN_SAMPLES=20` 前用当前样本 | 退化为旧行为,不会突跳 |
| 最小值跳变 > 5 s 判为时钟跳变,清窗重估 | 机器人重启 / NTP 对时 |
| 校正结果不在 `(now-2s, now+100ms]` 就退回 `now()` | 防止估计异常把 stamp 打到被 buffer 淘汰的位置 |
| 每 100 帧打印一次 offset | 现场可核对 |

### 验证(真机全栈)

| 指标 | 结果 |
|---|---|
| 原始 SLAM 采样间隔 | p50 **53.3 ms**,std 0.9 ms |
| 校正后 TF 间隔 | p50 **53.3 ms**,std **0.9 ms**(完全一致) |
| `now - tf stamp` | p50 **2.8 ms**,p95 7.5 ms(只剩传输延迟) |
| 按 detection stamp 插值命中率 | **87% → 98%** |
| tracks 输出 | ~11 Hz 不变 |

### 2.4 为什么不能写死 659.509 s

早先测得偏移 659.509 s;主机在 15:13 重启、15:16 `systemd-timesyncd` 初始同步后,
偏移变成 **661.17 s**。差别 1.66 s 来自一次性的时钟步进。
固定常量会在重启后失效,滑动最小值会自动收敛(实测 3 s 内稳定到 661.173 s,
之后 112 s 内仅变化 0.004 s ≈ 36 ppm)。

---

## 3. 改动清单

| 文件 | 改动 |
|---|---|
| `unified_track/odom_tf_broadcaster.py` | 重写:滑动窗口偏移估计 + 校正;删除 `STAMP_DRIFT_TOLERANCE_SEC` 的 now() 兜底 |
| `unified_track/input_adapter.py` | `_lookup` 加 `can_transform(timeout=0)` 快速失败;默认超时 0.05 → 0.01 |
| `unified_track_node.py` | 传给 InputAdapter 的 `lookup_timeout_sec` 0.05 → 0.01 |

Python 包是 `egg-link` develop 安装,**改完无需 colcon build**,重启节点即可。

---

## 4. 遗留

1. **其他消费者仍看到远端原始 stamp**。本次只修了 tracker 自己发布的
   `odom→base_link`。若有其它节点直接消费 `/utlidar/*` 并与本地时间戳做对齐,
   需要独立的 relay 节点(或各自实现同样的偏移估计)。
2. **偏移跳变检测阈值 5 s** 是经验值。若远端时钟以更大步长跳变(如整机对时),
   会走"清窗重估"分支,行为仍正确,但会在日志里告警一次。
3. **检测 stamp 落后 now 约 170–190 ms**(YOLOE 推理耗时)。这是链路固有的,
   Plan A 不改变它;命中率 98% 已经说明剩余 2% 是 TF 流停顿,不是时钟问题。
