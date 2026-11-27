# YOLOE 双目检测 E2E 调试记录

**日期**: 2026-09-28
**范围**: `lgw_bt_planner` Task 5 smoke test(检测条件 BT 节点真机 YOLOE 联调)
**状态**: 部分通过(head/arm CONFIRMED 率波动较大,需更多迭代)

## 背景

`IsHeadDetectionConfirmed` / `IsArmDetectionConfirmed` BT 节点订阅
`/perception/two_stage/camera_{head,arm}/result`,根据 YOLOE
`ObservationResult` 的 `status == CONFIRMED` 决定 SUCCESS/FAILURE。
Tasks 1-4 已经完成(evaluate_detection + gtest + BT 类 + plugin + demo XML)。
本记录覆盖 Task 5 真机联调发现的问题。

## 硬件拓扑

| 相机 | 接口 | 用途 | 备注 |
|---|---|---|---|
| D435 head | USB 3.0 | stage1(候选检测) | 1280x720x15 |
| D435 arm | USB 2.1 | stage2(确认) | **降到** 640x480x15 |

## 配置变更历史

| 时间 | 配置 | 现象 |
|---|---|---|
| 初始 | 两相机都 1280x720x15 | YOLOE 启动 2.5s 内 color 流死亡,CONFIRMED 几乎为 0 |
| Plan A(已采纳) | head 1280x720x15, **arm 640x480x15** | color 流稳定 |

### 为什么降 arm 而不是 head

- USB 2.1 实际带宽 ~280 Mbps(理论 480),单 D435 全 profile ~417 Mbps
- 降 arm 不需要重新标定手眼矩阵(`camera_optical_frame → gripper` 与分辨率无关)
- YOLOE 运行时从 `camera_info.k[0]/k[4]` 读 fx/fy,无硬编码内参

## 三类问题

### 问题 1:daemon RMW drift(已解决)

**症状**: 长时间运行后,host `ros2 daemon` 被 `docker exec ros2 topic hz` 子 shell
触发自动重启时,因为子 shell 没 source `l4t_setup.sh`,daemon 用了默认
`rmw_fastrtps_cpp`,而不是项目要求的 `rmw_cyclonedds_cpp`。

**后果**: daemon 看不到 cyclonedds 上的 topic,后续 `ros2 topic list` / 节点 introspection 全盲。

**修复**:
- 完全清理进程(容器 + host)+ SIGKILL daemon
- 让 daemon 下次自然重启时从 `l4t_setup.sh` 里读 RMW
- `test_yoloe_stability.sh` 里 `docker exec` 仍只 source `setup.bash` + `ros2_ws/setup.bash`,
  不 source `l4t_setup.sh` — **待修**

**验证 daemon RMW**:
```bash
ps -eo args | grep ros2cli.daemonize | grep -oE 'rmw_[a-z]+_cpp'
# 应该: rmw_cyclonedds_cpp
# 错了: rmw_fastrtps_cpp
```

### 问题 2:首帧后 DDS 大消息卡死(已解决)

**症状**: YOLOE 启动后,`STREAM_RX` 只拿到每个流的第 1 帧(count=1),之后
`CAMERA_CYCLE_SKIPPED` 的 `color_age/depth_age` 累积到 100s+,YOLOE 再也收不到新帧。

**对比**: 同套硬件、同样配置,之前能稳定 5+ 分钟累积 8900 帧 / 299 RX 事件。

**修复**: SIGKILL host daemon + 容器内 daemon,清完残留进程后重跑,YOLOE 立刻恢复正常。

**根因猜测**: CycloneDDS participant / fragment 重组队列在 daemon 长时间运行 + 大流量下
进入坏状态,无自动恢复路径。

### 问题 3:Per-camera CONFIRMED 率波动(未解决)

**两轮 30s 测试对比**:

|  | Run 1 | Run 2 |
|---|---|---|
| head CONFIRMED |85/124 (68.5%) | **0/111 (0%)** |
| head NO_TARGET | 18 | 60 |
| head TIMEOUT | 1 | 51 |
| arm CONFIRMED | 56/134 (41.8%) | 84/111 (75.7%) |
| arm NO_TARGET | 33 | 27 |
| arm TIMEOUT | 13 | 0 |
| 总 CONFIRMED | **141** | 84 |

**观察**:
- 两次结果几乎完全反转
- Run 2 head 几乎全坏(0 CONFIRMED,51 TIMEOUT)
- 总识别率从 141 降到 84(-40%)

**根因猜测**(未核实):
1. **目标物在视野里移动** — head 1280×720 全分辨率下,移动物体 blur 严重,直接 TIMEOUT
2. **两相机视野不重合** — 同一物体在某一时刻只在一个相机的 FOV 里
3. **光照变化** — 自然光下两相机角度不同,反射 / 阴影不一致
4. **stage1 / stage2 prompt 差异** — `runtime.yaml` 里 head 用 hardcoded prompts,
   arm 用 visual prompts,匹配难度不同

## BT 实际表现

**`detection_condition_test.xml`**(测试树):
```xml
<RetryUntilSuccessful num_attempts="100">
  <Sequence name="both_cameras_confirmed">
    <IsHeadDetectionConfirmed ... max_age_sec="2.5" />
    <IsArmDetectionConfirmed  ... max_age_sec="2.5" />
  </Sequence>
</RetryUntilSuccessful>
```

**RetryUntilSuccessful 备注**: BT.CPP v4 里 retry 节点 XML ID 是
`RetryUntilSuccessful`(对应 C++ 类 `RetryNode`),不是字面 `RetryNode`。
源:`BehaviorTree.CPP/src/bt_factory.cpp:138`。

**期望窗口**: num_attempts=100 × 0.1s tick ≈ 10s 窗口。
按 ~4Hz RUN + 1.4 Hz YOLOE result,理论上窗口里每相机都有 ≥10 条消息。
只要两路 result topic 同时有 CONFIRMED,Sequence 立刻 SUCCESS。

**最坏情况**: 出现 Run 2 那种 head 完全 0 的场景,BT 必须等 head 恢复才能走通。
RetryUntilSuccessful 等于死等,直到 num_attempts 耗尽 → FAILURE。

## 已知未做的事

- [ ] `test_yoloe_stability.sh` 里 `docker exec` 加上 `source l4t_setup.sh`,
      防止 daemon RMW drift(问题 1 复现)
- [ ] 多轮静态场景下识别率统计(目标不动,跑 3 轮以上)
- [ ] 确认 head 是否需要在 USB 3.0 上也降到 640x480x15(同 arm 对称)
- [ ] BT SUCCESS 路径真机验证(目前只验证了 YOLOE 数据通路,没真跑 BT action)

## 关键文件

| 路径 | 角色 |
|---|---|
| `test/test_yoloe_stability.sh` | 相机+YOLOE 诊断(已缩短到 30s 监控 + 5s 间隔) |
| `test/test_yoloe_e2e.sh` | 完整 BT E2E(含 BT action) |
| `src/lgw_bt_planner/src/evaluate_detection.cpp` | CONFIRMED + non-empty 检测逻辑 |
| `src/lgw_bt_planner/src/is_detection_confirmed_node.cpp` | BT leaf 节点 |
| `src/lgw_bt_planner/bt_trees/detection_condition_test.xml` | 测试树 |
| `/home/unitree/yoloe_deploy/config/runtime.yaml` | YOLOE stage1/2 配置 |
| `/home/unitree/yoloe_deploy/config/stage1_prompts.yaml` | head 用的 prompts(hardcoded) |
| `/home/unitree/yoloe_deploy/config/stage2_prompts.yaml` | arm 用的 prompts |

## 复现问题 1/2 的最小步骤

```bash
# host + container 清理
docker exec self_l4t bash -c "ps -eo pid,args | grep ros2cli | grep -v grep | awk '{print \$1}'" \
    | xargs -I{} docker exec self_l4t kill -9 {}

# 长跑 5 分钟触发 daemon 大流量坏状态
bash test_yoloe_stability.sh 300

# 之后任何 `ros2 topic hz` 触发 daemon 重启 → cyclonedds 主题看不见
```

## 复现问题 3 的最小步骤

```bash
# 静态场景下两轮对比
bash test_yoloe_stability.sh 30   # Run 1
# (手动清理)
bash test_yoloe_stability.sh 30   # Run 2

# 然后:
grep 'accepted=' /tmp/test_yoloe_stability_yoloe.log | awk -F'accepted=' '{print $2}' | awk '{print $1}' | sort | uniq -c
# head 0/111 vs 85/124 这种反转 = 问题 3
```