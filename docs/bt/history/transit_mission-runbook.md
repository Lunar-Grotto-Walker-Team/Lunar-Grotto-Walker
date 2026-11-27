# TransitMission 现场操作手册

摆渡任务：狗从**树启动那一刻的位置**出发，走到局部坐标 `(X, Y)`，再把朝向摆回那一刻的朝向。

---

## 1. 前置

- 容器 `self_l4t` 在跑。
- 导航链环境缓存存在：`/my_ws/l4t/.l4t_nav_env.sh`
  （没有就先生成：`bash /my_ws/l4t/nav_ws/scripts/lgw_nav_env.sh --refresh`）。
- `go2_transit` 与 `lgw_bt_planner` 都已 `colcon build` 过。
- **狗在开阔地**，遥控器在手。

## 2. 启动

```bash
# ① 起 A+C（point_lio + Nav2）
#
#    B（elevmap 地形链）**不需要**：摆渡只走 local costmap + FollowPath/Spin，
#    不碰 /terrain_traversability 和全局代价地图。start_chain.sh 会连 B 一起起，
#    A+B+C 太慢，现场手工分开起。
bash /my_ws/l4t/nav_ws/scripts/realstart/start_chain.sh

# ② 用遥控器让狗站起来（本任务树不做 StandUp/BalanceStand）

# ③ 起 transit_node + bt_runner
bash /my_ws/l4t/lg_walker_ws/scripts/start_transit_mission.sh
```

> **动作名默认已错开**（2026-11-27 起）：本 launch 的 `action_name` 默认是
> `/transit_bt_execution`，机械臂链的 `bt_runner` 仍是 `/lgw_bt_execution`，
> 两套可以同时在线，`send_goal` 不会随机落到**没加载 `go2_transit` 插件**的那一个。
> 只有在旧脚本/旧文档里还写着 `action_name:=lgw_bt_execution` 时才需要改回来。
>
> ⚠️ 但 **`transit_node` 本身仍然只能有一个**：`lgw_chain.launch.py` 在
> `use_transit:=true`（默认）时会自己起一个 `transit_node`，此时再跑本 launch
> 就有两个同名节点、两个 `/go2_transit/transit_to_pose` 服务器。那种情况要么给
> 机械臂链传 `use_transit:=false`，要么不要单独起本 launch。
>
> 另外：BT.CPP 的 Groot2 调试器会**绑定 1667/1668 端口**，机械臂链的 `bt_runner`
> 默认占着它们。本 launch 已经把 `groot2_port` 默认改成 **1669** 错开；如果还是报
> `Address already in use`，换一个没被占用的端口：`groot2_port:=1671`。

## 3. 填坐标

编辑 `src/lgw_bt_planner/bt_trees/transit_mission.xml` 的这一行：

```xml
<SetPoseStamped name="set_goal" output_key="goal_local"
                value="local;0.5;0.0;0.0;0;0;0;1"/>
```

> 2026-11-27 起仓库里默认就是 `0.5;0.0`（近距离实测用），现场洞口坐标
> `2.43;0.52` 写在文件注释里，验证完改回去。

- `local` 固定不动（节点只认这个 frame，写别的会直接 FAILURE）。
- 第 2 个数 = **X**：往正前方多少米。
- 第 3 个数 = **Y**：往**左**多少米（ROS `base_link` 约定，往右写负数）。
  洞口在左前方就是 `(X 正, Y 正)`。
- 后面 `0.0;0;0;0;1` 是 z 与四元数，固定不动。

**改完必须 `colcon build --packages-select lgw_bt_planner`**（约 2 s）：
`install/` 下是**拷贝安装**，不是 symlink-install，改 XML 不会即时生效
（2026-11-27 实测确认）。或者直接改 `install/.../bt_trees/` 下的副本，
也要**重启第 3 步的节点**，树才会重新加载。

## 4. 执行

**另开一个终端**（第 3 步的 launch 会占住当前终端），并先 source 环境——
容器里普通的 `bash -c` / 登录 shell 都没有 `ros2`：

```bash
source /my_ws/l4t/.l4t_nav_env.sh
ros2 action send_goal /transit_bt_execution btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: TransitMission}" --feedback
```

成功返回 `Tree finished with status: SUCCESS`；狗停在目标点、朝向与启动时一致。
到点判据是 **0.20 m**（三处一致：`cave_transit.yaml` 的
`default_xy_tolerance`、`nav2_params.yaml` 的
`transit_goal_checker.xy_goal_tolerance`、`FollowPath.xy_goal_tolerance`），
比探索用的 `general_goal_checker`（0.25 m）略严。
0.15 m 已在真机实测**不可达**：狗稳定停在 0.17~0.19，判不到点。

树内部最多试 **3 次**（首次 + 2 次重试）。

## 5. 中止

```bash
bash /my_ws/l4t/nav_ws/go2-nav2-terrain-pointlio/scripts/stop_robot.sh
```

它按这个顺序做：

1. **取消行为树的 goal**（`/lgw_bt_execution`、`/transit_bt_execution`、`/bt_execution`）；
2. 取消 nav2 的在飞 goal（`/follow_path`、`/spin`、`/navigate_to_pose`）；
3. 补发几拍 `Move(0,0,0)`（**不用 StopMove**——那会让固件丢弃经典步态）；
4. 回读实测速度。

**第 1 步不能省。** 只取消 nav2 的 goal 时，`FollowPath` 会以 `CANCELED` 返回，
树里的 `RetryUntilSuccessful` 把它当成本次失败并**立刻重发一条新 goal** ——
表现就是"急停后半秒，狗又走了"。（2026-11-18 复审抓到，已修进脚本。）

遥控器趴下/阻尼优先级最高，任何时候都能直接用。

## 6. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 树立刻 FAILURE | `frame_id` 不是 `local` | 改回 `local` |
| 树等 30 s 后 FAILURE，日志「TF/动作服务器未就绪」 | A+B+C 没起或没起全 | `start_chain.sh --status` 检查；`ros2 run tf2_ros tf2_echo odom base_footprint` |
| `send_goal` 报「Node not recognized: TransitToLocalPoint」 | goal 发到了 `/lgw_bt_execution`（机械臂那个 bt_runner 没有 go2_transit 插件） | 改发 `/transit_bt_execution` |
| 狗走到一半停下、树报 `unreachable` | 导航失败（障碍/局部极小值），或到点判据比实际停位精度更严 | 树会自动重试 2 次；仍失败先看 `final_xy_error`，若稳定卡在判据附近，把**三处** tolerance 一起放宽（0.20 → 0.25） |
| 日志 `Failed to make progress` ×1 后 20 s abort | Nav2 `SimpleProgressChecker`（20 s 内没累计移动 0.5 m），通常说明狗已停在判据外几厘米、DWB 不再输出平移 | 先确认判据是否过严；要定位是控制器没出速度还是狗没执行，用 `start_chain.sh --record` 录 `/cmd_vel_nav` |
| 实际到点偏离明显 | 局部原点被记在"狗被推动之后" | 树启动前不要动狗；原点只在第一次 tick 捕获一次 |

## 7. 限制

- 树里不做站/卧，也不做绕障。
- 局部原点在**第一个"TF 与动作服务器都就绪"的 tick**捕获（冷启动慢的时候可能比
  "树启动"晚几秒），之后不再重采。改完坐标重启节点时，狗不要在这段窗口里被推动。
- 目标精度受 nav2 `xy_goal_tolerance`（0.25 m）限制：`(0.3, 0)` 这种小距离基本不会真的走。
