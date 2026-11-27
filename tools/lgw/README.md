# tools/lgw —— 行为树启动与验证脚本

这些脚本原本位于独立的 `lg_walker_ws` 工作区，随行为树一起迁入本仓库。
它们**在宿主机执行**，通过 `docker exec` 把命令送进容器 `self_l4t` 里跑。

```
tools/lgw/
├── _common.sh        # 公共路径：WS_FORK_HOST / WS_FORK_CT / CONTAINER / vendor
├── start/            # 启动/停止链路、触发行为树（日常使用）
├── runtime/          # 运行时辅助：odom_tf_bridge.py
├── bench/            # bench_cleanup、hold-recovery 验证、跟随时延测量、假目标发布
├── record/           # 数据录制：相机 bag、TCP 位姿、Servo 轨迹
├── diag/             # 诊断：servo 状态监视、零指令/恢复播种检查、跟踪轨迹分析
├── agx/              # 机械臂侧一次性工具：CPV 能力探测、夹爪标定/直连测试
├── config/           # DDS(lo) 与 perception_bridge 配置、示例命令、TCP 航点
└── test/             # 感知与跟踪的端到端脚本（需要 YOLOE 部署）
```

> 只收录**运行、验证与排查需要**的脚本。早期的一次性调试脚本（`_diag_*.py`、
> `lookat_*_sweep.py`、`jacobian_*`、`fk_*`、各类 `record_*` 变体、`.run/` 探针等）
> 没有迁入——它们是 AI 调试过程的中间产物，设计结论已经沉淀在
> [`docs/bt/`](../../docs/bt/README.md) 与 `docs/bt/history/`。

## 路径约定

`start/*.sh` 与 `run_bt.sh` 都 `source _common.sh`，不再各自写死工作区路径：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `WS_FORK_HOST` | 仓库根（自动推导） | 宿主机上的仓库路径 |
| `WS_FORK_CT` | `WS_FORK_HOST` 按挂载根改写；否则 `/my_ws/l4t/Lunar-Grotto-Walker` | 容器内仓库路径 |
| `HOST_MOUNT` / `CT_MOUNT` | `/home/unitree/my_ws` / `/my_ws` | 宿主机 ↔ 容器挂载根 |
| `CONTAINER` | `self_l4t` | 容器名 |
| `VENDOR_ROOT_CT` | `${CT_MOUNT}/l4t/Lunar-Grotto-Walker-Vendor` | vendor 工作区在容器内的路径 |
| `AGX_CAN_ACTIVATE_CT` | `${VENDOR_ROOT_CT}/src/agx_arm_ros/scripts/can_activate.sh` | CAN 激活脚本（随 vendor 源码提供，不安装到 install/） |
| `L4T_ENV_CT` | `/my_ws/l4t/.l4t_env.sh` | 容器内的环境缓存（属于容器环境，不在本仓库） |

仓库不在默认位置时：

```bash
WS_FORK_CT=/my_ws/l4t/Lunar-Grotto-Walker \
HOST_MOUNT=/home/unitree/my_ws CT_MOUNT=/my_ws \
  bash tools/lgw/start/start_lgw_chain.sh
```

> `l4t_setup.sh` / `.l4t_env.sh` 属于容器环境本身（不在本仓库），仍按
> `/my_ws/l4t/...` 引用。

## 典型顺序

```bash
# 1) 机械臂驱动（CAN + agx_arm_ctrl）——宿主机另开终端
bash tools/lgw/start/start_lgw_arm.sh --driver-only

# 2) 感知（可选，需要 YOLOE 部署与两路相机）
bash tools/lgw/start/start_lgw_yoloe.sh

# 3) 主链路：MoveIt + Servo + tracker + 可选 BT
bash tools/lgw/start/start_lgw_chain.sh use_bt:=true follow:=true

# 4) 触发一棵树
bash tools/lgw/start/run_bt.sh TrackCollectReactive
```

## 说明

- 脚本里的日志写到仓库内 `.run/`（已 gitignore）；
- `bench_cleanup.sh` 按 PID 逐个 `kill -15`，**不要**改成 `pkill -f ros`；
- 更早的一次性诊断脚本（`_diag_*.py`、`lookat_*_sweep.py` 等）暂未迁入，
  需要时从历史工作区取。
