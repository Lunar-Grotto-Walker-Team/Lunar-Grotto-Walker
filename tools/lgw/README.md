# tools/lgw —— 行为树启动与验证脚本

这些脚本原本位于独立的 `lg_walker_ws` 工作区，随行为树一起迁入本仓库。
它们**在宿主机执行**，通过 `docker exec` 把命令送进容器 `self_l4t` 里跑。

```
tools/lgw/
├── _common.sh        # 公共路径：WS_FORK_HOST / WS_FORK_CT / CONTAINER
├── start/            # 启动/停止链路、触发行为树
├── bench/            # bench_cleanup.sh（按 PID 清理，不用 pkill -f ros）
├── config/           # DDS(lo) 与 perception_bridge 配置、示例命令、TCP 航点
└── test/             # 感知链路端到端脚本（需要 YOLOE 部署）
```

## 路径约定

`start/*.sh` 与 `run_bt.sh` 都 `source _common.sh`，不再各自写死工作区路径：

| 变量 | 默认值 | 说明 |
|---|---|---|
| `WS_FORK_HOST` | 仓库根（自动推导） | 宿主机上的仓库路径 |
| `WS_FORK_CT` | `WS_FORK_HOST` 按挂载根改写；否则 `/my_ws/l4t/Lunar-Grotto-Walker` | 容器内仓库路径 |
| `HOST_MOUNT` / `CT_MOUNT` | `/home/unitree/my_ws` / `/my_ws` | 宿主机 ↔ 容器挂载根 |
| `CONTAINER` | `self_l4t` | 容器名 |
| `L4T_ENV_CT` | `/my_ws/l4t/.l4t_env.sh` | 容器内的环境缓存 |

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
