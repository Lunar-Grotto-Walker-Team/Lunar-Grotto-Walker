# 容器环境缓存:把每次 7.3 秒的 source 变成 4 毫秒

- 日期:2026-10-31
- 工具:`scripts/lgw_env.sh`
- 缓存位置:**`/my_ws/l4t/.l4t_env.sh`**(容器内;宿主路径 `l4t/.l4t_env.sh`)

## 1. 问题

`docker exec` 每次都是一个全新的 shell,没有环境继承。所以每条命令都要:

```bash
source /my_ws/l4t/l4t_setup.sh
```

实测:**7.27 s**(user 4.97 s + sys 2.16 s)。原因不是脚本本身写得慢,而是它要串
**六个**工作空间的 `setup.bash`:

```
/opt/ros/humble
l4t/unitree_ros2
l4t/agx_arm_ws
l4t/ros2_ws
l4t/bt_ros2_ws
l4t/lg_walker_ws
l4t/front_camera_ws
```

每次都要扫一遍各工作空间的 `share/ament_index` 和 `local_setup.*`,叠起来就是 7 秒。
开发循环里(改代码 → 构建 → 跑测试)每条命令都要付一遍。

## 2. 办法:缓存结果,而不是重算过程

`l4t_setup.sh` 的最终产物就是一组环境变量。把 `export -p` 的输出存下来直接 source:

```bash
# 容器内,一次
source /my_ws/l4t/l4t_setup.sh && export -p > /my_ws/l4t/.l4t_env.sh

# 之后每条命令
source /my_ws/l4t/.l4t_env.sh          # 0.004 s
```

缓存文件 26 行,其中 24 个是真正有意义的变量:

```
AMENT_PREFIX_PATH  CMAKE_PREFIX_PATH  COLCON_PREFIX_PATH  LD_LIBRARY_PATH  PYTHONPATH
PATH  RMW_IMPLEMENTATION  ROS_DOMAIN_ID  CYCLONEDDS_URI  ROS_DISTRO  ROS_VERSION ...
```

## 3. 实测收益

| 场景 | 旧(source l4t_setup.sh) | 新(source 缓存) |
|---|---|---|
| source 本身 | 7.27 s | **0.004 s** |
| 跑一遍 12 个 gtest(含 source) | 7.66 s | **0.46 s** |
| `colcon build`(空转) | — | 2.78 s |

## 4. 验证矩阵(2026-10-31 实测)

缓存环境与 `l4t_setup.sh` 等价,逐项验过:

| 用法 | 结果 |
|---|---|
| `ros2 pkg prefix lgw_bt_planner` | ✅ 解析到 install 路径 |
| `python3 -c "from lgw_bt_planner.msg import ServoState"` | ✅ 自定义消息可导入 |
| `ros2 pkg prefix kdl_parser` | ✅ |
| `RMW_IMPLEMENTATION` / `ROS_DOMAIN_ID` / `CYCLONEDDS_URI` | ✅ `rmw_cyclonedds_cpp` / `0` / eth10 |
| `colcon build --packages-select lgw_bt_planner` | ✅ `1 package finished` |
| **`ros2 launch lgw_full.launch.py`(整套 mock 栈)** | ✅ 节点全起,`/control/joint_states` 有数据 |
| gtest 二进制 | ✅ 12 passed |

## 5. 什么时候必须刷新

- **安装了新包**(`AMENT_PREFIX_PATH` / `CMAKE_PREFIX_PATH` 里写死了各 `install/` 路径,
  新包不在其中)→ 必须 `--refresh`;
- **只重新构建已有包** → 不需要,路径不变;
- **容器重启** → 缓存文件在容器内,需要重新生成(容器重启不会删除 bind-mount 下的
  文件,但本文件在 `/my_ws/l4t/` 下属于 bind-mount,所以实际上**容器重启后仍在**——
  只有宿主机删文件才会丢)。

## 6. 已知限制

- 只缓存**导出的变量**。如果将来某个 `setup.bash` 开始定义 shell **函数或别名**,
  `export -p` 抓不到,需要改用别的方式(目前 24 个全是 `declare -x`,没有函数)。
- `PWD` / `OLDPWD` / `SHLVL` 这类 shell 变量也在文件里,无害。

## 7. 用法

```bash
bash scripts/lgw_env.sh --refresh    # 重新生成(~7 s)
bash scripts/lgw_env.sh --show       # 看缓存了哪些变量

# 之后任意命令
docker exec -w /my_ws/l4t/lg_walker_ws self_l4t bash -c \
  "source /my_ws/l4t/.l4t_env.sh && colcon build --packages-select lgw_bt_planner"
```

> 注意:CLAUDE.md 里"ros2 节点启动前必须先 source l4t_setup.sh"这条要求的**意图**是
> 拿到正确的环境;本缓存提供的是同一份环境(逐项验证见 §4),因此等价满足。
