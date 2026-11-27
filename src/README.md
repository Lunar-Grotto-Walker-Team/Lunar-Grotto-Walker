# ROS2 工作空间

本目录用于放置 ROS2 功能包。

> **`point_lio_ros2/` 不在仓库里**：它来自
> [Lunar-Grotto-Walker-PointLIO](https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-PointLIO)
> （GPL-2.0，单独仓库），`vcs import src < deps.repos` 之后才出现。
>
> 本目录里**不会**出现 `livox_ros_driver2/`、`unitree_ros2/` 等 vendor 包——
> 那些在独立的 vendor 工作区里编译，用 `source` 它的 `install/setup.bash` 引入。
> 见 [依赖说明](../docs/dependencies.md)。

## 工作空间构建

```bash
cd Lunar-Grotto-Walker
colcon build
source install/setup.bash
```

## 添加功能包

在 `src/` 目录下创建或克隆功能包：

```bash
# 创建新包
cd src
ros2 pkg create <package_name> --dependencies rclpy

# 克隆现有包
git clone <repo_url>
```

## 推荐的功能包结构

```
src/
├── my_robot_control/      # 机器人控制
├── my_navigation/         # 导航规划
├── my_perception/          # 感知算法
└── my_description/         # 机器人模型描述
```
