# ROS2 工作空间

本目录用于放置 ROS2 功能包。

> **clone 下来是空的几个位置**：`point_lio_ros2/` 来自
> [Lunar-Grotto-Walker-PointLIO](https://github.com/Lunar-Grotto-Walker-Team/Lunar-Grotto-Walker-PointLIO)
> （GPL-2.0，单独仓库），`livox_ros_driver2/`、`unitree_ros2/` 等来自 vendor 仓库。
> 都要先 `vcs import src < deps.repos` 才有，见 [依赖说明](../docs/dependencies.md)。

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
