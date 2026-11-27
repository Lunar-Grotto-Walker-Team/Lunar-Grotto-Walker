# YOLOE 双目相机稳定性调试记录(Path A/C/D/E 修复)

**日期**: 2026-10-04
**范围**: `test_yoloe_stability.sh` 反复报"两路相机 missing"的根因定位与修复
**状态**: Round 2 修复完成,Path E(head 1280x720→640x480)落 src + install,md5 `48993e8b85f1b8a571670a1e675e8b34`,等真机回归

## 背景

YOLOE dual-camera perception pipeline 在同 session 内反复报"两路相机 missing":
- 第一次跑(arm=1280x720)稳定 ~3min
- 第二次跑 arm missing → `RS2_USB_STATUS_BUSY`
- 选 Path C 修 BUSY,跑第三次仍 missing(head 12s 撑过去,arm 没撑过去)
- 修 type error(SIGSEGV),跑第四次仍 missing

最终定位三层独立问题,组合修复 Path A+C+D。

## 硬件拓扑

| 相机 | 设备 | USB 拓扑 | sysfs 路径 | lsusb pid |
|---|---|---|---|---|
| head | D435i | USB 3.0 root hub 直挂 | `/sys/bus/usb/devices/2-1` | `0b3a` |
| arm | D435 | USB 2.0,经 Genesys Logic 4-port hub | `/sys/bus/usb/devices/1-3.3` | `0b07` |

arm 在 USB 2.0 hub 后面,本来就脆弱(带宽 ~280 Mbps,单 D435 全 profile ~417 Mbps)。

## 时间线 / 问题演进

| Run | 现象 | 错误信号 | 根因 |
|---|---|---|---|
| 1 | 通 | (无) | baseline 干净 |
| 2 | arm missing | `RS2_USB_STATUS_BUSY` → `falling back to partial device` → `uvc_get_ctrl(UVC_GET_MAX) failed` | kernel libusb handle 残留(本 session 之前的 run 死时留下) |
| 3(应用 Path C 后) | head+arm 都 missing | `Wrong parameter type ... wait_for_device_timeout is of type {double}, setting it to {integer} is not allowed` → exit -11 | launch_arguments 类型错(Path C 加 `"30"` 应是 `"30.0"`) |
| 4(修类型后) | 仍 missing | `RS2_USB_STATUS_BUSY` 仍出,但 head 12s 撑过去(`RealSense Node Is Up`),arm `failed to set power state` → `Error starting device` | Path C 不充分(只动 device 端,不动 kernel handle) |

## 三类独立问题

### 问题 1: launch_arguments 类型 coerce 失败(Path C 类型错)

**症状**: realsense_node_factory.cpp:436 抛异常,进程 SIGSEGV(exit code -11),
两相机在 `__init__` 阶段直接死,根本走不到 USB init。YOLOE 看到
`color_age=never depth_age=never` 持续 SKIPPED。

**根因**: Python ROS 2 launch 文件的 `launch_arguments={...}` dict value
**全是字符串**,rclpy 把 `"30"` parse 成 int,设给 `wait_for_device_timeout`
(declared as `double`)失败,类型不 coerce。

**修复**:
```diff
- "wait_for_device_timeout": "30"
+ "wait_for_device_timeout": "30.0"  # double type per rs_launch.py
```

**类型映射约定**:
| 参数类型 | 必须传的字符串 |
|---|---|
| `double` | `"30.0"`(不要 `"30"`) |
| `int` | `"30"` |
| `bool` | `"true"` / `"false"`(rs_launch 多接受) |
| `string` | 任意字符串 |



