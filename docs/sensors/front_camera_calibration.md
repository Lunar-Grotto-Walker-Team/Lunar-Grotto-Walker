# Go2 前视相机内参标定 SOP

把 `config/front_camera_720.yaml` 里的占位内参换成真机实测值。
**做 SLAM / 投影之前必须标定**。

---

## 1. 准备

### 1.1 棋盘格规格

- **11 × 10 格子**(10 × 9 inner corners,inner corners 是内部角点数)
- **方格边长 20 mm** = 0.020 m
- 用哑光纸打印,**裱在硬板上**(硬纸板 / KT 板 / 亚克力都行)
- **不要用光面相纸**:反光会让角点检测失败

### 1.2 容器内一次性安装

```bash
docker exec self_l4t bash -c "
  apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-camera-calibration python3-yaml
"
```

### 1.3 确认驱动在跑 + 图像流通

启动 driver:

```bash
docker exec -w /my_ws/l4t/Lunar-Grotto-Walker self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash &&
  ros2 launch go2_front_camera_driver front_camera.launch.py
"
```

另一个终端确认 ~15 Hz:

```bash
docker exec self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash &&
  ros2 topic hz /camera/frontcamera/image_raw
"
```

---

## 2. 启动标定 GUI

```bash
docker exec -w /my_ws/l4t/Lunar-Grotto-Walker self_l4t bash -c "
  export DISPLAY=:1 &&
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/l4t_setup.sh &&
  source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash &&
  ros2 run camera_calibration cameracalibrator \
    --size 10x9 \
    --square 0.020 \
    --no-service-check \
    image:=/camera/frontcamera/image_raw
"
```

**注意**:executable 名是 `cameracalibrator`,**不要加 `.py`**(包里的 wrapper 脚本名不带扩展名)。

参数说明:
- `--size 10x9`:棋盘内部角点 10 列 × 9 行(对应 11×10 个格子)
- `--square 0.020`:方格边长 0.020 m
- `--no-service-check`:driver 不提供 `set_camera_info` service,跳过检查

GUI 弹出后看到 4 个进度条:**X / Y / Size / Skew**,加上当前帧和右上角的 `CALIBRATE` / `SAVE` / `COMMIT` 按钮。

### 关于"灰色图像"

GUI 显示的图像是**灰度**的,**不是 bug**:`camera_calibration` 内部把 BGR 转成灰度做角点检测,显示时再转回 3 通道画标记。Driver 发的是 `bgr8` 彩色,完全没问题。想看彩色用 `rqt_image_view` 订阅 `/camera/frontcamera/image_raw`。

---

## 3. 4 个进度条含义

每次角点检测成功,终端打一行(累计计数):
```
*** Added sample N, p_x = 0.711, p_y = 0.296, p_size = 0.442, skew = 0.395
```

4 个数描述这一帧棋盘的位置和姿态:

| 字段 | 范围 | 含义 |
|---|---|---|
| `p_x` | 0=最左, 1=最右 | 板子在画面水平方向位置 |
| `p_y` | 0=最上, 1=最下 | 板子在画面垂直方向位置 |
| `p_size` | 0=极小, 1=占满 | 板子占画面比例(等价于远近) |
| `skew` | 0=正对, 1=极斜 | 板子的倾斜程度 |

进度条变绿的条件是 **p_xxx 覆盖范围足够大**:

| 进度条 | 全绿要求 |
|---|---|
| **X** | 出现 p_x < 0.2 **且** p_x > 0.8 的帧 |
| **Y** | 出现 p_y < 0.2 **且** p_y > 0.8 的帧 |
| **Size** | 出现 p_size < 0.25 **且** p_size > 0.55 的帧 |
| **Skew** | 出现 skew > 0.6 的帧(大幅倾斜) |

**4 条全绿才能 CALIBRATE**,否则按钮是灰的。

---

## 4. 棋盘格移动策略

目标是 **30+ 帧** 覆盖画面各区域、各种距离、各种角度。建议:

### 4.1 远近(影响 Size)
```
贴近:Go2 离板子约 0.3m,板子占画面 60-80%        拍 5 帧
中距:Go2 离板子约 0.8m,板子占画面 30-40%        拍 5 帧
拉远:Go2 离板子约 1.5m+,板子占画面 15-25%       拍 5 帧
```

### 4.2 横向(影响 X)
```
最左:板子完全在画面左侧,p_x < 0.2              拍 4 帧
中间:板子在画面中部                              拍 4 帧
最右:板子完全在画面右侧,p_x > 0.8               拍 4 帧
```

### 4.3 倾斜(影响 Skew)
```
前后倾:板子上下边缘远离 / 靠近相机,±30°~45°    拍 4 帧
左右倾:板子左右边缘远离 / 靠近相机,±30°~45°    拍 4 帧
绕光轴:板子在画面里歪 ±45°                      拍 4 帧
```

### 4.4 广角相机特殊提示

Go2 前相机是广角(实测水平 FOV 约 75°,垂直 FOV 约 47°),**画面 4 个角畸变最严重**:
- 角点检测在边角容易失败
- **板子不要伸到画面 4 个角**,保持在中央 70% 区域
- 但 X 进度条又要求横向覆盖,所以"推到画面最左/最右,但要稍微往中间收一点"

### 4.5 进度条没全绿怎么办

如果一直某条是黄的,说明该方向缺样本:
- X 黄 → 没把板子推到画面两侧边缘
- Size 黄 → 远近变化不够大,要么贴近要么拉远
- Skew 黄 → 没大幅倾斜,板子一直正对相机

**保持耐心,慢慢拍,采集 30+ 帧**。

---

## 5. 触发标定

4 条进度条全绿后:
1. 点 **CALIBRATE**(右下)。等 2-3 秒,终端打印 RMS 重投影误差:
   ```
   **** Calibrating ****
   mono pinhole calibration...
   D = [...]
   K = [...]
   ...
   ('Wrote calibration data to', '/tmp/calibrationdata.tar.gz')
   ```
   - **RMS < 0.5 px**:标定质量好,继续 SAVE
   - **RMS 0.5-1.0 px**:可接受,但建议重采几帧再标
   - **RMS > 1.0 px**:重新采集更多变化样本
2. 点 **SAVE**(写 `/tmp/calibrationdata.tar.gz`,含 `ost.yaml` + 帧图 + calibrationdata.txt)
3. 点 **COMMIT**(可选,我们 driver 不接收,no-op)
4. 关闭 GUI

---

## 6. 把标定结果转换 + 应用

### 6.1 提取 ost.yaml

```bash
docker exec self_l4t bash -c "tar -xzf /tmp/calibrationdata.tar.gz -C /tmp"
```

### 6.2 跑转换脚本,覆盖 config yaml

```bash
docker exec self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  python3 /my_ws/l4t/Lunar-Grotto-Walker/tools/apply_calibration.py \
    /tmp/ost.yaml \
    /my_ws/l4t/Lunar-Grotto-Walker/src/go2_front_camera_driver/config/front_camera_720.yaml
"
```

脚本会自动:
- 读取 K / D / distortion_model
- 把 `camera_name` 改成 `go2_front_camera`
- 添加 `rectification_matrix: I` + `projection_matrix: [K | 0]`(单目布局)
- distortion 系数补齐 / 截断到 5 个(plumb_bob 标准)

**⚠️ 注意 Projection matrix**:oST 文件输出的 P 跟 K 不一致(典型 stereo 行为)。单目相机要用 P = [K | 0](fx'=fx, fy'=fy, tx=ty=0)。脚本已经处理这个。

### 6.3 重 build + 重启 driver + 验证

```bash
# 重 build(让 install/ 跟新 src/ 对齐)
docker exec -w /my_ws/l4t/Lunar-Grotto-Walker self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/ros2_ws/install/setup.bash &&
  colcon build --packages-select go2_front_camera_driver
"

# 重启 driver
docker exec self_l4t bash -c "pkill -f go2_front_camera_driver_node; sleep 1"
docker exec -w /my_ws/l4t/Lunar-Grotto-Walker self_l4t bash -c "
  source /my_ws/l4t/l4t_setup.sh &&
  source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash &&
  ros2 launch go2_front_camera_driver front_camera.launch.py
" &

# 验证新内参
docker exec self_l4t bash -c "
  source /opt/ros/humble/setup.bash &&
  source /my_ws/l4t/Lunar-Grotto-Walker/install/setup.bash &&
  ros2 topic echo --once /camera/frontcamera/camera_info | head -25
"
```

期望:`camera_name: go2_front_camera`,K 矩阵 fx≈829、fy≈830、cx≈632、cy≈364(以你这次实测为准)。

---

## 7. 内参解读参考

标定输出示例(本次实测 Go2):

```
K = [828.97, 0,      631.88,
     0,      829.79, 363.94,
     0,      0,      1]
D = [-0.356, 0.102, -0.003, -0.003, 0]   # plumb_bob: k1, k2, p1, p2, k3
```

**焦距**:
- fx ≈ 829,fy ≈ 830 → 像素密度均匀,无明显各向异性
- 1m 远的物体在画面里约占 829 像素(m/像素换算:物体实际长度 m / 829 ≈ 占画面比例)

**主点**:
- cx ≈ 632(图像宽 1280,理论中心 640),偏左 8 像素 → 镜头安装基本居中
- cy ≈ 364(图像高 720,理论中心 360),几乎正中

**实测 FOV**:
- 水平 FOV = 2 × atan(1280 / (2 × 828.97)) ≈ **75.3°**
- 垂直 FOV = 2 × atan(720 / (2 × 829.79)) ≈ **46.9°**
- 对角 FOV ≈ **88.6°**

(对比 Unitree 官方 spec 标 100° / 56° → 实测有效像素 FOV 比 spec 小约 25%,这在广角相机里是正常的,通常官方标的是对角或含畸变前的数字。)

**畸变**:
- k1 = -0.356 → **桶形畸变**(barrel),负值且较大,典型广角
- k2 = +0.102 → k1 的高阶修正
- p1, p2 ≈ 0 → 切向畸变极小(镜头安装无偏移)
- k3 = 0 → 不需要径向三阶修正

畸变越大,广角相机"边缘拉伸"的视觉感受越强。SLAM / 投影前必须用这组参数校正,否则边缘点会偏几像素。

---

## 8. 何时重新标定

- 镜头更换或重新对焦
- Go2 受到硬冲击(传感器可能在壳内移位)
- SLAM / 投影残差突然异常变大

---

## 9. 已知坑

- **`--no-service-check` 必须加**:driver 不提供 `set_camera_info` service,GUI 默认会去 query,会卡住
- **GUI 显示是灰度图**,不是 driver 发了灰度图(想看彩色用 `rqt_image_view`)
- **广角 4 角畸变重**:板子避开画面 4 角
- **Go2 实际帧率不稳**:std dev 可能 0.01-0.02s,正常;只要 average 在 13-15Hz 就行
- **Distortion 系数符号**:k1 负 = 桶形畸变(广角典型);k1 正 = 枕形畸变(长焦典型)
- **标定时 Go2 必须完全静止**:运动模糊会让 corner detection 失败
- **`/tmp/calibrationdata.tar.gz` 容器内会丢**(容器重启后清空):做完立刻 `docker cp self_l4t:/tmp/calibrationdata.tar.gz ./` 备份

---

## 附:本次实测标定结果参考

> 以下是某次 Go2 实测值,不同机器会有差异,不要直接复用。

```yaml
camera_matrix:
  rows: 3
  cols: 3
  data: [828.970896, 0.0,       631.879733,
         0.0,       829.787441, 363.941895,
         0.0,       0.0,       1.0]

distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.355926, 0.101672, -0.003034, -0.003237, 0.0]
```

- 实测水平 FOV:75.3°,垂直 FOV:46.9°
- 桶形畸变 k1 = -0.356(广角典型)
- RMS 重投影误差:< 0.5 px(标定质量好)
