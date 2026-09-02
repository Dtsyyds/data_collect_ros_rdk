# 储罐多传感器采集 Demo：采集、校验与回溯

本文面向现场演示人员。目标是在不填写额外参数的情况下，完成 D405、MID-360、MID-360
IMU 和涡流设备的同步采集、MCAP 检查与可视化回放。

当前版本使用涡流数据的主机接收时间作为锚点，在有界缓存中匹配最近的 D405、MID-360
和 IMU 时间戳。它是软件时间匹配闭环，不是硬件触发同步。所有原始消息和原始时间戳均
保持不变，匹配结果单独记录在 `/derived/inspection/fusion_index`。

## 1. 演示结构

标准演示只需要三条主命令：

```bash
# 1. 采集 60 秒并自动关闭 MCAP
ros2 launch inspection_bringup eddy_sync_capture.launch.py

# 2. 自动检查最新采集包
ros2 run inspection_tools inspect_latest

# 3. 自动回放最新采集包并打开 RViz
ros2 launch inspection_bringup eddy_sync_replay.launch.py
```

每个新终端都需要先进入工作空间并加载环境：

```bash
cd ~/physic_bev/tank_inspection_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 2. 演示前检查

### 2.1 硬件连接

- D405 已接入当前 USB 2.1 端口；本 Demo 默认使用 `640x480x15` 的深度和彩色流。
- MID-360 地址为 `192.168.2.3`，网线和供电正常。
- 涡流设备位于 `/dev/ttyACM0`，串口速率为 4 Mbaud。
- 演示区域内放置有几何特征的目标，涡流探头附近准备一块金属试件。

依次执行：

```bash
ping -c 3 192.168.2.3
rs-enumerate-devices -s
ls -l /dev/ttyACM0
df -h .
```

预期现象：

- MID-360 ping 无丢包；
- RealSense 输出中出现 `Intel RealSense D405`；
- `/dev/ttyACM0` 对当前用户可读写；
- 工作空间至少保留 5 GiB。60 秒 Demo 通常约占 1 GiB，实际大小随场景和压缩率变化。

D405 日志中的 `connected using a 2.1 port` 是当前硬件条件下的已知提示，不代表启动失败。

### 2.2 现场准备

- 关闭之前残留的采集或回放进程，避免两个节点同时占用相机或串口。
- 固定 MID-360 与 D405 的相对位置，不要在采集中改变粗标定支架。
- 为方便讲解，建议同时展示终端和 RViz 窗口。

## 3. 一键采集

执行：

```bash
ros2 launch inspection_bringup eddy_sync_capture.launch.py
```

默认行为：

- 同时启动 D405 深度与彩色图像；
- 启动 MID-360 点云和 IMU，并将 IMU 加速度从 `g` 转换为 `m/s²`；
- 启动 `/dev/ttyACM0` 上的涡流驱动；
- 涡流原始帧约 500 Hz 全量保存；
- 每 10 个涡流帧生成一个约 50 Hz 的 `FusionIndex`；
- 使用已配置的粗略 D405—MID-360 外参发布静态 TF；
- 写入 MCAP，60 秒后自动安全停止；
- 输出目录自动命名为 `bags/eddy_sync_loop_时间戳/`。

启动成功时应能看到以下关键信息：

- `RealSense Node Is Up!`；
- Livox 输出 `livox/lidar publish use PointCloud2 format`；
- 涡流驱动开始持续发布且 dropped frame 为 0；
- `rosbag2_recorder` 输出 `Recording...`；
- 同步节点显示 Eddy 为 anchor、`anchor_decimation=10` 或相应诊断信息。

采集过程中建议进行一段容易回溯的动作：

1. 前 5 秒保持设备和目标静止；
2. 缓慢移动一个可见目标经过 D405 与 MID-360 的共同视野；
3. 同时让涡流探头从空气接近金属表面，再沿金属表面平稳移动；
4. 最后 5 秒重新静止。

正常情况下等待自动结束即可。需要提前结束时按 `Ctrl+C`，并等待终端出现
`Writing remaining messages`、`Recording stopped` 和各节点 cleanly finished 后再断电。
不要直接关闭终端或拔掉存储设备。

## 4. 自动校验最新采集包

采集进程退出后，在工作空间执行：

```bash
ros2 run inspection_tools inspect_latest
```

该命令会自动选择最新的、包含 `metadata.yaml` 的 `bags/eddy_sync_loop_*` 目录，并完成：

- MCAP 可读性和消息反序列化检查；
- Demo 必需 Topic 检查；
- 图像、点云和涡流数组结构检查；
- Topic 时间戳单调性检查；
- 缓存上限和同步队列丢帧检查；
- 各 Topic 数量与平均频率汇总；
- 稳态软件时间匹配有效率汇总；
- 在采集目录生成 `validation_report.json`。

理想输出应满足：

- `result: PASS`；
- `errors=0`；
- `pending_dropped: 0`；
- 稳态 `steady temporal valid` 接近 100%；
- Eddy raw 约 500 Hz、FusionIndex 约 50 Hz、D405 约 15 Hz、MID-360 cloud 约 10 Hz。

出现若干 `provisional_warnings` 是当前涡流协议的已知限制：传感器 ID、标定 ID、采样率、
增益、lift-off 和 Q 分量尚未由设备协议提供。Demo 校验会明确保留这些警告，但不会把它们
误判为数据损坏。

如需查看指定的旧采集包，可传入目录：

```bash
ros2 run inspection_tools inspect_latest \
  bags/eddy_sync_loop_20260721_211217_658866
```

也可以查看 ROS 原生摘要：

```bash
ros2 bag info -s mcap bags/eddy_sync_loop_20260721_211217_658866
```

## 5. 一键回放与 RViz 回溯

确认采集包校验通过后执行：

```bash
ros2 launch inspection_bringup eddy_sync_replay.launch.py
```

该命令自动选择最新的已关闭 `eddy_sync_loop_*` 采集包，循环回放并打开准备好的 RViz：

- 彩色/强度显示的点云：MID-360 原始 PointCloud2；
- 绿色点云：根据 MCAP 中的 D405 深度图和 CameraInfo 在线重建的 XYZ 点云；
- 图像面板：D405 录制的彩色图像；
- TF：`livox_frame`、D405 各光学坐标系及粗略外参关系。

回放只读取 MCAP，因此不要求 D405、MID-360 或涡流设备仍然连接。需要指定旧包时使用：

```bash
ros2 launch inspection_bringup eddy_sync_replay.launch.py \
  bag_directory:=/home/dts/physic_bev/tank_inspection_ws/bags/eddy_sync_loop_旧时间戳
```

D405 重建点云是回放阶段的派生数据，不会写回原始 MCAP。当前外参是粗标定结果，因此
几何轮廓应大致同向和重合，但不能作为毫米级标定验收依据。

回放循环运行。完成讲解后，在启动回放的终端按 `Ctrl+C` 关闭 rosbag、重建节点和 RViz。

### 5.1 展示涡流原始帧

回放运行时，新开一个已加载环境的终端：

```bash
ros2 topic echo /inspection/eddy_current/raw --once
```

重点解释：

- `header.stamp`：主机完成接收该涡流帧时的 ROS 时间；
- `sequence`：涡流帧序号；
- `channel_count=8`、`sample_count=20`：每帧 8 通道、每通道 20 点；
- `signal_i`：当前协议实际提供的涡流幅值数据；
- `signal_q` 及未赋值标定字段：协议暂未提供，不在软件中伪造。

### 5.2 展示软件时间匹配结果

```bash
ros2 topic echo /derived/inspection/fusion_index --once
```

重点解释：

- `anchor_topic` 和 `anchor_sequence` 指向对应涡流锚点；
- `camera_stamp`、`lidar_stamp` 和 IMU 时间窗口指出实际选中的原始消息；
- `camera_time_error_ns`、`lidar_time_error_ns` 是相对锚点的时间差；
- `camera_valid`、`lidar_valid`、`imu_valid` 表示是否在容差范围内完成匹配；
- `pose_valid=false` 是因为当前 Demo 尚未接入 `/robot/odometry`，不是相机/雷达同步失败。

## 6. 采集目录内容与复原关系

一个正常 Demo 目录结构如下：

```text
bags/eddy_sync_loop_YYYYMMDD_HHMMSS_ffffff/
├── metadata.yaml
├── eddy_sync_loop_..._0.mcap
└── validation_report.json       # 执行 inspect_latest 后生成
```

- `metadata.yaml`：rosbag2 索引、时间范围、消息总数、Topic 类型和 MCAP 分片信息；
- `*.mcap`：D405、MID-360、IMU、涡流、TF、诊断和 FusionIndex 的实际消息；
- `validation_report.json`：结构校验、频率、时差、缓存和有效率的机器可读报告。

复原时，`ros2 bag play` 恢复原始 Topic；`depth_to_pointcloud` 使用深度图和 CameraInfo 重建
D405 XYZ；RViz 使用 `/tf_static` 把 D405 与 MID-360 放入同一坐标关系；已经记录的
`FusionIndex` 恢复采集时形成的跨传感器时间关联。

## 7. 建议的五分钟讲解顺序

1. **系统组成（30 秒）**：说明 D405 提供彩色/深度，MID-360 提供点云/IMU，涡流探头
   提供表面检测信号。
2. **一键采集（60 秒）**：运行采集命令，展示各驱动启动和 MCAP 正在写入。
3. **软件时间闭环（45 秒）**：说明原始时间戳不被修改，涡流锚点与最近的相机、雷达、
   IMU 数据通过 FusionIndex 建立可追溯关联。
4. **完整性检查（45 秒）**：运行 `inspect_latest`，展示 PASS、Topic 频率、零队列丢帧和
   稳态有效率。
5. **可视化回放（90 秒）**：打开 RViz，展示 MID-360、D405 重建点云、彩色图像和 TF，
   再输出一条 EddyCurrentFrame 和 FusionIndex。
6. **边界说明（30 秒）**：当前是 USB 2.1 下的基础闭环、粗外参和软件时间匹配；精标定、
   硬件触发同步、机器人里程计和涡流正式标定属于下一阶段。

## 8. 常见问题

### D405 启动失败

- 确认 `rs-enumerate-devices -s` 能看到设备；
- 关闭占用相机的其他 RealSense/RViz 进程；
- 重新插拔 USB 后再次确认设备枚举；
- USB 2.1 性能提示可以接受，但持续的设备断开或无图像不能接受。

### MID-360 无点云

- 确认 `ping -c 3 192.168.2.3` 无丢包；
- 检查主机网卡仍在 JSON 配置对应网段；
- 确认日志出现 Livox 初始化成功和 PointCloud2 发布信息。

### `/dev/ttyACM0` Permission denied

- 执行 `ls -l /dev/ttyACM0` 检查当前用户权限；
- 确认之前配置的 ACL 或 `dialout` 组仍有效；
- 不要在演示过程中临时使用 `sudo ros2 launch`，否则会改变 ROS 环境和文件归属。

### 校验 FAIL

- 先看 `errors=`，再打开最新采集目录中的 `validation_report.json`；
- Topic missing 通常表示对应设备没有真正发布；
- timestamp regression、反序列化失败或 `pending_dropped>0` 不应忽略，应重新采集；
- 仅有 provisional Eddy warnings 时，Demo 数据仍可用于当前软件闭环展示。

### RViz 看不到点云

- Fixed Frame 应为 `livox_frame`；
- 等待回放的 2 秒启动延时；
- 检查 MID-360 和 D405 reconstructed XYZ 两个 Display 已启用；
- 若画面过重，保持默认 `point_stride=2`，不要在 USB 2.1 Demo 中改为全分辨率点云。
