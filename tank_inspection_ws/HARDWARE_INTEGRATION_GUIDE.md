# 罐体巡检硬件启动、验证与新设备接入指南

本文档适用于工作区：

```text
/home/dts/physic_bev/tank_inspection_ws
```

当前已用真实 Intel RealSense D405 和 MID-360 完成单设备及联合采集验证：深度图、
CameraInfo、点云、原始/标准化 IMU、TF、diagnostics 和 MCAP 离线结构检查均已通过。
`eddy_driver` 已接入统一硬件 Launch，但涡流、超声和机器人控制器仍未完成真实设备验证；
跨设备空间标定与统一时钟也尚未验收。
本文档不包含 Physics-BEV、神经网络或检测算法。

## 1. 基本原则

- 原始模态必须使用独立 ROS 2 Topic，不得拼成一条多模态大消息。
- `/derived/inspection/fusion_index` 只保存时间关联和状态，不复制图像、点云或波形。
- 原始 MCAP 是不可变数据源；验证、Manifest 和派生结果不能回写或替换原始数据。
- 硬件时间戳应代表采样时刻，不能用收到数据的主机时刻冒充设备采样时刻。
- 无数据、超时或未标定时必须标记 `valid=false`，不能发布零数据冒充有效测量。
- 每次 rosbag 输出目录必须是不存在的新目录。
- 录制结束必须使用一次正常 `Ctrl+C`，等待 recorder 输出 `Recording stopped`。
- 不在 Conda/venv 中运行 ROS 2 节点；用户名、设备序列号和站点 IP 不写进通用 Launch
  或源代码，站点 IP 只放在显式传入并经过审核的厂商 JSON 中；不保存云凭据。
- 修改网络、安装系统包或运行 `sudo` 前必须单独确认。

## 2. 每个终端的环境准备

如果提示符包含 `(base)`，先退出 Conda：

```bash
conda deactivate
```

然后执行：

```bash
cd /home/dts/physic_bev/tank_inspection_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOG_DIR="$PWD/reports/ros_logs"
```

如果需要完全干净的 shell：

```bash
env -i USER="$USER" LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  PATH=/usr/bin:/bin:/usr/sbin:/sbin \
  ROS_LOG_DIR="$PWD/reports/ros_logs" \
  /usr/bin/bash --noprofile --norc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 3. D405：启动与实时检查

### 3.1 当前工程模式：只启用深度

当前 `hardware_pipeline.launch.py` 启用 D405 深度流，关闭彩色流。先进行不录制检查：

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_mid360:=false \
  start_d405:=true \
  d405_serial_no:=_YOUR_SERIAL \
  d405_depth_profile:=640x480x30 \
  start_sync:=false \
  record:=false
```

数字序列号前的下划线是 RealSense ROS 参数约定。不要把真实序列号写入 Launch、YAML
或源代码；每次在命令行传入。

在另一个已 source 的终端检查：

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /camera/camera/depth/image_rect_raw -v
```

预期至少存在：

| Topic | 类型 | 用途 |
|---|---|---|
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 原始深度图 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度相机内参 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 驱动诊断 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 相机内部静态 TF |

读取一帧而不展开大数组：

```bash
ros2 topic echo /camera/camera/depth/image_rect_raw \
  sensor_msgs/msg/Image --no-arr --once --timeout 10 \
  --qos-reliability reliable --qos-durability volatile --qos-depth 10
```

对当前测试配置，预期字段是：

```text
frame_id: camera_depth_optical_frame
height: 480
width: 640
encoding: 16UC1
step: 1280
data length: 614400
```

结构关系必须满足：

```text
step == width * 2
data_length == height * step
```

检查 CameraInfo：

```bash
ros2 topic echo /camera/camera/depth/camera_info \
  sensor_msgs/msg/CameraInfo --no-arr --once --timeout 10
```

检查帧率：

```bash
ros2 topic hz /camera/camera/depth/image_rect_raw --window 200 --wall-time
```

当前现场硬件固定为 USB 2.1，因此基础 loop 默认使用 `640x480x15`，深度和彩色应接近
15 Hz。短时调度抖动可以出现，但持续平均值明显偏低、不断断流或时间戳倒退必须处理。
后续升级 USB 链路后，再重新验收 `640x480x30`，不能只改参数而沿用 15 Hz 的验收结果。

### 3.2 查看深度图

保持相机节点运行：

```bash
ros2 run rqt_image_view rqt_image_view
```

选择：

```text
/camera/camera/depth/image_rect_raw
```

深度图是 16 位距离数据，界面显示为灰度或伪彩色，不等同于普通彩色照片。如果对比度
很低，启用查看器的动态范围归一化。

也可在 RViz2 中添加 `Image` 显示，并将 QoS 设置为 `Reliable`、`Volatile`：

```bash
rviz2
```

### 3.3 查看并采集 D405 彩色流

工程默认硬件入口同时开启 D405 的 `640x480x15` 深度和彩色流，并把两路图像及其
`CameraInfo` 写入 MCAP：

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py
```

检查：

```bash
ros2 topic list | grep color
ros2 topic echo /camera/camera/color/image_raw \
  sensor_msgs/msg/Image --no-arr --once --timeout 10
```

彩色流通常使用：

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

在 `rqt_image_view` 中选择 `/camera/camera/color/image_raw` 即可查看。

如需临时覆盖默认格式和帧率，可传入 `d405_color_profile` 与
`d405_color_format`。D405 的彩色参数属于 `depth_module`，工程 Launch 已处理这个型号差异。
已生成且没有彩色 Topic 的旧 MCAP 无法事后补回彩色数据。

当前 `FusionIndex.camera_stamp` 对应深度图时间。后续如果彩色图也参与融合，应明确深度与
彩色的硬件同步语义；需要独立彩色索引时应扩展接口，不能把两个流混用为一个时间戳。

## 4. D405：MCAP 录制和验证

### 4.1 录制至少 65 秒

调试配置每 30 秒或 200 MiB 自动切卷。选择一个不存在的新目录：

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_mid360:=false \
  start_d405:=true \
  d405_serial_no:=_YOUR_SERIAL \
  d405_depth_profile:=640x480x30 \
  start_sync:=false \
  record:=true \
  debug_mode:=true \
  output_directory:="$PWD/bags/d405_test_001"
```

保持至少 65 秒，然后按一次 `Ctrl+C`。等待相机和 recorder 正常退出。

### 4.2 检查关闭与自动分卷

```bash
find bags/d405_test_001 -maxdepth 1 -name '*.mcap' \
  -printf '%f %s bytes\n'
find bags/d405_test_001 -maxdepth 1 \
  \( -name '*.active' -o -name '*.tmp' \) -print
```

通过条件：

- 至少两个 `.mcap` 分卷。
- 存在 `metadata.yaml`。
- 不存在 `.active` 或 `.tmp` 文件。

查看 rosbag 信息：

```bash
ros2 bag info -s mcap bags/d405_test_001
```

检查 `Storage id: mcap`、录制时长、消息数量和 Topic 类型。深度帧数应约等于录制秒数
乘以实际帧率。

### 4.3 运行验证器

只有 D405 时，其他多模态 Topic 合理缺失，因此使用 `--allow-partial`：

```bash
ros2 run inspection_tools validate_bag \
  bags/d405_test_001 \
  --allow-partial \
  --output reports/d405_test_001_validation.json
```

查看摘要：

```bash
jq '{
  valid,
  errors,
  image: .topics["/camera/camera/depth/image_rect_raw"],
  camera_info: .topics["/camera/camera/depth/camera_info"]
}' reports/d405_test_001_validation.json
```

通过条件：

- `valid == true`。
- `errors == []`。
- 深度 Topic 的 `non_monotonic_count == 0`。
- `average_rate_hz` 接近配置帧率。
- 所有 Image 的尺寸、`step` 和数据长度一致。

接齐全部要求 Topic 后，不再使用 `--allow-partial`。缺少任何必需原始 Topic 都应使完整
多模态验证失败。

### 4.4 生成 SHA256、Manifest 和本机远端副本

真实任务应传入真实资产信息；冒烟测试可明确使用 `UNASSIGNED`，不能编造标签：

```bash
ros2 run inspection_tools finalize_bag \
  bags/d405_test_001 \
  --task-id d405-test-001 \
  --asset-id UNASSIGNED \
  --course-id UNASSIGNED \
  --plate-id UNASSIGNED \
  --validation-report reports/d405_test_001_validation.json \
  --remote-store remote_store
```

预期状态：

```text
CLOSED -> HASHED -> COPIED -> VERIFIED
```

比较本地和副本哈希：

```bash
sha256sum \
  bags/d405_test_001/*.mcap \
  remote_store/d405-test-001/d405_test_001/*.mcap
```

同名 MCAP 的两组 SHA256 必须一致。本工具不会删除本地原始数据。

### 4.5 回放验证

先停止真实相机节点，避免真实 Topic 与回放 Topic 重名。

终端 A 先建立可靠订阅：

```bash
ros2 topic echo /camera/camera/depth/image_rect_raw \
  sensor_msgs/msg/Image --no-arr --once --timeout 15 \
  --qos-reliability reliable --qos-durability volatile --qos-depth 10
```

终端 B 再播放：

```bash
ros2 bag play -i bags/d405_test_001 mcap \
  --rate 1.0 --disable-keyboard-controls --wait-for-all-acked 1000
```

终端 A 收到结构正确的帧，且播放器正常结束，说明数据可被 rosbag2/MCAP 读取和重新发布。
也可在回放期间打开 `rqt_image_view`。

## 5. 当前标准硬件 Topic 合同

新设备优先适配到以下稳定 Topic；不要为同一物理量随意增加临时 Topic 名。

| 模态 | 标准 Topic | ROS 2 类型 | 默认角色 |
|---|---|---|---|
| D405 深度 | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 相机匹配源 |
| D405 内参 | `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 标定信息 |
| MID-360 点云 | `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 雷达匹配源 |
| MID-360 厂商原始 IMU | `/livox/imu_raw` | `sensor_msgs/msg/Imu` | 原始留档；加速度单位 g |
| MID-360 标准 IMU | `/livox/imu` | `sensor_msgs/msg/Imu` | 锚点窗口数据；SI 单位 |
| 机器人位姿 | `/robot/odometry` | `nav_msgs/msg/Odometry` | 前后位姿插值 |
| 机器人状态 | `/robot/state` | `inspection_interfaces/msg/RobotState` | 运行状态 |
| 超声原始帧 | `/inspection/ultrasound/raw` | `inspection_interfaces/msg/UltrasoundFrame` | 可选融合锚点 |
| 涡流原始帧 | `/inspection/eddy_current/raw` | `inspection_interfaces/msg/EddyCurrentFrame` | 硬件默认融合锚点 |
| 探头状态 | `/inspection/probe_state` | `inspection_interfaces/msg/ProbeState` | 接触与质量状态 |
| 静态外参 | `/tf_static` | `tf2_msgs/msg/TFMessage` | 坐标变换 |
| 融合索引 | `/derived/inspection/fusion_index` | `inspection_interfaces/msg/FusionIndex` | 派生索引 |
| 诊断 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 缓存和接收统计 |

完整字段可直接查询：

```bash
ros2 interface show inspection_interfaces/msg/EddyCurrentFrame
ros2 interface show inspection_interfaces/msg/UltrasoundFrame
ros2 interface show inspection_interfaces/msg/ProbeState
ros2 interface show inspection_interfaces/msg/RobotState
ros2 interface show inspection_interfaces/msg/FusionIndex
```

## 6. 新设备接入的标准流程

每加入一种新设备，都按以下顺序执行。不要先修改同步器再猜设备协议。

### 6.1 明确设备数据合同

接入前收集：

- 厂商、型号、固件版本和官方驱动/SDK版本。
- 物理接口：USB、以太网、串口、CAN、PCIe或厂商采集卡。
- 通信协议、SDK、示例数据包和错误码说明。
- 通道数、采样点数、采样率、触发率和数据编码。
- 设备时间戳来源、单位、回绕规则和时钟同步能力。
- 坐标系、单位、正方向、测量范围和无效值表示。
- 标定文件版本、传感器 ID 和探头/镜头/线圈编号。
- 断连、丢包、过温、饱和、脱耦或离地时的状态表示。

这些信息不完整时只能做协议探测，不能声称完成可靠硬件采集。

### 6.2 创建适配器，而不是修改原始接口语义

高吞吐设备适配器优先使用 `rclcpp/C++`。适配器职责仅包括：

1. 从官方驱动、SDK或设备协议读取数据。
2. 校验包长度、序列号、时间戳和状态位。
3. 转换成项目标准 ROS 2 消息。
4. 发布接收、丢包、乱序、断连和设备错误诊断。

适配器不得：

- 把多个大模态拼成一条消息。
- 在回调中进行神经网络或耗时融合。
- 用主机当前时间覆盖有效的设备采样时间。
- 用零数组代替缺失数据并标记为有效。
- 在源码中写死 IP、序列号或标定值。

建议为每种厂商协议建立独立包，例如：

```text
src/inspection_eddy_adapter/
src/inspection_livox_adapter/       # 仅在官方输出需二次适配时
src/inspection_robot_adapter/
```

每个包需要正确的 `package.xml`、`CMakeLists.txt`、参数 YAML、Launch 入口和测试。

### 6.3 时间戳规则

所有进入同步器的消息必须具有非零 `header.stamp`：

- 首选硬件采样时间。
- 设备时间必须转换到项目统一的 ROS 时间域。
- 记录设备时间与主机接收时间的差值和漂移。
- 明确时间戳代表帧开始、帧结束、点扫描时刻还是包接收时刻。
- 点云的 `timestamp` 或 `time_offset` 必须定义单位、符号和参考点。
- 发生时钟跳变或回绕时发布诊断并使相应融合结果失效。

跨设备融合前必须验证统一时间源。仅仅看到 Topic 同时刷新不等于时间同步完成。

### 6.4 坐标系和外参

每个空间传感器必须有稳定、唯一的 `frame_id`。通过 `/tf_static` 发布经过测量和标定的：

```text
base_link -> camera_link
base_link -> lidar_link
base_link -> eddy_probe_link
base_link -> ultrasound_probe_link
```

禁止使用全零外参冒充已标定结果。未完成外参标定时：

- 可以录制原始 Topic。
- `asset_coordinate_valid` 必须保持 `false`。
- 不得声称完成罐体坐标或 Physics-BEV 投影。

基础数据流 loop 使用 `config/rough_extrinsics.yaml` 中带版本的粗外参。Launch 只接受明确
声明 `calibration_stage: rough` 且 `production_valid: false` 的配置，并在启动日志中打印
标定 ID、平移和旋转。该 TF 可用于 RViz 粗叠加和数据流联调，但不能解除上述生产限制。

连接两台设备后，可用单独入口生成 D405 派生深度点云并与 MID-360 实时叠加；该入口不
录包，也不启动同步器：

```bash
ros2 launch inspection_bringup rough_calibration_view.launch.py
```

### 6.5 QoS 选择

先用以下命令查看真实发布端 QoS：

```bash
ros2 topic info TOPIC_NAME -v
```

一般原则：

- 大带宽、高频原始流可使用 `best_effort`、`volatile` 和有界 `keep_last`。
- 设备或驱动要求可靠传输时使用 `reliable`，并通过压力测试确认不会形成无界积压。
- `/tf_static` 使用 `reliable`、`transient_local`。
- 录制器 QoS 必须与发布端兼容。

确定 QoS 后，同时更新：

```text
src/inspection_bringup/config/qos_overrides.yaml
```

### 6.6 更新 Launch 和录制列表

在硬件入口中增加参数和 Topic remapping：

```text
src/inspection_bringup/launch/hardware_pipeline.launch.py
```

尚未通过实机验证的硬件驱动必须保持显式 opt-in。当前默认配置只包含已联合验证的
MID-360 和 D405 深度流；涡流、同步器及后续新增设备仍不得默认启动。新增原始 Topic 后更新：

```text
src/inspection_bringup/config/record_topics.yaml
src/inspection_bringup/config/qos_overrides.yaml
```

确认原始 Topic 位于原始命名空间；只有派生数据使用 `/derived/...`。

### 6.7 扩展验证器和测试

任何新增数据类型都必须同时增加：

- 单消息结构验证。
- 数组长度和维度关系验证。
- 必需字段、单位和编码验证。
- 时间戳单调性检查。
- 异常包、缺失字段和超时测试。
- MCAP 写入、关闭、回放和哈希测试。

相关位置：

```text
src/inspection_tools/inspection_tools/validate_bag.py
src/inspection_tools/inspection_tools/validators.py
src/inspection_tools/test/
src/inspection_sync/test/
```

修改后执行：

```bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

任何失败都必须修复或记录为阻塞，不得隐藏。

## 7. MID-360 接入清单

MID-360 第一阶段使用本工作区内 `livox_ros_driver2` 的
`sensor_msgs/msg/PointCloud2` 输出，不让 Livox CustomMsg 成为核心采集接口。

接入前：

1. 确认主机网口、雷达 IP、子网掩码和设备发现方式。
2. 修改主机网络前记录现状并取得授权；不要自动改默认路由或其他网卡。
3. 只使用 Livox 官方文档和官方 `livox_ros_driver2` 仓库对应的 Jazzy/ROS 2配置。
4. 记录雷达型号、固件、驱动提交或版本和配置文件校验值。
5. 确认时间同步来源和点/包时间戳语义。

`livox_ros_driver2` 1.2.6 的官方 PointXYZRTLT PointCloud2 应验证字段：

```text
x
y
z
intensity
tag
line
timestamp
```

其中 `timestamp` 是每个点的 `float64` 时间戳。若后续独立适配器发布相对点时间，允许用
已明确单位和参考点的 `time_offset` 代替 `timestamp`；两者至少存在一个。不能把整帧同一个
接收时刻复制到所有点来冒充逐点时间。

还要验证：

- `height`、`width`、`point_step`、`row_step` 和 `data` 长度一致。
- 字段 datatype、offset 和字节序正确。
- `frame_id` 稳定且与静态外参一致。
- 点时间偏移的单位和参考时刻已记录。
- `/livox/lidar` 实测频率接近配置值。
- `/livox/imu_raw` 保留厂商协议值：角速度 rad/s、加速度 g。
- `/livox/imu` 保留角速度 rad/s，将加速度乘以 9.80665 转为 m/s^2，并明确标记无姿态估计。
- 断网、重连和丢包均有诊断，缓存不会无限增长。

Livox 官方 MID-360 通信协议明确规定 IMU `gyro_*` 单位为 rad/s、`acc_*` 单位为 g：
[Livox MID-360 Communication Protocol](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)。
`livox_ros_driver2` 1.2.6 把这些字段直接复制进 `sensor_msgs/msg/Imu`，因此不能把厂商输出
直接当作完全符合 ROS SI 单位的标准 Topic。工程 Launch 会把厂商 `/livox/imu` 重映射为
`/livox/imu_raw`，再由 `inspection_livox_adapter` 发布标准 `/livox/imu`；两者都写入 MCAP。

当前工程固定使用 `xfer_format=0`、`multi_topic=0`，不启动 RViz。默认参数已经指向安装后
的已核对厂商 JSON，并默认开启 MID-360、D405 深度流、IMU 适配器和生产 MCAP 录制，
因此联合采集只需：

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py
```

每次启动会在当前工作区的 `bags/` 下生成新的时间戳目录，无需手工修改
`output_directory`。如只检查实时 Topic 而不录制：

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py record:=false
```

默认 `start_mid360:=true`、`start_d405:=true`；D405 自动选择当前连接设备，无需保存
序列号。若 JSON 不存在或构建后找不到 `livox_ros_driver2`，Launch 会明确失败。当前 JSON
已配置雷达 `192.168.2.3`、主机
`192.168.2.100` 和 561xx--565xx 端口；换网口、主机或站点时必须重新审核，不能直接照搬。
如果官方 Topic 不是标准名称，优先使用 Launch remapping；只有消息结构或单位不兼容时
才写独立适配器。

### 7.1 当前实机基线（2026-07-21）

- 驱动 1.2.6、SDK2 1.2.5 兼容构建；点云约 10.000 Hz，IMU 约 199.994 Hz。
- PointCloud2 字段为 `x/y/z/intensity/tag/line/timestamp`，offset 分别为
  `0/4/8/12/16/17/18`，`point_step=26`，`frame_id=livox_frame`。
- 一帧实测约 19,968 点，逐点时间覆盖约 100.25 ms。不同扫描线交错存放，数组内逐点
  时间戳不保证单调；融合/去畸变必须读取每点 `timestamp`。
- 原始与标准 IMU 同时订阅时都是 199.994 Hz，时间戳集合一致；静止加速度均值由
  0.993 g 转为 9.741 m/s^2，适配器拒绝数和时间戳倒退数均为 0。
- 冒烟包 `bags/mid360_smoke_20260721_1525`：92.55 秒、4 个分卷、217.5 MiB、
  37,765 条消息；`--allow-partial` 验证通过且 0 错误。

这组基线只证明单设备采集。JSON 中外参仍全零，尚未证明 base-to-lidar 外参、与 D405/
机器人时钟一致、断网重连或长时间温升稳定性。

## 8. 涡流设备接入清单

涡流适配器应发布：

```text
/inspection/eddy_current/raw
inspection_interfaces/msg/EddyCurrentFrame
```

关键约定：

- `header.stamp` 是该帧采样时间。
- `sequence` 单调递增；重启后的行为要记录。
- `sensor_id` 和 `calibration_id` 来自参数或任务配置。
- `channel_count`、`sample_count` 与实际数据一致。
- `signal_i` 和 `signal_q` 均按 channel-major 保存。
- 两个数组长度都应为 `channel_count * sample_count`。
- `sampling_rate_hz`、`excitation_frequency_hz`、`gain_db` 和 `lift_off_mm` 单位固定。
- `quality_score` 必须来自可解释的质量规则；未知时不能填写虚假的高分。

当前加入的 STM32 协议帧是 `20样本 x 8通道 x int16`，不包含设备采样时间戳、Q 分量、
增益或 lift-off。现阶段驱动采取可审计的保守映射：

- 每个完整协议帧都发布为一条标准消息，`signal_i` 按 channel-major 保存。
- `header.stamp` 是主机完成该帧协议解析的系统时间，不冒充硬件采样时间。
- `signal_q` 保持相同数组长度，但所有元素为 NaN；`quality_score=0`。
- 未配置的 `gain_db`、`lift_off_mm` 为 NaN，未知采样率和激励频率为 0。
- `/diagnostics` 记录时间戳来源、布局、解析帧数、队列长度和队列丢帧数。

这使原始 ADC 可以先被无损、可追溯地接入采集链，但不表示 I/Q、标定或跨设备时间同步
已经完成。验证器会把未分配 ID、零采样率/激励、NaN 增益/lift-off 或完全缺失的有限 Q
数据列为 `eddy-current acceptance blocker`，所以这种过渡数据可以采集和检查，但不能进入
`VERIFIED` 的完整硬件验收。厂商语义确认后必须替换上述未知量，并用真实数据重新验收。

独立启动检查：

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_mid360:=false \
  start_eddy:=true \
  eddy_device:=/dev/ttyACM0 \
  eddy_sensor_id:=ACTUAL_SENSOR_ID \
  eddy_calibration_id:=UNASSIGNED \
  eddy_sampling_rate_hz:=0.0 \
  start_sync:=false record:=false
```

在接入前需要厂商提供或确认：

- 设备型号、采集卡和探头/线圈型号。
- USB/以太网/串口/CAN/SDK 接口。
- 数据包结构、字节序、缩放系数和校验方式。
- I/Q 是原始 ADC、解调值还是已滤波结果。
- 通道扫描顺序、触发方式和时间戳位置。
- 激励频率、增益、lift-off 和温度补偿标定方法。
- 断耦、饱和、过载、丢包和探头离地的状态位。

当前硬件同步入口默认使用：

```text
anchor_topic=/inspection/eddy_current/raw
```

涡流帧未发布时不会产生锚点。即使涡流存在，若相机、点云、IMU或Odometry缺失，
同步器仍会输出 `FusionIndex`，但对应 `valid=false` 并填写 `invalid_reason`；这是正确行为。

涡流接入后的最低测试：

```bash
ros2 topic info /inspection/eddy_current/raw -v
ros2 topic hz /inspection/eddy_current/raw --window 200 --wall-time
ros2 topic echo /inspection/eddy_current/raw \
  inspection_interfaces/msg/EddyCurrentFrame --no-arr --once --timeout 10
```

然后执行包含 D405、MID-360、IMU、Odometry 和涡流的 60 秒录制，运行不带
`--allow-partial` 的完整验证。

## 9. 开启硬件同步器

只有设备协议、时间戳和 Topic 基本检查完成后才开启：

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_d405:=true \
  d405_serial_no:=_YOUR_SERIAL \
  start_mid360:=true \
  mid360_config_path:=/absolute/path/to/MID360_config.json \
  start_eddy:=true \
  eddy_device:=/dev/ttyACM0 \
  eddy_sensor_id:=ACTUAL_SENSOR_ID \
  start_sync:=true \
  anchor_topic:=/inspection/eddy_current/raw \
  asset_id:=UNASSIGNED \
  course_id:=UNASSIGNED \
  plate_id:=UNASSIGNED \
  weld_id:=UNASSIGNED \
  record:=true \
  debug_mode:=true \
  output_directory:="$PWD/bags/hardware_multimodal_001"
```

未完成测绘和资产坐标标定前保持 `UNASSIGNED`。硬件 Launch 强制
`asset_coordinate_valid=false`，避免继承 mock 标记。

检查融合输出：

```bash
ros2 topic echo /derived/inspection/fusion_index \
  inspection_interfaces/msg/FusionIndex --no-arr --once --timeout 10
ros2 topic hz /derived/inspection/fusion_index --window 200 --wall-time
ros2 topic echo /diagnostics \
  diagnostic_msgs/msg/DiagnosticArray --no-arr --once --timeout 10
```

重点检查：

- `anchor_topic` 和 `anchor_sequence` 正确。
- camera/lidar/imu/pose 的 valid 标志符合实际数据存在情况。
- 时间误差不超过配置容差。
- `invalid_reason` 对缺失或超时原因有明确描述。
- 各缓存长度始终低于 `max_count`，同时受 `max_age` 淘汰。
- 迟到、乱序和接收计数与故障注入相符。
- 不存在持续增长的 pending、RSS 或 rosbag 写缓存。

## 10. 分阶段验收建议

### 阶段 A：单设备冒烟

- 连续运行 60--120 秒。
- 检查消息结构、频率、时间戳、QoS和诊断。
- 生成至少两个 MCAP 分卷。
- 完成关闭、验证、哈希、复制和回放。

### 阶段 B：多设备短测

- 同时启动全部真实 Topic，运行 5--15 分钟。
- 检查融合有效率和各模态时间误差。
- 断开并恢复一个设备，确认诊断和 `valid=false` 行为。
- 检查缓存长度、进程 RSS、CPU、磁盘写入和网络丢包。

### 阶段 C：正式采集前压力测试

- 按生产分卷配置连续运行至少 1 小时。
- 使用目标分辨率、帧率、点率和探头触发率。
- 检查温升、USB/网络重连、磁盘空间和写入余量。
- 回放完整任务，并重新运行同步器生成 FusionIndex。
- 验证本地和远端副本全部 SHA256。
- 记录标定版本、设备固件、驱动版本和任务参数。

只有阶段 C 通过后，才能把硬件链路视为可用于正式采集；仍不能据此声称 Physics-BEV
或缺陷检测算法已经完成。

## 11. 常见问题

| 现象 | 检查与处理 |
|---|---|
| 提示符显示 `(base)` | `conda deactivate` 后重新 source ROS 和工作区。 |
| 找不到 D405 | 检查 USB、电源、`rs-enumerate-devices -s` 和设备权限。 |
| D405 显示 USB 2.1 | 直连 USB 3.x，避免低速 Hub；重新测帧率和带宽。 |
| RealSense 出现 undefined symbol | 确认 `realsense2_camera`、`diagnostic_updater` 和 ROS 二进制来自兼容仓库版本。 |
| 能 echo 深度但看不到图 | 使用 `rqt_image_view`，选择深度 Topic，并启用动态范围归一化。 |
| MCAP 没有彩色图 | 当前工程关闭彩色且未录制彩色 Topic；启用并更新录制列表后重新采集。 |
| recorder 未订阅 Topic | 检查 Topic 名、类型和 `qos_overrides.yaml` 是否兼容发布端。 |
| 找不到 Livox 包 | 在工作区重新构建并 `source install/setup.bash`，确认 `ros2 pkg prefix livox_ros_driver2`。 |
| MID-360 启动即报配置错误 | 传入存在的绝对 `mid360_config_path`，并核对 JSON 中现场 IP/端口。 |
| 涡流诊断长期 WARN | 当前协议缺硬件时间戳/Q/标定语义；核对 `dropped_frame_count`，完成合同后再验收。 |
| 没有 FusionIndex | 检查 `start_sync` 和锚点 Topic；默认硬件锚点是涡流。 |
| FusionIndex 大量 invalid | 查看 `invalid_reason`、诊断计数、时间容差、IMU窗口和Odometry前后间隔。 |
| 回放开始时提示少量 lost | 先启动可靠订阅，再以 1x 回放并使用 `--wait-for-all-acked`。 |
| 验证器报告缺少 Topic | 单设备测试使用 `--allow-partial`；完整多模态验收不得使用。 |
| rosbag 目录已存在 | 选择新的输出目录；不要覆盖、删除或复用旧原始数据目录。 |

## 12. 采集前最终检查表

- [ ] 已退出 Conda/venv并 source正确 ROS 和工作区。
- [ ] 所有设备型号、固件、驱动版本和标定ID已记录。
- [ ] D405 深度/彩色选择和分辨率已明确。
- [ ] D405 使用目标 USB 链路并通过持续带宽测试。
- [ ] MID-360 IP、`timestamp`/`time_offset` 单位与参考点、点云字段和 frame_id 已验证。
- [ ] 涡流 I/Q 编码、数组长度、激励、增益和 lift-off 已验证。
- [ ] IMU和Odometry单位、时间戳、协方差和频率已验证。
- [ ] base_link 到相机、雷达和探头的外参来自真实标定。
- [ ] 跨设备时间同步和时钟漂移已验证。
- [ ] `record_topics.yaml` 包含所有要求的独立原始 Topic。
- [ ] QoS 与真实发布端兼容。
- [ ] `asset_id/course_id/plate_id/weld_id` 真实且可追溯，或明确为 `UNASSIGNED`。
- [ ] 磁盘容量满足任务估算并保留安全余量。
- [ ] 60秒调试分卷、完整验证、回放、SHA256和复制均通过。
- [ ] 录制结束使用正常 SIGINT，没有 `.active` 或 `.tmp` 文件。
- [ ] 没有自动删除本地原始 MCAP。

当前已完成结果和已知限制见 [STATUS.md](STATUS.md)，工程常用命令见 [README.md](README.md)。
