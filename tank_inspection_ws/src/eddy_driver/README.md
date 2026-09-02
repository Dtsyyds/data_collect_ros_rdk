# eddy_driver

便携式激励采集装置 (STM32) ROS2 驱动包，封装 USB 串口涡流数据采集与参数设置。激励默认并始终保持开启。

## 架构

```
eddy_driver_node (ROS2 Node) ── EddyController (C++) ── USB UART (/dev/ttyACM0) ── STM32
         │
         ├── /inspection/eddy_current/raw (EddyCurrentFrame) → 标准采集帧
         ├── /eddy/state  (EddyState)                       → 8通道均值/状态
         ├── ~/adc_data   (UInt8MultiArray)                 → 最近原始 ADC 帧（调试）
         ├── /diagnostics (DiagnosticArray)                 → 接收、队列和语义状态
         └── /eddy/cmd    (Int32MultiArray)                 ← 激励控制指令
```

## 话题接口

| 话题 | 方向 | 类型 | 说明 |
|---|---|---|---|
| `/inspection/eddy_current/raw` | pub | `inspection_interfaces/msg/EddyCurrentFrame` | 每个已解析的 8x20 ADC 帧 |
| `/eddy/state` | pub | `eddy_driver/msg/EddyState` | 8通道涡流均值 + 状态反馈 |
| `~/adc_data` | pub | `std_msgs/UInt8MultiArray` | 最近一帧的 320-byte ADC payload（不含协议头尾） |
| `/diagnostics` | pub | `diagnostic_msgs/msg/DiagnosticArray` | 串口状态、帧数、队列丢帧和当前数据语义 |
| `/eddy/cmd` | sub | `std_msgs/Int32MultiArray` | 幅值/频率设置与查询指令 `[cmd, value]` |

### 标准采集帧的当前语义

STM32 数据帧不含设备时间戳，也没有给出 Q 分量、增益或 lift-off。为避免伪造数据：

- `header.stamp` 是主机完成协议帧解析的系统时间。
- `sequence` 是驱动解析帧序号；队列溢出的帧数写入 `/diagnostics`。
- `signal_i` 是按 channel-major 排列的原始 int16 ADC 值（转换为 float32）。
- `signal_q` 与 `signal_i` 等长，但全部为 NaN。
- `quality_score=0`；未知 `gain_db`、`lift_off_mm` 为 NaN。
- 未确认时 `sampling_rate_hz=0`；激励频率在收到设备查询应答前也为 0。

因此当前路径适合保真记录和协议验收，但在硬件采样时钟、I/Q 含义和标定确认之前，不能
把它作为“已标定涡流融合数据”。

### EddyState 消息

```
int16[8] channels       # 8 通道均值 (每帧 20 样本平均)
int32 current_amplitude # 设备最近一次确认的当前激励幅值
int32 current_frequency # 设备最近一次确认的当前激励频率 [Hz]
bool has_status         # 是否有状态更新
uint8 status_cmd        # 应答命令字
int32 status_value      # 应答返回值
```

驱动启动后会按固定频率自动查询幅值和频率；设备尚未返回对应应答时，当前值为 `0`。

## 串口协议

### ADC 采样帧 (324 bytes)

```
┌────────┬────────────────────┬────────┐
│ 55 AA  │  320 bytes payload │ 7E FE  │
│ (2B)   │  20样本×8通道×int16│ (2B)   │
└────────┴────────────────────┴────────┘
```

### 命令帧 (7 bytes)

```
┌────┬─────┬────────────┬────┐
│ AA │ CMD │ value (LE) │ 55 │
│ 1B │ 1B  │    4B      │ 1B │
└────┴─────┴────────────┴────┘
```

## 命令字

| 命令字 | 宏 | 说明 | value |
|---|---|---|---|
| `0x01` | `CMD_SET_AMPLITUDE` | 设置激励振幅 | 10–450 |
| `0x02` | `CMD_SET_FREQUENCY` | 设置激励频率 | 10–420000000 Hz |
| `0x11` | `CMD_QUERY_AMPLITUDE` | 查询振幅 | 0 |
| `0x12` | `CMD_QUERY_FREQUENCY` | 查询频率 | 0 |

激励不提供软件启停指令，驱动启动、运行及退出期间均保持硬件默认开启状态。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `device` | `/dev/ttyACM0` | 串口设备路径 |
| `baud` | `4000000` | 波特率 (4 Mbps) |
| `publish_rate` | `10.0` | ROS 侧队列排空及兼容状态发布频率 (Hz)；标准帧不会主动降采样 |
| `settings_query_rate` | `1.0` | 自动查询当前幅值和频率的频率 (Hz) |
| `sensor_id` | `UNASSIGNED` | 真实设备 ID，必须由任务配置传入 |
| `calibration_id` | `UNASSIGNED` | 标定版本 ID |
| `frame_id` | `eddy_probe_link` | 探头坐标系 |
| `sampling_rate_hz` | `0.0` | 帧内每通道采样率；0 表示未知 |
| `gain_db` | `-1.0` | 增益；负值会输出 NaN |
| `lift_off_mm` | `-1.0` | 提离量；负值会输出 NaN |

## 文件结构

```
eddy_driver/
├── CMakeLists.txt
├── package.xml
├── msg/EddyState.msg              # 自定义消息
├── include/eddy_driver/
│   ├── eddy_controller.hpp        # 串口控制器 (线程安全双向队列)
│   └── eddy_driver_node.hpp       # ROS2 节点
├── src/
│   ├── eddy_controller.cpp        # 串口读写 + 协议状态机解析
│   ├── eddy_driver_node.cpp       # 节点实现 (pub/sub/timer)
│   └── eddy_driver_main.cpp       # 入口
├── config/eddy_params.yaml        # 参数配置
└── launch/eddy_launch.py          # 启动脚本
```

## 构建

```bash
colcon build --packages-select eddy_driver
```

## 运行

```bash
# 独立启动
ros2 launch eddy_driver eddy_launch.py

# 指定串口
ros2 launch eddy_driver eddy_launch.py device:=/dev/ttyUSB0

# 通过本工程硬件入口启动
ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_eddy:=true eddy_device:=/dev/ttyACM0 \
  eddy_sensor_id:=ACTUAL_SENSOR_ID start_sync:=false record:=false
```

## 命令行调试

```bash
# 查看涡流数据
ros2 topic echo /eddy/state
ros2 topic echo /inspection/eddy_current/raw \
  inspection_interfaces/msg/EddyCurrentFrame --no-arr --once --timeout 10
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray --no-arr --once

# 设置激励振幅
ros2 topic pub /eddy/cmd std_msgs/msg/Int32MultiArray "{data: [1, 200]}"
```
