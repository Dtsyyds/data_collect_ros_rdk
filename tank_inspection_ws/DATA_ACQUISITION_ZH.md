# 数据采集机制详解（中文）

> 本文是 `README.md` / `STATUS.md` 之外的一篇中文讲解，梳理"这套系统如何采集数据"的完整链路。
> 基于源码（launch / 配置 / C++ 节点）整理，含文件引用供核对。

掌握了全部采集链路细节，下面完整讲解这套系统**如何采集数据**。

## 一、整体数据流

采集链路分三层，从上到下串联：

```text
① 传感器层（真机 或 仿真 mock）
   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
   │ D405 相机    │   │ MID-360 雷达 │   │ STM32 涡流    │
   │ (realsense)  │   │ (livox驱动)  │   │ (eddy_driver) │
   └──────────────┘   └──────────────┘   └──────────────┘
         │  15Hz 图像/相机信息   10Hz 点云 / 200Hz IMU      ~500Hz 8×20 ADC
         ▼                   ▼                    ▼
   ┌──────────────────────────────────────────────────────┐
   │ ② 录制层：ros2 bag record（MCAP 分片、QoS、时长/大小）│   ← 原始源数据（不可变）
   └──────────────────────────────────────────────────────┘
         │（可选，opt-in）
         ▼
   ┌──────────────────────────────────────────────────────┐
   │ ③ 融合层：inspection_sync ──> /derived/…/fusion_index │   ← 派生索引（含载荷）
   └──────────────────────────────────────────────────────┘
```

**关键设计原则**：第①层的原始数据（MCAP）被视为不可变源；第③层 `FusionIndex` **只包含时间戳/有效性/时间误差/插值位姿/资产坐标，绝不携带图像、点云或波形载荷**，因此派生索引再快也不会把源数据撑爆。

## 二、传感器层——数据从哪来

### 真机采集（`hardware_pipeline.launch.py`）
launch 里把四个节点串起来（`hardware_pipeline.launch.py:291` 起）：

| 数据源                | 节点                                       | 发布话题                                                      | 实测速率/内容                                                                                                                                          |
| --------------------- | ------------------------------------------ | ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Intel D405**        | `realsense2_camera_node`（官方）           | `/camera/camera/depth/image_rect_raw` + `camera_info` + color | USB2.1 受限下 **640×480×15**（Z16 深度 / RGB8 彩色）；D405 的彩色走 stereo/depth 模块，不是普通 rgb_camera（`hardware_pipeline.launch.py:310`）        |
| **MID-360**           | `livox_ros_driver2_node`（本地 vendor 包） | `/livox/lidar`                                                | **10 Hz** PointCloud2，`xfer_format=0` PointXYZRTLT，`frame_id=livox_frame`                                                                            |
| **Livox IMU（原始）** | 同驱动，remap                              | `/livox/imu_raw`                                              | **200 Hz**，vendor 原样输出加速度单位为 **g**（官方协议定义）                                                                                          |
| **Livox IMU（SI）**   | `inspection_livox_adapter`                 | `/livox/imu`                                                  | **200 Hz**，经 `9.80665` 转换到 **m/s²**（`hardware_pipeline.launch.py:87`）                                                                           |
| **STM32 涡流探头**    | `eddy_driver_node`（`start_eddy:=true`）   | `/inspection/eddy_current/raw`                                | 串口 `/dev/ttyACM0` **4 Mbaud**，每个解析的 8×20 ADC 帧一帧，约 **500 Hz**；`signal_q=NaN`、`quality_score=0`（协议尚无 Q 分量/设备时间戳，见 STATUS） |
| **坐标/TF**           | `tf2_ros` 静态发布                         | `/tf` `/tf_static`                                            | 恒等 `inspection_origin→livox_frame` + 粗糙 `livox_frame→camera_link`（`production_valid=false`）                                                      |

超声、机器人控制器、探头状态在真机侧是**外部适配器**，只需发布约定话题即可接入；驱动层不实现它们。

### 仿真采集（`mock_pipeline.launch.py`）
当没有硬件时，用 `inspection_mock_sensors` 的 `mock_sensors_node` 发布**确定性**（`random_seed:42`）同话题仿真数据，发布话题与真机完全一致（`mock_sensors_node.cpp:89-103`）：

```text
depth 15Hz(848×480) · lidar 10Hz · imu 200Hz · odometry 50Hz · robot 20Hz
ultrasound 50Hz(2通道×2048采样) · eddy 25Hz(2通道×512) · probe 50Hz
```

它还模拟一台在半径 15m / 高 18m 罐壁上做螺旋运动的机器人，甚至可加载 `tank_vision_inspection` 导出的**任务轨迹 JSON** 代替默认螺旋（`mock_sensors.yaml:29`），用于复现真实巡检任务。

## 三、录制层——数据怎么落盘

所有原始话题的落盘统一由一个 `ros2 bag record` 进程完成（`record_only.launch.py:21`，被各采集入口 include）：

```bash
ros2 bag record --storage mcap
  --output <时间戳目录>
  --max-bag-size <大小限制>  --max-bag-duration <时长限制>
  --storage-config-file mcap_writer_options.yaml   # chunking/CRC/索引/summary
  --qos-profile-overrides-path qos_overrides.yaml
  --topics <record_topics.yaml 里的 16 个话题>
```

**录什么**（`record_topics.yaml`）——16 个话题：

```text
D405 深度图+相机信息、D405 彩色图+相机信息
Livox 点云 /livox/lidar、原始 IMU /livox/imu_raw、SI IMU /livox/imu
机器人 odometry + state、超声 raw、涡流 raw、探头状态
/tf、/tf_static、/diagnostics、/derived/inspection/fusion_index
```

**分片策略**（同文件）：生产档 **10 GB / 900 s** 自动切分；调试档 200 MB / 30 s。这保证单个 MCAP 文件可控、中途断电最多丢最后一个分片。

## 四、融合索引层——时间对齐（可选，opt-in）

默认裸采集是各管各的原始流，不做对齐。需要"一帧索引 = 某一时刻各传感器快照"时才开 `start_sync:=true`，`inspection_sync`（C++）负责：

1. **每个模态维护"有界缓存"**（`cache.yaml`）——同时限制保留时长和条数上限：
   ```
   camera 2s/100 帧 · lidar 2s/40 帧 · imu 5s/1500 条 · odometry 5s/500 · 涡流 2s/200 · …
   ```
   从根本上杜绝积压（这是它区别于"开个 rosbag 全录"的技术卖点）。
2. **选锚点**：默认锚在涡流/超声话题（README 场景用涡流，约 500 Hz 帧）。
3. **抽稀锚定**：`anchor_decimation`（涡流场景=10）——所有涡流原始帧照常全录，但**每第 10 帧**才生成一个 `FusionIndex` → 约 50 Hz 派生索引。
4. **最近帧匹配**：对该锚点，到各缓存里找 header 时间戳最近的 D405/MID-360 帧，把选中的时间戳、时间误差、有效性写进 `FusionIndex`（涡流场景实测 D405 最近帧平均绝对误差 16.7ms、MID-360 24.3ms）。
5. **资产坐标换算**：`asset_coordinate_mode: tank_xyz` 把笛卡尔 odometry 位姿转成罐壁圆柱坐标 **`(s_m 弧长, v_m 垂直, n_m 法向)`**（用半径/起始角/探针间距换算，`cache.yaml:42`），为后续缺陷定位的 `(s,v,n)` 语义打底。
6. **时间基准纪律**：各模态缓存条目里的原始 header 时间戳**一个都不改**；跨设备固定钟偏 `camera_clock_offset_ms`/`livox_clock_offset_ms` 默认 0，在线自适应默认关闭——README 里专门记录了一次反例：给 Livox 强加固定偏移植只让匹配"换帧"，绝对误差反而变差，故判定为周期换相而非真实时钟修正。

## 五、一键采集与自动停机

`baseline_capture.launch.py` 是"一键 60 秒基线采集"入口，内部 = `hardware_pipeline` + **定时关机**：

```python
TimerAction(period=seconds,                     # 默认 60
    actions=[EmitEvent(event=Shutdown(reason="baseline capture duration 60s completed"))])
```

到点发射 launch 的 `Shutdown` 事件让整条链路**干净停机**——录像器把缓存 flush、各进程以 0 退出（`baseline_capture.launch.py:38`）。所以现场操作员只需：

```bash
# 一键采集 60s 原始数据（自动停、自动生成 bags/baseline_loop_时间戳 目录）
ros2 launch inspection_bringup baseline_capture.launch.py

# 结束后校验→哈希→拷到本地 NAS（先本地后远端 SHA256 比对）
ros2 run inspection_tools validate_bag  bags/baseline_loop_* ...
ros2 run inspection_tools finalize_bag ...
```

## 六、一次采集背后的数据管线小结

```text
传感器/仿真 ──每个模态独立发布原始话题（带各自 header 时间戳）
     │
     ├─(默认开) ros2 bag record → 多分片 MCAP（10GB/15min 自动切）── 这是"数据"
     │
     └─(选配)  inspection_sync：有界缓存 + 最近帧匹配 + SLERP/插值
                  └─> 每 N 个锚帧 生成一个 FusionIndex（仅时间/位姿/有效性/资产坐标）
                         ├─ 原始 MCAP 不含 FusionIndex 载荷
                         └─ 回放时可重生成（replay_regenerated_* 验证过 99.9%+ 全有效）
```

一句话总结：**传感器侧尽量"真实、低层、带权威时间戳"，录制侧"不可变 MCAP + 定时长分片 + QoS 保证"，分析侧"只抽派生索引、不碰原始帧"**——采集的目标是给后续 Physics-BEV 提供**边界有界、来源可追溯、时间可对齐**的多模态数据。
