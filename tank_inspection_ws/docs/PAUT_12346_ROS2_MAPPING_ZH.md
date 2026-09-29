# PAUT `:12346` 线格式 → ROS 2 映射与决策记录

- **文档日期**：2026-09-18
- **状态**：已实现并真机验证通过
- **上位规范**：`docs/INSPECTION_DATA_SPEC_ZH.md`（跨模态骨架与契约 C1–C7）
- **本文用途**：记录**实际**线格式到 ROS 消息的逐字段映射，以及实现过程中的关键决策与遗留问题

---

## 0. ⚠️ 先纠正一个前提：旧规格文档的字段定义是错的

`docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md` 的字段定义基于 `docs/PAUT_DATA_FORMAT_SPEC_ZH.md`（一份**在设计阶段假想的**线格式），而设备端**实际实现**的是另一套（`PautRawBroadcaster.cs` / `PautWireFormat.cs`）。两者字段对不上：

| 旧规格文档写的 | 设备实际发的 |
|---|---|
| `board_ch1` / `board_ch2` | ✗ 不存在；实际是 `board_ip`(u32) |
| `hw_filter_khz` | 实际是 `filter_level`(u8 档位，**不是 kHz**) |
| `element_group` | ✗ 不存在；实际是 `probe_type`(u8) |
| `wedge_velocity_m_s` **u16** | 实际 **f32**（u16 装不下 2337.0 这类值） |
| `specimen_velocity_m_s` **u16** | 实际 **f32** |
| `main_offset_mm` | 实际是 `primary_offset_mm` |
| `specimen_depth_mm` | 实际是 `img_height_mm` + `img_height_bias_mm` |
| `focus_start_angle_deg` / `focus_end_angle_deg` | 实际是 `focus_angle_deg` / `focus_depth_mm` |
| `step_angle_or_aperture` | 实际拆成 `beam_step` + `little_aperture` |
| `encoder_resolution_mm`（单一） | 实际是 `enc1_resolution` + `enc2_resolution` |
| `initial_time_us` | ✗ 不存在；实际是 `start_ns_raw`(u16, ns) |
| — | 漏了 `range_ns`（时基范围） |
| — | 漏了 `probe_name` / `wedge_name` / `notes` 三串 UTF-8 |
| 常量 `SCAN_MODE_FMC = 2` | 实际 `SCAN_MODE_TOFM = 2`，顺序是 线扫/扇扫/TOFM/TOFD/PWI |

**权威来源是设备端的 `.cs` 与 `scripts/recv_paut_12346.py`**（后者逐字面对照前者写成，并在真机上验证通过）。本文映射表以它们为准。

---

## 1. 报文结构

**全包显式小端、紧凑排列（无填充字节）。**

```
┌────────────────────────────────────────────┐
│ A. 包封装  24 B   魔数/版本/类型/长度/CRC32  │
├────────────────────────────────────────────┤
│ B/C/D. 载荷（按 packet_type 解释）           │
└────────────────────────────────────────────┘
```

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 4 | `magic` | `0x54554150`（"PAUT" 小端） |
| 4 | 2 | `version_major` | 1 |
| 6 | 2 | `version_minor` | 0 |
| 8 | 1 | `packet_type` | 1=CONFIG / 2=FRAME / 3=HEARTBEAT |
| 9 | 1 | `flags` | 保留 |
| 10 | 2 | `header_len` | 载荷起始偏移（CONFIG=140，FRAME=72，HEARTBEAT=32） |
| 12 | 4 | `packet_len` | 整包字节数 |
| 16 | 4 | `crc32` | 覆盖 **[24, packet_len)**，IEEE 802.3 |
| 20 | 4 | 保留 | 填 0 |

| 包类型 | 发送时机 | 大小 |
|---|---|---|
| `CONFIG`(1) | 启动 + 变更时 + **每 2s 重发** | ≈227 B |
| `FRAME`(2) | 每整帧（实测 ≈85 Hz） | 72 + 61×896 = **54,728 B** |
| `HEARTBEAT`(3) | **每 1s**（未开采集时也发，用于判活） | 32 B |

---

## 2. `CONFIG` → `PautConfig.msg` 逐字段映射

| 线格式偏移 | 线格式字段 | ROS 字段 | 类型/单位 | 备注 |
|---|---|---|---|---|
| 24 | `board_ip` | `board_ip` | u32 | **字节序特殊**，见 §2.1 |
| 28 | `sample_point_num` | `sample_point_num` | u32 / 点 | 板卡配置的采样点数（实测 1000） |
| 32 | `range_ns` | `range_ns` | u32 / ns | 时基范围（实测 100000） |
| 36 | `ut_voltage_v` | `ut_voltage_v` | u16 / V | |
| 38 | `pulse_width_ns` | `pulse_width_ns` | u16 / ns | |
| 40 | `prf_hz` | `prf_hz` | u16 / Hz | ⚠️ 脉冲重复频率，**不是采样率** |
| 42 | `start_ns_raw` | `start_ns_raw` | u16 / ns | ⚠️ 不可信，见 §3 |
| 44 | `board_type` | `board_type` | u8 | 0–8，见消息内 `BOARD_TYPE_*` 常量 |
| 45 | `filter_level` | `filter_level` | u8 / 档位 | **不是 kHz** |
| 48 | `element_pitch_mm` | `element_pitch_mm` | f32 / mm | |
| 52 | `frequency_mhz` | `frequency_mhz` | f32 / MHz | |
| 56 | `element_total` | `element_total` | u16 / 个 | 实测 64 |
| 58 | `probe_type` | `probe_type` | u8 | 设备侧枚举 |
| 60 | `wedge_angle_deg` | `wedge_angle_deg` | f32 / ° | |
| 64 | `wedge_velocity_m_s` | `wedge_velocity_m_s` | f32 / (m/s) | |
| 68 | `first_ele_height_mm` | `first_ele_height_mm` | f32 / mm | |
| 72 | `primary_offset_mm` | `primary_offset_mm` | f32 / mm | |
| 76 | `img_height_mm` | `img_height_mm` | f32 / mm | ⚠️ **显示窗口高度**，非完整声程 |
| 80 | `img_height_bias_mm` | `img_height_bias_mm` | f32 / mm | 窗口起点（第 0 行对应多深） |
| 84 | `focus_angle_deg` | `focus_angle_deg` | f32 / ° | |
| 88 | `focus_depth_mm` | `focus_depth_mm` | f32 / mm | |
| 92 | `specimen_vel_m_s` | `specimen_vel_m_s` | f32 / (m/s) | 成像算法**实际使用**的声速 |
| 96 | `board_vel_m_s` | `board_vel_m_s` | f32 / (m/s) | 另一来源，**两个都发出来便于发现不一致** |
| 100 | `little_aperture` | `little_aperture` | u16 / 阵元 | 实测 4 |
| 102 | `beam_step` | `beam_step` | u16 / 个 | 实测 1；波束位置 `x_i = i × element_pitch_mm × beam_step` |
| 104 | `beam_count` | `beam_count` | u16 / 个 | 实测 61 |
| 106 | `scan_mode` | `scan_mode` | u8 | 见 `SCAN_MODE_*` 常量 |
| 108 | `sampling_rate_hz` | `sampling_rate_hz` | f32 / Hz | ⚠️ **反推值**，见 §3.1 |
| 112 | `sampling_rate_src` | `sampling_rate_src` | u8 | `RATE_SRC_DERIVED_UNVERIFIED` |
| 113 | `sample_dtype` | `sample_dtype` | u8 | `SAMPLE_DTYPE_U8` |
| 114 | `adc_bits` | `adc_bits` | u8 | `ADC_BITS_UNKNOWN`(0) |
| 116 | `enc1_resolution` | `enc1_resolution` | f32 | 原始数值（实测 51.2） |
| 120 | `enc2_resolution` | `enc2_resolution` | f32 | 原始数值（实测 64.0） |
| 124 | `enc1_unit` | `enc1_unit` | u8 | ⚠️ `ENCODER_UNIT_UNKNOWN` — 量纲未定 |
| 125 | `enc1_polarity` | `enc1_polarity` | i8 | +1 / −1 |
| 126 | `enc1_mode` | `enc1_mode` | u8 | |
| 127 | `enc2_unit` | `enc2_unit` | u8 | |
| 128 | `enc2_polarity` | `enc2_polarity` | i8 | |
| 129 | `enc2_mode` | `enc2_mode` | u8 | |
| 132 | `config_seq` | `config_seq` | u32 | 与帧配对（§4） |
| 140+ | `probe_name` | `probe_name` | string | u16 长度前缀 + UTF-8 |
| 变长 | `wedge_name` | `wedge_name` | string | 同上 |
| 变长 | `notes` | `notes` | string | **设备端自我说明**：不可信/未验证项都写在这里 |

**驱动额外补充（设备不提供）**：`header`（收包时刻）、`sensor_id` / `calibration_id`（采集端标签，来自 ROS 参数）。

**驱动派生**：`start_ns_trusted = false`（恒 false，见 §3）。

### 2.1 `board_ip` 的字节序陷阱

它是 .NET `IPAddress.Address` 形式，**与点分表示相反**：

```
值 0x8201A8C0  →  192.168.1.130
即 a = v & 0xFF, b = (v>>8)&0xFF, c = (v>>16)&0xFF, d = (v>>24)&0xFF
```

依据：设备 `config.json` 里的 `"ip": 2181146816`（=0x8201A8C0）实测对应该板卡。

---

## 3. 「未知」的如实表达

设备端已明确几类**不可用**的量。**不填 0、不填 NaN 一刀切**（契约 C4）：

| 未知项 | 线格式表现 | ROS 里的表达 | 为什么不简单填 0/NaN |
|---|---|---|---|
| 设备硬件时戳 | 无此字段 | `device_timestamp_ns = 0` **且** `timestamp_source = TS_HOST_RECEIVE`；`header.stamp` 写**主机收包时刻** | `0` 配合 `timestamp_source` 是**无歧义**的「未知」，不是「零时刻」。这正是契约 C1.3 存在的理由 |
| ADC 位深 | `adc_bits = 0` | `adc_bits = 0` + 常量 `ADC_BITS_UNKNOWN = 0` | 位深 0 物理不可能，是无歧义哨兵；**但必须命名成常量**，否则下游当数字用 |
| 编码器单位 | `enc_unit = 0` | **`enc1_resolution` 保真填 51.2**，仅 `enc1_unit = ENCODER_UNIT_UNKNOWN` | ⚠️ 分辨率**数值是真的**，只是量纲未知。填 NaN 会**丢掉真实信息**。消费端在 `unit == UNKNOWN` 时不得用于换算，并清除 `STATUS_ENCODER_VALID` |
| 采样率可信度 | 有值 + `src = 1` | 保真填值 + `sampling_rate_src = RATE_SRC_DERIVED_UNVERIFIED`，且 `STATUS_CALIBRATED` **不置位** | 这是**第三个状态**（提供了但未验证），NaN 表达不了 |
| `start_ns_raw` 可信度 | 有值 | 保真填值 + `start_ns_trusted = false` | 设备端缺陷：`SendParamsManager` 每次下发参数都 `startNs += wedgeDelay`（非幂等，累积漂移） |

### 3.1 采样率是反推的

设备**不直接提供采样率**。当前值是：

```
sampling_rate_hz = sample_point_num / (range_ns × 1e-9)
                 = 1000 / 1e-4  = 1.0e7 Hz = 10 MHz
```

⚠️ **未经验证**，必须用已知厚度试块实测反推（底波落在第 N 点 → `fs = N × v / (2 × 深度)`）。
设备的 `notes` 字段里也明确写了 `fs=10.000MHz derived(...),UNVERIFIED`。

---

## 4. `config_seq` 的机制（本方案的关键设计）

### 4.1 三层机制

```
设备端                        驱动                          采集端
──────                       ────                          ─────
内容变化 → seq += 1
每 2s 重发同一 seq
        ──────UDP──────►  内容指纹比对
                           ├ 变了 → 发布 PautConfig（transient_local 闩锁）
                           └ 没变 → 丢弃（不动 ROS）
                                                      ──ROS──►  录制器 / 分析脚本
                                                                按 config_seq 配对
```

### 4.2 为什么用**内容指纹**而不是 `config_seq` 判变化

设备**每 2s 重发同一份配置**（`config_seq` 不变）。若驱动靠"seq 递增"判变化，它无法区分：

- **重发**（seq 相同、内容相同）→ 不该发布
- **变更**（seq 递增）→ 该发布

且设备重启会导致 seq 回退，只看 seq 会误判。用**内容指纹**（FNV-1a 覆盖全部字段，**不含 `config_seq`**）判变化，两种情况都正确，还能兜住"字段改了但 seq 没改"这种异常。

### 4.3 为什么 config 走独立话题而非每帧携带

| | 每帧携带 | 独立话题（本方案） |
|---|---|---|
| 带宽 | 86 Hz × 227 B ≈ 19 KB/s | 每次变更 1 条 |
| 语义 | 配置被当成"每帧重复的字段集合" | 配置是**有独立生命周期的实体** |
| ROS 惯用法 | 逆惯例 | 与 `sensor_msgs/Image` + `CameraInfo` 同构 |
| 行业规范 | — | **ONDE 也这么分**（`Setup` + `Datasets`） |

---

## 5. `FRAME` → `PautFrameV2.msg` 逐字段映射

| 线格式偏移 | 线格式字段 | ROS 字段 | 类型/单位 |
|---|---|---|---|
| 24 | `frame_seq` | `frame_seq` | u64 |
| 32 | `device_timestamp_ns` | `device_timestamp_ns` | u64 / ns（**恒 0**） |
| 40 | `scan_axis_encoder` | `scan_axis_encoder` | i32 / count |
| 44 | `index_axis_encoder` | `index_axis_encoder` | i32 / count |
| 48 | `beam_count` | `beam_count` | u16 / 个（实测 61） |
| 50 | `sample_count` | `sample_count` | u16 / 个（实测 **896**） |
| 52 | `element_total` | `element_total` | u16 / 个 |
| 54 | `status_flags` | `status_flags` | u16 bitfield |
| 56 | `saturated_count` | `saturated_count` | u32 / 个 |
| 60 | `frag_index` | — | **不映射**（本版不分片） |
| 62 | `frag_count` | — | **不映射**（驱动遇 `!=1` 丢弃并计数） |
| 64 | `config_seq` | `config_seq` | u32 |
| 68 | 保留 | — | — |
| 72+ | 数据段 | `samples` | **`uint8[]`**，长度 = `beam_count × sample_count` |

**数据段轴序**：`samples[beam * sample_count + sample]` —— 行优先、**波束为外层轴**。

**驱动补充**：`header`（收包时刻）、`schema_version=1`、`timestamp_source=TS_HOST_RECEIVE`。

### 5.1 三个取舍

| 取舍 | 决定 | 理由 |
|---|---|---|
| `samples` 元素类型 | **`uint8[]`** | 线上即 u8，零转换零损失。⚠️ 旧规格文档里"带宽减半"那句是针对**旧 int32 线格式**的；对新的 u8 格式，改用 `uint16[]` 是**带宽翻倍**而高位恒 0 |
| `saturation_mask` | **不做** | 设备**不产生该段**（数据段到 `72+beam*sample` 就结束）；`saturated_count` 已够用且**真机逐帧验证一致**。定义恒为空数组反而让消费端困惑 |
| `InspectionMeta` 嵌套 | **不嵌套，先平铺** | `InspectionMeta` 要求位置是工件坐标系 SI 米，但设备只给编码器 **raw counts** 且单位未知，只能填 NaN。为 6 个可填字段引入全仓首个嵌套消息不值。**留后路：字段只允许追加在末尾**（契约 C6.2） |

### 5.2 `896` vs `1000` ⚠️

```
CONFIG.sample_point_num = 1000   ← 板卡配置的采样点数
FRAME.sample_count      =  896   ← 实际每波束字节数（DplGlobal.WaveDataQty）
```

**差 104 点（约 10%）**。设备源码里处理该场景的保护代码当前是**注释状态**：

```csharp
int dataLength = DplGlobal.WaveDataQty;      // 896，写死
//if ((packageId + 1) * DplGlobal.WaveDataQty > pointNum)
//    dataLength = pointNum - startIndex;    // ← 被注释
```

**含义**：板卡可能按 1000 点发、而上位机每包固定取 896 —— **每波束后 104 点疑似被丢弃**。

**当前处置**：驱动**如实透传 896**、不做补偿；校验器记为 **warning**（`sample_count_mismatch_seqs`）。
**深度换算只能信 `FRAME.sample_count`**。

---

## 6. 话题与 QoS

| 话题 | 消息 | QoS | 说明 |
|---|---|---|---|
| `/inspection/paut/raw_v2` | `PautFrameV2` | `best_effort` + `volatile` + `keep_last(100)` | 高频数据 |
| `/inspection/paut/config` | `PautConfig` | **`transient_local`** + `reliable` + `keep_last(1)` | 闩锁，晚加入者自动补发 |
| `/diagnostics` | `DiagnosticArray` | `reliable` + `keep_last(10)` | |

### 6.1 ⚠️ 为什么不复用 `/inspection/paut/raw`

`PautFrameV2` 走**新话题名**，不是复用旧话题。两条原因：

1. **DDS 层**：同一话题下两个不同消息类型会导致订阅不匹配
2. **校验器**：`validate_bag.py` 按**话题名**分派校验（`elif topic == ...`），同话题双类型会反序列化失败

旧类型 `PautFrame` **冻结保留**（`bags/paut_real` 等 5 个 bag 依赖它的类型哈希，契约 C6.4）。

### 6.2 ⚠️ `qos_overrides.yaml` 的陷阱（实测踩过）

`inspection_bringup/config/qos_overrides.yaml` 里**除 `/tf_static` 外每条都显式写了 `durability: volatile`**。而 `rosbag2` 的**默认订阅 QoS 已是 `TRANSIENT_LOCAL`**（Jazzy 实测）。

**若照抄邻条目给 config 话题加一条 `volatile`，配置消息会静默丢失** —— bag 里只有数据没有配置，而**录制器不报任何错**。

**规则：要么不写这条，要么写就必须是 `transient_local`。**

实测三组对照（发布端 `TRANSIENT_LOCAL`，录制器晚于发布启动）：

| `qos_overrides.yaml` 写法 | 抓到的消息数 |
|---|---|
| 不写该话题（走默认） | ✅ 1 |
| 显式 `durability: transient_local` | ✅ 1 |
| 显式 `durability: volatile` | ❌ **0** |

---

## 7. 契约 C7.2 的跨话题检查

契约要求「帧的 `config_seq` 必须能在 config 话题找到」。但 `validate_bag.py` 的 `validate()` 是**单次流式遍历**，只能看到当前一条消息。

**解法**：遍历期把两侧攒进集合，**遍历结束后**统一判定。沿用该文件里 `eddy_acceptance` 已有的「累积 + 末尾判定」模式，不破坏单次遍历的设计。

### 7.1 ⚠️ 必须把启动竞态记为 warning 而非 error

**设备启动初期，FRAME 会早于首份 CONFIG** —— CONFIG 每 2s 重发、而 FRAME 是 ~86 Hz。这段窗口内帧的 `config_seq` 指向一份**还没收到的**配置。

**这是预期行为，不是数据损坏。** 若判为 error，**正常录制的 bag 会被误判为失败**。

| 情况 | 处置 |
|---|---|
| 帧引用了 config 从未出现的 seq | **warning**（预期竞态） |
| 有数据帧但**完全没有** config | **error**（数据不可解释） |
| `FRAME.sample_count` ≠ `CONFIG.sample_point_num` | **warning**（设备端已知差异） |
| `CONFIG.scan_mode != 线性扫` | **error**（本版只支持线扫） |
| `PautFrameV2` / `PautConfig` 自身结构错 | **error**（照旧） |

`--allow-provisional-paut` 可把前两类也降为 warning。

---

## 8. 落地状态

| 计划项 | 内容 | 状态 |
|---|---|---|
| T1 | `PautConfig.msg` | ✅ |
| T2 | `PautFrameV2.msg` | ✅ |
| T3 | `CMakeLists.txt` 注册 | ✅ |
| T4 | `record_topics.yaml` 加话题 | ✅ |
| T5 | `qos_overrides.yaml` **保持不动** | ✅ |
| T6 | 驱动 config 发布 | ✅ |
| T7 | 参数补齐 | ✅（v0 的 4 个参数已删除） |
| T8 | UDP 解析 v1 | ✅ `paut_v1_protocol` + `paut_v1_receiver` |
| T9 | `config_seq` 联动 | ✅（内容指纹 + transient_local） |
| T10 | 合成源升级 | ✅ **本就已是 v1**（该任务描述过时） |
| T11 | 端到端冒烟 | ✅ 见 §9 |
| T12 | HDF5 exporter | ⏳ 未做（阶段 4） |
| T13 | 全聚焦支持 | ⏳ 未做 |
| T14 | 逐样本饱和掩码 | ⏳ 未做 |

**计划外的改动**：`hardware_pipeline.launch.py`（主硬件管线，原计划漏列）—— `paut_port` 默认值、4 个废弃参数、话题重映射均已同步更新。

---

## 9. 验证记录（真机）

### 9.1 单元测试

| 层 | 结果 |
|---|---|
| C++ 协议解析（`test_v1_protocol`，gtest） | **12 用例全过**；含**跨实现比对**（Python 生成字节 → C++ 解析 → 逐字段断言） |
| Python 校验器（`test_validators.py`） | **9 用例全过**（新增 2 个 + 既有 7 个回归） |
| `colcon test`（paut_driver + inspection_tools） | **30 tests, 0 errors, 0 failures** |

### 9.2 端到端

真机采集 14 秒：

```
/inspection/paut/raw_v2 | PautFrameV2 | Count: 1037   (≈75.8 fps)
/inspection/paut/config | PautConfig  | Count:    2
Bag size: 1.7 MiB   Duration: 13.68s
```

> `Bag size` 偏小是因为数据 95% 是零（探头未耦合），Zstd 压缩约 33 倍。
> **判断录制是否成功要看 `Count`，不是 `Bag size`** —— QoS 不匹配时 `Count` 会是 0，而压缩会让 size 偏小。

**回放侧**：`ros2 bag play` 后**等 3 秒**再订阅 config，仍能收到（`transient_local` 在回放侧生效）—— 证明 bag 里的配置是**可用**的，不只是"录进去了"。

### 9.3 校验器输出

```
valid: True    errors: []
paut_v2:
  frame_count: 1037     config_count: 2
  config_seqs_seen: [4] frame_config_seqs: [4]
  frame_config_seqs_missing: []        ← C7.2 配对通过
  config_sample_point_num: {"4": 1000}
  frame_sample_count:      {"4": 896}
  sample_count_mismatch_seqs: [4]      ← 896 vs 1000 被检出
warnings:
  - paut-v1: config_seq 4: frame sample_count 896 != config sample_point_num 1000;
    depth conversion must use the frame value
```

### 9.4 排查过程中弄清的两件事

1. **帧率"168 Hz"是假象**：是**两个来源叠加**（真机 84.9 fps + 合成源 82.4 fps）。只跑驱动时是 **84.9 Hz**，正常。
2. **`SO_REUSEADDR` 是双刃剑**：它让两个驱动能同时绑 `:12346`（调试时方便与 Python 工具并行），但也意味着**残留的旧节点会静默地重复发布**，表现为话题帧率翻倍而**毫无报错**。**采集前务必 `pkill -f paut_driver_node`。**

---

## 10. 遗留问题

| # | 问题 | 影响 | 待办 |
|---|---|---|---|
| 1 | **采样率未验证** | 深度换算不可信 | 已知厚度试块实测（`fs = N × v / (2 × 深度)`） |
| 2 | **编码器单位未定** | 位置轴无法换算 | 移动探头实测 `count/mm` 还是 `mm/count` |
| 3 | **896 vs 1000** | 疑似丢 10% 深度范围 | 需设备方确认；试块验证底波是否落在 896 内 |
| 4 | `start_ns_raw` 累积漂移 | 时基起点不可信 | 设备端缺陷，单独立项修（改了会影响下发给板卡的值） |
| 5 | 8 位位宽 | 动态范围受限 | 需查设备底层库能否开放 16 位 |
| 6 | 无硬件时戳 | 多模态融合精度 = 网络抖动 | 需设备端加时钟或上硬件触发同步 |

---

*相关文档：`docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md`（消息层规格，其字段定义已被本文 §0 修正）、`docs/INSPECTION_DATA_SPEC_ZH.md`（跨模态契约）、`docs/PAUT_IMPLEMENTATION_PLAN_ZH.md`（任务清单）、`scripts/recv_paut_12346.py`（线格式权威参考实现）*
