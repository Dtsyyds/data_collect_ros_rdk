# PAUT ROS 2 消息规格 v1.0

- **文档日期**：2026-09-11
- **上层依据**：`docs/PAUT_DATA_FORMAT_SPEC_ZH.md`（设备 → 主机 UDP 线格式 v1）
- **适用**：`inspection_interfaces` 消息定义、`paut_driver` 发布逻辑、MCAP 录制话题与 QoS
- **状态**：拟定，待评审

---

## 1. 三个设计决策

### D1 — 采集头**不**随帧走，拆成独立话题（核心决策）

线格式 v1 的设计原则 P1「元数据随帧传输，每包自描述」，理由是 **UDP 无连接、会丢包、设备会重启**。

**这个理由在 ROS 里不成立。** DDS 原生提供：

| UDP 线格式的问题 | DDS 的原生解法 |
|---|---|
| 会丢包 | `reliability: reliable` |
| 晚加入的订阅者拿不到已发的元数据 | `durability: transient_local`（晚加入自动补发最后一帧） |
| 设备重启后状态丢失 | 发布端重发 + 晚加入语义 |

所以照搬 P1 到 ROS 是**把线格式的权宜之计当成了设计目标**。ROS 侧按惯例拆成两个话题更干净——而且这正是行业规范的做法：

> **ONDE 本身就是这么分的**：`/Public/Setup`（一次性的 JSON 全量元数据）+ `/Public/Groups/<Gid>/Datasets/<Did>`（数据）。我们线上因为 UDP 限制才合并，落到 ROS 应当还原成两层。

**代价要讲清楚**：消费端需要 join 两个话题。缓解办法是导出训练集时做一次物化 join——那本来就是 exporter 的职责。

### D2 — 保留厂商原生单位，字段名内嵌单位后缀

严格按 REP-103 应当统一 SI（m / s / rad）。但线格式 P2 规定**采集端不做任何缩放**，既然不转换，单位就必须写进字段名，否则 `element_pitch` 是 0.5 还是 0.0005 无从判断。

**这是对 REP-103 的有意偏离**：保真优先于单位统一。转换动作交给 exporter。

### D3 — 数组展平 + 显式维度，不用多维

ROS 2 消息没有多维数组。`uint16[] samples` 展平存放，轴序由 `PautConfig.scan_mode` 决定，维度由 `beam_count` / `sample_count` / `element_total` **显式声明**（不推导）。

---

## 2. 消息定义

### 2.1 `PautConfig.msg`（新增）

对应线格式 v1 采集头（§4）。

```
# PAUT 采集配置 —— 厂商规格中「更新写一次」的字段。
# 对应线格式 v1 采集头: docs/PAUT_DATA_FORMAT_SPEC_ZH.md §4
# 发布: /inspection/paut/config
# QoS: transient_local + reliable + keep_last(1)
# 启动时发布一次; 之后仅在配置变化时重发, 每次重发 config_seq 递增。

std_msgs/Header header              # 本配置生效时刻

uint32 config_seq                   # 配置序号; PautFrame.config_seq 与之配对

# ---- 板卡段 ----
uint8  board_type                   # 板卡类型
uint16 board_ch1                    # 同时激发/接收阵元数
uint16 board_ch2                    # 最大阵元数
uint16 pulse_width_ns               # 脉冲宽度 [ns]
uint16 excitation_voltage_v         # 激励电压 [V]
uint16 prf_hz                       # 脉冲重复频率 [Hz]  ⚠ 不是采样率
uint16 hw_filter_khz                # 硬件滤波 [kHz]; 0=不滤波
uint16 sample_count                 # 采样点数 (本机实测 167)

# ---- 探头段 ----
uint8   element_group               # 1=线阵; 其它=每线阵元数×线数=阵元总数
uint16  element_total               # 阵元总数 (本机实测 32)
float32 element_pitch_mm            # 相邻阵元中心间距 [mm]
float32 first_element_offset_mm     # 第一阵元中心距/晶元高度 [mm]

# ---- 模块段 ----
uint8  wedge_angle_deg              # 模块角度 [deg]; 无楔块填 0
uint16 wedge_velocity_m_s           # 模块声速 [m/s]; 无楔块填 0

# ---- 扫查段 ----
float32 main_offset_mm              # 主偏偏移 [mm]; 不适用填 NaN
uint8   scan_mode                   # 见 SCAN_MODE_* 常量
float32 specimen_depth_mm           # 试件深度 [mm]
float32 focus_start_angle_deg       # 聚焦起始角度 [deg]
float32 focus_end_angle_deg         # 聚焦终止角度 [deg]
uint16  specimen_velocity_m_s       # 试件声速 [m/s]
uint8   aperture                    # 孔径 [倍 pitch]
uint16  focus_depth_mm              # 聚焦深度 [mm]
float32 step_angle_or_aperture      # 步进角度 [deg] 或步进孔径 [个], 按 scan_mode 解释

# ---- 时间基准 (线格式 §4.6) ----
float32 sampling_rate_hz            # ADC 采样率 [Hz]; 未知填 NaN
float32 initial_time_us             # 下标 0 对应的声程延时 [us]; 未知填 NaN
uint8   adc_bits                    # ADC 位深; 0=未知
uint8   sample_dtype                # 见 SAMPLE_DTYPE_* 常量

# ---- 位置基准 (线格式 §4.7) ----
float32 encoder_resolution_mm       # 每计数对应毫米 [mm/count]; 未知填 NaN
int8    encoder_polarity_scan       # +1 / -1
int8    encoder_polarity_index      # +1 / -1

# ---- 常量 ----
uint8 BOARD_TYPE_0 = 0              # 0=多浦乐
uint8 BOARD_TYPE_1 = 1              # 1=中科
uint8 BOARD_TYPE_2 = 2              # 2=奥斯迪康

uint8 SCAN_MODE_LINEAR = 0          # 线扫
uint8 SCAN_MODE_SECTOR = 1          # 扇扫
uint8 SCAN_MODE_FMC    = 2          # 全聚焦
uint8 SCAN_MODE_TOFD   = 3          # TOFD
uint8 SCAN_MODE_PWI    = 4          # PWI

uint8 SAMPLE_DTYPE_U8  = 0
uint8 SAMPLE_DTYPE_I16 = 1
uint8 SAMPLE_DTYPE_U16 = 2
uint8 SAMPLE_DTYPE_I32 = 3
uint8 SAMPLE_DTYPE_F32 = 4
```

> 板卡类型常量用**序号命名**而非拼音/音译，避免造词；中文对照写在注释里。

### 2.2 `PautFrame.msg`（改写）

对应线格式 v1 的帧块 + 数据段（§5、§6）。

```
# PAUT 单帧数据。
# 对应线格式 v1 帧块+数据段: docs/PAUT_DATA_FORMAT_SPEC_ZH.md §5, §6
# 发布: /inspection/paut/raw
# QoS: sensor data (best_effort, keep_last(100))
# 与 PautConfig 通过 config_seq 配对。

std_msgs/Header header              # 优先设备时戳; 设备无时戳时为采集端收包时刻

uint32 config_seq                   # 关联 PautConfig.config_seq

# ---- 帧标识 ----
uint64 frame_seq                    # 设备帧编号
uint64 device_timestamp_ns          # 设备单调时钟 [ns]; 0 = 设备无时戳

# ---- 位置 ----
int32 scan_axis_encoder             # 扫查轴编码器计数
int32 index_axis_encoder            # 步进轴编码器计数

# ---- 维度 (显式声明, 不推导) ----
uint16 beam_count                   # 波束个数
uint16 sample_count                 # 采样点数
uint16 element_total                # 阵元总数 (全聚焦模式需要)

# ---- 状态 ----
uint16 status_flags                 # 见 STATUS_* 常量
uint32 saturated_count              # 本帧饱和样本数

# ---- 数据 ----
# 轴序由 PautConfig.scan_mode 决定:
#   SCAN_MODE_LINEAR / SCAN_MODE_SECTOR:
#       [beam_count][sample_count]                     行优先, 波束为外层轴
#   SCAN_MODE_FMC:
#       [element_total][element_total][sample_count]   行优先, [发射][接收][采样]
# 元素实际类型由 PautConfig.sample_dtype 声明; 本字段统一以 uint16 承载。
uint16[] samples

# 逐样本饱和掩码: 每样本 1 bit, 与 samples 同轴序, 行优先。
# 空数组 = 设备未提供。长度应为 ceil(元素数/8)。
uint8[] saturation_mask

# ---- 常量 ----
uint16 STATUS_HAS_DATA                = 1     # 本帧数据有效
uint16 STATUS_SATURATED               = 2     # 本帧存在饱和样本
uint16 STATUS_NO_SYNC                 = 4     # 编码器/触发失同步
uint16 STATUS_ENCODER_VALID           = 8     # 编码器值有效
uint16 STATUS_CALIBRATED              = 32    # 采样率/声速等标定量齐备
uint16 STATUS_HAS_DEVICE_TIMESTAMP    = 64    # header.stamp 来自设备时戳
```

> **线格式的 `STATUS_IS_FRAGMENTED` 在 ROS 侧被移除**：分片是 UDP 的传输层问题，DDS 已自行处理。帧块里的 `frag_index` / `frag_count` 同理不需要进消息。

---

## 3. 字段映射（线格式 v1 → ROS 2）

### 3.1 采集头 → `PautConfig`

| 线格式字段 | ROS 字段 | 类型变化 | 备注 |
|---|---|---|---|
| `magic` / `version_*` / `packet_len` / `header_len` / `crc32` | — | **丢弃** | 传输层职责，DDS 已保证 |
| `board_type` … `sample_count` | `board_type` … `sample_count` | 同类型 | 板卡段原样 |
| `element_group` … `first_element_offset_mm` | 同名 | 同类型 | 探头段原样 |
| `wedge_angle_deg` / `wedge_velocity_m_s` | 同名 | 同类型 | 模块段原样 |
| `main_offset_mm` … `step_angle_or_aperture` | 同名 | 同类型 | 扫查段原样 |
| `reserved` / `reserved2` | — | **丢弃** | 线格式对齐填充，ROS 无意义 |
| `sampling_rate_hz` / `initial_time_us` / `adc_bits` / `sample_dtype` | 同名 | 同类型 | ★ 时间基准 |
| `encoder_resolution_mm` / 双极性 | 同名 | 同类型 | ★ 位置基准 |
| — | `config_seq` | **新增** | ROS 侧配对用 |
| — | `header` | **新增** | ROS 惯例 |

### 3.2 帧块 + 数据段 → `PautFrame`

| 线格式字段 | ROS 字段 | 类型变化 | 备注 |
|---|---|---|---|
| `frame_seq` | `frame_seq` | 同名 u64 | |
| `device_timestamp_ns` | `device_timestamp_ns` | 同名 u64 | |
| — | `header.stamp` | **新增** | 设备时戳优先，无则收包时刻 |
| `scan_axis_encoder` / `index_axis_encoder` | 同名 | 同名 i32 | |
| `beam_count` / `sample_count_f` / `element_total_f` | `beam_count` / `sample_count` / `element_total` | 去掉 `_f` 后缀 | 帧块内不再冗余，ROS 里就是唯一来源 |
| `status_flags` / `saturated_count` | 同名 | 同名 | |
| `frag_index` / `frag_count` | — | **丢弃** | 传输层职责 |
| 数据段 | `samples` | `u16[]` | 展平 + 显式维度 |
| 饱和掩码段 | `saturation_mask` | `u8[]` | 空 = 未提供 |

---

## 4. 话题与 QoS

| 话题 | 消息 | QoS | 说明 |
|---|---|---|---|
| `/inspection/paut/config` | `inspection_interfaces/msg/PautConfig` | `transient_local`<br>`reliable`<br>`keep_last(1)` | 晚加入的订阅者自动补发 |
| `/inspection/paut/raw` | `inspection_interfaces/msg/PautFrame` | `best_effort`<br>`keep_last(100)` | 与现状一致 |

### 4.1 录制配置（ROS 2 Jazzy 实测结论）

> **本节结论经 Jazzy 实测验证，非推测。** 实验：发布端以 `TRANSIENT_LOCAL + RELIABLE` 发一次即停止，录制器在其之后才启动，看能否抓到。

**`ros2 bag record` 的默认订阅 QoS 本身就是 `TRANSIENT_LOCAL`**（`ros2 topic info -v` 实测）：

```
Node name: rosbag2_recorder
  Reliability: RELIABLE
  History (Depth): KEEP_ALL
  Durability: TRANSIENT_LOCAL
```

因此**不写任何 QoS 覆盖也能正常抓到配置消息**。三组对照：

| `qos_overrides.yaml` 中的写法 | 抓到的消息数 |
|---|---|
| 不写该话题（走默认） | ✅ 1 |
| 显式 `durability: transient_local` | ✅ 1 |
| 显式 `durability: volatile` | ❌ **0** |

> ⚠️ **真正的陷阱在这里**：本项目现有 `qos_overrides.yaml` 中，除 `/tf_static` 外**每一条都显式写了 `durability: volatile`**。若照抄邻条目为 `/inspection/paut/config` 加一条，**反而会把这个默认行为破坏掉**——表现为 bag 里只有数据、没有配置，且**不报任何错**。
>
> **规则：要么不写这条，要么写就必须是 `transient_local`。**

`src/inspection_bringup/config/record_topics.yaml` 仍需在 `topics:` 列表（第 12 行 `/inspection/paut/raw` 附近）新增，否则录制器不会订阅该话题：

```yaml
  - /inspection/paut/config
```

**回放侧同样实测通过**：`ros2 bag play` 播放含 1 条 config + 44 条数据的 bag，订阅者在播放开始 **3 秒后**才加入，仍成功收到 config——player 按录制时记录的 `offered_qos_profiles` 以 `transient_local` 发布。

（可选加固：驱动周期性重发同一 `config_seq` 的配置，可对 QoS 误配免疫。实测默认已正常，非必需。）

### 4.2 `config_seq` 的生命周期

| 场景 | 行为 |
|---|---|
| 驱动启动 | 发布 `PautConfig`，`config_seq = 1` |
| 配置未变 | 不重发；`PautFrame.config_seq` 恒为 1 |
| 设备端配置变更（如换探头） | 重发 `PautConfig`，`config_seq += 1`；后续帧携带新值 |
| 驱动重启 | `config_seq` 归 1，且 `PautConfig` 重新发布 |

> `config_seq` **仅在同一采集会话内唯一**。跨会话需按 bag 文件区分，不要跨 bag 直接比较。

---

## 5. 与当前 `PautFrame` 的差异

`src/inspection_interfaces/msg/PautFrame.msg` 现有定义（v0）：

```
std_msgs/Header header
uint64 sequence
string sensor_id
string calibration_id
uint32 channel_count        # ← 实际是深度点数 (167)
uint32 sample_count         # ← 实际是波束数 (61)
float32 sampling_rate_hz
float32 gain_db
float32 sound_velocity_m_s
string sample_encoding
int32[] samples
```

| 变更 | 说明 |
|---|---|
| `channel_count` → 拆分 | 原值 167 是**深度**；拆为 `PautConfig.sample_count` |
| `sample_count` → 重定义 | 原值 61 是**波束**；改名 `beam_count`（`sample_count` 让给真实采样点数） |
| `sequence` → `frame_seq` | 语义澄清：是设备帧号，非主机计数 |
| `sensor_id` / `calibration_id` | 保留，建议移入 `PautConfig` |
| `sample_encoding` → `sample_dtype` | 由字符串改为枚举 |
| `gain_db` / `sound_velocity_m_s` | → `PautConfig`；`sound_velocity_m_s` 与新增的 `specimen_velocity_m_s` 需**合并**，避免两个声速字段打架 |
| `int32[] samples` → `uint16[] samples` | 类型修正 + **带宽减半** |
| — | 新增 `config_seq` / `device_timestamp_ns` / 双轴编码器 / 显式维度 / 状态位 |

### 5.1 迁移代价

**改 `PautFrame` 会使类型哈希变化，已有 `bags/paut_real`（958 帧）无法用新类型反序列化。**

三条路，按推荐排序：

| 方案 | 做法 | 代价 |
|---|---|---|
| **A（推荐）** | 新定义落到**新类型名** `PautFrameV2`，旧 `PautFrame` 冻结保留 | 无。两套类型并存，旧 bag 照常可读 |
| B | 直接改 `PautFrame`，接受旧 bag 失效 | 丢失已有的 958 帧真机数据 |
| C | 改 `PautFrame` 并同步写一次性转换脚本升级旧 bag | 工作量最大，但类型名保持干净 |

---

## 6. 落地改动清单

| 文件 | 改动 |
|---|---|
| `src/inspection_interfaces/msg/PautConfig.msg` | **新增** |
| `src/inspection_interfaces/msg/PautFrame.msg` | 改写（或新增 `PautFrameV2.msg`，见 §5.1） |
| `src/inspection_interfaces/CMakeLists.txt` | `rosidl_generate_interfaces` 中加入 `PautConfig.msg` |
| `src/paut_driver/include/paut_driver/paut_udp_receiver.hpp` | 解析 v1 报文（封装 + 采集头 + 帧块 + 数据段） |
| `src/paut_driver/src/paut_udp_receiver.cpp` | 按偏移解析；**显式小端**，替换现有主机字节序 `memcpy`（`paut_udp_receiver.cpp:118`） |
| `src/paut_driver/src/paut_driver_node.cpp` | 新增 `config_pub_`（transient_local）；`config_seq` 变更检测；`onFrame` 填充新字段 |
| `src/paut_driver/config/paut_params.yaml` | 新增标定参数（`sampling_rate_hz` / `initial_time_us` / 编码器分辨率等），替代现有硬编码默认值 |
| `src/inspection_bringup/config/record_topics.yaml:12` | 增加 `/inspection/paut/config` |
| `src/inspection_bringup/config/qos_overrides.yaml:51` | 增加 `/inspection/paut/config` 的 transient_local QoS |
| `src/inspection_tools/inspection_tools/validators.py` | 校验：`config_seq` 配对、维度自洽、`status_flags` 一致性 |
| `scripts/send_paut_frames.py` | 合成源升级到 v1 报文，用于无真机冒烟 |

---

## 7. 预期收益

| 指标 | 现状 (v0) | v1 (u16) | 变化 |
|---|---|---|---|
| 单帧数据量 | 40,748 B | 20,374 B | **−50%** |
| 袋大小（22 s / 958 帧） | 4.0 MiB | ≈ 2.0 MiB | **−50%** |
| 未压缩速率 | 6.4 GB/h | 3.2 GB/h | **−50%** |
| 深度可解释性 | ❌ 无法换算 | ✅ `sampling_rate` + `initial_time` + `velocity` | 从「无量纲数组」变为「毫米坐标」 |
| 位置可解释性 | ❌ 无 | ✅ 双轴编码器 + 分辨率 | 多模态融合成为可能 |
| 饱和可检测 | ❌ 靠 `==255` 硬判 | ✅ `saturated_count` + 掩码 | 消除误判 |
| 多模态对齐 | ❌ 仅主机收包时刻 | ✅ 设备单调时戳 | 精度提升 |

---

## 8. 待确认

| ID | 问题 | 影响 |
|---|---|---|
| Q1 | §5.1 选 A / B / C 哪个迁移方案 | 决定是否需要转换脚本、旧 bag 是否保留 |
| Q2 | `specimen_velocity_m_s` 与现有 `sound_velocity_m_s` 是否同一物理量 | 若是则合并；若否（如一个指楔块一个指试件）需各自命名 |
| Q3 | 全聚焦模式下 `samples` 是否仍用 `uint16`（342 KB/帧 × 43.5 Hz 的带宽） | 可能需在消息层限制 FMC 帧率 |
| Q4 | `saturation_mask` 是否纳入 v1 首发，还是留待设备端具备算力后再加 | 影响一次性改造的工作量 |
| Q5 | `sensor_id` / `calibration_id` 放 `PautConfig` 还是保留在 `PautFrame` | 前者更合理（属配置），但会影响 `validators.py` 现有校验 |

---

*相关文档：`docs/PAUT_DATA_FORMAT_SPEC_ZH.md`（线格式 v1）、`docs/PAUT_DATA_STORAGE_FORMATS_ZH.md`（行业规范对照）、`docs/image.png`（厂商原始格式表）*
