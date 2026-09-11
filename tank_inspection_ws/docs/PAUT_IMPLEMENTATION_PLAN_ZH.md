# PAUT 协议落地开发任务清单

- **文档日期**：2026-09-11
- **状态**：待开发
- **上游依据**：
  - `docs/PAUT_DATA_FORMAT_SPEC_ZH.md` —— 设备 → 主机 UDP 线格式 v1
  - `docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md` —— ROS 2 消息层规格 v1
  - `docs/PAUT_DATA_STORAGE_FORMATS_ZH.md` —— 行业规范对照
- **本文用途**：把上述规格拆成可执行、有依赖顺序的任务，供后续开发直接取用

---

## 0. 任务总览

```
阶段 0  设备端确认（外部依赖，阻塞阶段 2）
   ↓
阶段 1  不依赖设备端 ← 可立即开工
   ↓
阶段 2  依赖设备端 v1 报文
   ↓
阶段 3  验收（V1–V9）
   ↓
阶段 4  可选增强
```

| 阶段 | 任务数 | 可立即开工 | 阻塞于 |
|---|---|---|---|
| 0 设备端确认 | Q1–Q7 | — | 设备方答复 |
| 1 采集端骨架 | T1–T7 | ✅ | 无 |
| 2 报文接通 | T8–T11 | ❌ | 阶段 0 答复 + 设备端实现 |
| 3 验收 | V1–V9 | ❌ | 阶段 2 |
| 4 可选增强 | T12–T14 | ❌ | 阶段 3 |

---

## 1. 阶段 0：设备端确认（阻塞项）

> 完整问题清单见 `docs/PAUT_DATA_FORMAT_SPEC_ZH.md` §14。**其中 Q3 / Q4 / Q5 直接决定协议设计，须优先取得答复。**

| ID | 问题 | 影响 |
|---|---|---|
| Q1 | 采样率是否设备原生可提供 | 决定 §4.6 是填真值还是靠试件深度反推 |
| Q2 | `initial_time`（时基起点）能否提供 | 深度换算的必要条件 |
| Q3 | **样本值域为何仅 `0–255`？ADC 实际位深？** | ⚠️ 潜在最大收益：可能只传了低 8 位 |
| Q4 | 全聚焦结构是否为 `[发射][接收][采样点数]` | 决定 FMC 数据段布局 |
| Q5 | 全聚焦的物理链路（WiFi 撑不住 119 Mbit/s） | 决定 FMC 可行性 |
| Q6 | 编码器计数单位与分辨率 | ⑤ 位置轴能否使用 |
| Q7 | TOFD / PWI 的数据段结构 | 决定是否预留 |

**并行事项**：确认能否开放**未波束合成的原始阵元数据**——厂商表显示 `扫查模式` 含 `全聚焦`，即 FMC 全矩阵可得，这是后续自研重建算法的前提。

---

## 2. 阶段 1：不依赖设备端（可立即开工）

### T1 — 新增 `PautConfig.msg`

| 项 | 内容 |
|---|---|
| 文件 | `src/inspection_interfaces/msg/PautConfig.msg` |
| 内容 | 见 `docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md` §2.1 |
| 验收 | `colcon build` 通过；`ros2 interface show inspection_interfaces/msg/PautConfig` 输出符合规格 |
| 依赖 | 无 |

### T2 — 新增 `PautFrameV2.msg`（**不修改**旧 `PautFrame`）

| 项 | 内容 |
|---|---|
| 文件 | `src/inspection_interfaces/msg/PautFrameV2.msg`（新文件） |
| 内容 | 见 `docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md` §2.2 |
| 风险 | ⚠️ **不要直接改 `PautFrame.msg`**——会使类型哈希变化，`bags/paut_real`（958 帧真机数据）无法反序列化。方案 A 见 `docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md` §5.1 |
| 验收 | 新旧类型并存，`ros2 bag info bags/paut_real` 仍正常 |
| 依赖 | 无 |

### T3 — 注册新消息

| 项 | 内容 |
|---|---|
| 文件 | `src/inspection_interfaces/CMakeLists.txt` |
| 改动 | `rosidl_generate_interfaces` 中加入 `msg/PautConfig.msg`、`msg/PautFrameV2.msg` |
| 验收 | 两个新类型可由 `ros2 interface show` 列出 |
| 依赖 | T1、T2 |

### T4 — 录制话题清单加入 config

| 项 | 内容 |
|---|---|
| 文件 | `src/inspection_bringup/config/record_topics.yaml`（第 12 行 `/inspection/paut/raw` 附近） |
| 改动 | 新增 `- /inspection/paut/config` |
| 验收 | `ros2 bag record` 日志中出现 "Subscribed to topic '/inspection/paut/config'" |
| 依赖 | 无 |

### T5 — QoS 覆盖：**保持不动** ⚠️

| 项 | 内容 |
|---|---|
| 文件 | `src/inspection_bringup/config/qos_overrides.yaml` |
| 改动 | **不新增 `/inspection/paut/config` 条目** |
| 原因 | 实测 `rosbag2` 默认订阅 QoS 已是 `TRANSIENT_LOCAL`，不写即正常。而本文件其余条目**均显式写了 `durability: volatile`**，照抄会让配置消息 100% 抓不到且**不报错** |
| 若确有需要显式声明 | 必须写 `durability: transient_local`，**绝不能是 volatile** |
| 验收 | 录制后 `ros2 bag info` 中 `/inspection/paut/config` 计数 ≥ 1 |
| 依赖 | 无 |

### T6 — 驱动端配置发布骨架（数据源先用参数，不接 UDP）

| 项 | 内容 |
|---|---|
| 文件 | `src/paut_driver/src/paut_driver_node.cpp`、`include/paut_driver/paut_driver_node.hpp` |
| 改动 | ① 新增 `config_pub_`，QoS = `transient_local + reliable + keep_last(1)`；② 启动时从 ROS 参数构造 `PautConfig` 并发布，`config_seq = 1`；③ 实现 `configEquals()` 与 `publishConfigIfChanged()`（`docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md` §5 伪代码） |
| 目的 | **不依赖设备端**即可打通"配置存储"链路并验证 |
| 验收 | 录制一段 bag，`ros2 bag info` 中 `/inspection/paut/config` 恰好 1 条；`ros2 topic echo /inspection/paut/config` 晚于发布启动仍能收到 |
| 依赖 | T1、T3 |

### T7 — 参数文件补齐标定量

| 项 | 内容 |
|---|---|
| 文件 | `src/paut_driver/config/paut_params.yaml` |
| 改动 | 新增 `sampling_rate_hz` / `initial_time_us` / `adc_bits` / `sample_dtype` / `encoder_resolution_mm` 等；**未知项填 `NaN`（浮点）或专用枚举，不得填 0** |
| 原因 | 设计原则 P6：零值与未知物理含义不同 |
| 验收 | 参数文件加载无报错；未填项在消息中表现为 `NaN` |
| 依赖 | T6 |

---

## 3. 阶段 2：接通 v1 报文（依赖设备端）

### T8 — UDP 解析 v1 报文

| 项 | 内容 |
|---|---|
| 文件 | `src/paut_driver/src/paut_udp_receiver.cpp`、`include/paut_driver/paut_udp_receiver.hpp` |
| 改动 | 按 `docs/PAUT_DATA_FORMAT_SPEC_ZH.md` §3–§7 解析：包封装 → 采集头 → 帧块 → 数据段 →（可选）饱和掩码 |
| 必做 | ① **显式小端解析**，替换现有的主机字节序 `memcpy`（`paut_udp_receiver.cpp:118`）；② `magic` / `crc32` 校验；③ `packet_len` 与维度自洽校验 |
| 兼容 | 按 `magic` 自动识别 v0 / v1，**支持设备端双发**，便于灰度切换 |
| 验收 | 收到 v1 报文时字段全部正确填充；v0 报文仍能解析 |
| 依赖 | Q1–Q7 答复中至少 Q3、Q4 |

### T9 — `config_seq` 变更检测接通

| 项 | 内容 |
|---|---|
| 文件 | `src/paut_driver/src/paut_driver_node.cpp` |
| 改动 | 将 T6 的骨架数据源从 ROS 参数切换为**从 v1 采集头解析**；采集头变化时递增 `config_seq` 并重发 |
| 验收 | 采集期间修改设备端配置，bag 中出现第 2 条 `/inspection/paut/config`，`config_seq = 2`，后续帧携带新值 |
| 依赖 | T6、T8 |

### T10 — 合成推流源升级到 v1

| 项 | 内容 |
|---|---|
| 文件 | `scripts/send_paut_frames.py` |
| 改动 | 按 v1 线格式构造报文，用于无真机冒烟 |
| 验收 | 启动合成源 + `paut_capture.launch.py`，bag 中字段与规格一致 |
| 依赖 | T8 |

### T11 — 端到端冒烟

| 项 | 内容 |
|---|---|
| 步骤 | 合成源 → `paut_capture.launch.py` → `ros2 bag info` → 逐字段核对 |
| 验收 | 见阶段 3 的 V1、V2 |
| 依赖 | T9、T10 |

---

## 4. 阶段 3：验收

按 `docs/PAUT_DATA_FORMAT_SPEC_ZH.md` §10 逐条执行。**V3 / V4 / V7 是决定数据能否用于定量分析的卡口，不可跳过。**

| ID | 验证项 | 关键性 |
|---|---|---|
| V1 | 报文完整性（magic / crc32） | |
| V2 | 维度自洽（`packet_len` 等式） | |
| **V3** | **采样率正确性**（已知厚度试块反推，误差 < 2%） | 🔴 卡口 |
| **V4** | **动态范围**（值域是否突破 255 → 判定 Q3 是否解决） | 🔴 卡口 |
| V5 | 编码器有效性（分辨率 × Δcount ≈ 实际位移） | |
| V6 | 声速正确性（反算深度 ≈ 试块厚度） | |
| **V7** | **轴序正确性**（扫过台阶，回波应沿采样轴平移） | 🔴 卡口 |
| V8 | 状态位（人为饱和 → `saturated_count > 0`） | |
| V9 | 全聚焦分片（若启用） | |

---

## 5. 阶段 4：可选增强

| ID | 任务 | 说明 |
|---|---|---|
| T12 | HDF5 exporter（ONDE 风格） | 按 `docs/PAUT_DATA_STORAGE_FORMATS_ZH.md` §4 六类逐项映射；缺失项显式写未知 |
| T13 | 全聚焦模式支持 | ⚠️ 须先解决 Q5 的物理链路；数据量 342 KB/帧、119 Mbit/s，需分片（帧块已预留 `frag_index`/`frag_count`） |
| T14 | 逐样本饱和掩码 | 设备端具备算力后再启用；开销约 6.3% |

---

## 6. 本次已发现的既有缺陷（不依赖设备端，建议尽早修）

| # | 位置 | 问题 | 严重度 |
|---|---|---|---|
| 1 | `src/inspection_interfaces/msg/PautFrame.msg:2` | 轴语义注释**正好写反**（称 167=channel/61=depth） | HIGH |
| 2 | `PautFrame.msg` | 字段名 `channel_count`(167) / `sample_count`(61) 与实际语义相反 | HIGH |
| 3 | `paut_udp_receiver.cpp:118` | 按**主机字节序** `memcpy`，大小端异构时静默出错 | MEDIUM |
| 4 | `paut_udp_receiver.cpp:137` 注释 | 称"目标 channel-major"，实际产出 depth-major | MEDIUM |

> 缺陷 1 / 2 的修正时机：若采 T2 的 `PautFrameV2` 方案，旧类型原样冻结，**仅需在旧 `.msg` 上加一行"本类型轴语义有误，见 PautFrameV2"的警示注释**即可，零风险。

---

## 7. 已知陷阱速查

| 陷阱 | 后果 | 对策 |
|---|---|---|
| `qos_overrides.yaml` 给 config 写 `volatile` | 配置消息**静默丢失** | 不写该条目；写就必须 `transient_local`（见 T5） |
| 误以为"必须配 transient_local 才行" | 画蛇添足引入风险 | rosbag2 默认已是 `TRANSIENT_LOCAL`（实测） |
| 直接改 `PautFrame.msg` | 旧 bag 类型哈希失效 | 用 `PautFrameV2`（T2） |
| 未知量填 `0` | 零值与"未知"语义混淆 | 填 `NaN` / 专用枚举（T7） |
| `prf_hz` 当作采样率 | 声速/深度量级离谱 | `prf_hz` 是脉冲重复频率（帧率） |
| 假定 `采样点数` ↔ `试件深度` 严格对应 | 深度换算系统性偏差 | 必须经 V3 试块实测验证 |
| 全聚焦按现状带宽上 | WiFi 撑不住，丢帧 | 先解决 Q5（有线） |

---

*相关文档：`docs/PAUT_DATA_FORMAT_SPEC_ZH.md`、`docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md`、`docs/PAUT_DATA_STORAGE_FORMATS_ZH.md`、`docs/image.png`*
