# 多模态检测数据规范 —— 骨架与契约 v0.1

- **文档日期**：2026-09-11
- **状态**：骨架与契约待评审冻结；词典待填充
- **覆盖模态**：多通道超声、相控阵超声（PAUT）、涡流、磁
- **本文定位**：**跨模态顶层规范**。各模态的实例化文档见 `docs/PAUT_*.md`（PAUT 已先行）

---

## 0. 本文只定义两样东西

| 组成 | 内容 | 冻结时机 |
|---|---|---|
| **骨架** | 六类信息中哪些跨模态公共、哪些模态特有；消息如何分层 | **本文即冻结** |
| **契约** | 时间、坐标、单位、未知值、命名、版本、校验 七条规则 | **本文即冻结** |
| **词典** | 各模态每个字段的名称/类型/单位/语义 | ❌ **不在本文**，边采边填 |

> **为什么要这样切**：从单一模态（PAUT）归纳通用规律必然出错——`channel_count` 语义在 PAUT 与超声里相反、`int32` vs `int16` 不一致，都是这么长出来的。骨架和契约不依赖具体字段，可以现在定；**统一语义模型必须等 ≥2 个模态有真机数据才有资格定**（见 §7）。

---

## 1. 分层架构

```
┌──────────────────────────────────────────────────────────┐
│ L3  语义 / 融合层    统一坐标 + 统一时间 + 工件标注        │  训练、缺陷判定
├──────────────────────────────────────────────────────────┤
│ L2  导出规范层       ★ 大一统数据规范在这里落地 ★          │  HDF5 / ONDE 风格
│                     统一 SI 单位、统一轴序、可 join        │
├──────────────────────────────────────────────────────────┤
│ L1  采集层           MCAP + 各模态 ROS 消息 + 公共 meta    │  ← 本文规范这一层
│                     保真、原生单位、模态细节完整            │
├──────────────────────────────────────────────────────────┤
│ L0  设备层           各厂商私有协议 / 私有文件格式          │
└──────────────────────────────────────────────────────────┘
```

**核心判断：统一发生在 L2，不在 L1。**

| | L1 采集层 | L2 导出层 |
|---|---|---|
| 目标 | **保真** + 模态细节完整 | **统一** + 可比 + 可训练 |
| 单位 | 原生（mm / ° / counts） | SI（m / s / rad） |
| 结构 | 各模态独立 | 大一统 |
| 约束 | 无损、可回放 | 一致、可 join |

两个目标**本质冲突**，不能塞进同一个类型。硬塞的结果就是字段名为了"通用"而模糊，语义反了都没人发现。

> **旁证**：行业规范 ONDE 也是这么分的——`/Public/Setup`（元数据）与 `Datasets`（数据）共用一套语义模型但物理分离。

---

## 2. 骨架：六类信息的公共/特有划分

| 类 | 多通道超声 | 相控阵 | 涡流 | 磁 | 归属 |
|---|---|---|---|---|---|
| ② **时间轴** | 采样率、初时 | 采样率、初时 | 采样率、激励周期 | 采样率 | 🟦 **公共** |
| ⑤ **位置轴** | 探头位姿 | 双轴编码器 | 探头位姿 | 探头位姿 | 🟦 **公共** |
| ⑥ **状态位** | 耦合、饱和 | 耦合、饱和 | 提离、饱和 | 饱和 | 🟦 **公共** |
| ① 信号本体 | 波形 A 扫 | B 扫 / FMC | 阻抗 I/Q | 场强 | 🟨 模态特有 |
| ③ 几何 | 阵元 | 阵元 + 聚焦律 | 线圈参数 | 磁传感器 | 🟨 模态特有 |
| ④ 激励 | 脉冲参数 | 偏转律 | 激励频率/幅值 | 磁化强度 | 🟨 模态特有 |

**②⑤⑥ 跨模态完全同构，①③④ 必然各异。**

→ 公共部分抽成 `InspectionMeta`（§4），作为**每个模态消息的首字段**；模态特有字段留在各自消息里。

---

## 3. 七条契约

### C1 — 时间契约 🔴 **最高优先级**

**所有模态的测量时刻必须可追溯到同一时钟域。**

| 规则 | 内容 |
|---|---|
| C1.1 | 每条测量携带 `device_timestamp_ns`（u64，设备单调时钟）。**`0` 表示设备不提供** |
| C1.2 | `header.stamp` 取值：有设备时戳 → 用它；否则用采集主机收包时刻 |
| C1.3 | **必须**用 `timestamp_source` 字段显式标明用的是哪一种（枚举见 §4） |
| C1.4 | **禁止**把主机收包时刻当作测量时刻使用而不加标记——这是静默的精度损失 |
| C1.5 | 必须记录 `timestamp_uncertainty_ns`（时间不确定度）。`TS_HOST_RECEIVE` 的不确定度 = 网络抖动（典型 ms 级） |

> ⚠️ **这是当前最大的架构风险**：PAUT 设备**无硬件时戳**（见 `reports/paut_real_analysis/`）。若涡流与磁的设备同样如此，多模态融合的时间对齐精度就只有网络抖动级别——**融合会名存实亡**。
>
> **本条可能反向约束设备选型/改造**：要么设备端加单调时钟，要么上硬件触发同步（PTP / 触发线）。**建议优先探清再做其他设计。**

### C2 — 坐标契约 🔴 **最高优先级**

**所有模态的位置必须能变换到统一的工件坐标系（workpiece frame）。**

| 规则 | 内容 |
|---|---|
| C2.1 | 公共 meta 中的位置**一律为 SI 米**，且**已变换到工件坐标系** |
| C2.2 | **禁止**把编码器 counts 直接放入 meta 用于融合。counts 留在模态配置里，转换由驱动完成 |
| C2.3 | 变换链必须可由 TF 树还原 → **`/tf` 与 `/tf_static` 必须录制**（当前 `record_topics.yaml` 已含） |
| C2.4 | `header.frame_id` 指向工件坐标系；命名需统一（建议 `workpiece/<asset_id>`） |
| C2.5 | 无法变换时填 `NaN`（见 C4），并清除 `STATUS_ENCODER_VALID` |

> 既有实现参考：`FusionIndex.msg` 的 `u_m` / `v_m` / `s_m` / `n_m` 已是这条契约的雏形。

### C3 — 单位契约

| 层 | 单位策略 |
|---|---|
| **公共 meta** | **强制 SI**：m / s / rad / N / °C / kg。字段名**不带**单位后缀 |
| **模态载荷** | **保留原生单位**，字段名**内嵌**单位后缀（`_mm` / `_ns` / `_deg`） |

| 规则 | 内容 |
|---|---|
| C3.1 | meta 里出现 `_mm` / `_counts` / `_deg` 即为违规 |
| C3.2 | 所有单位必须可追溯到 SI（纯比例或仿射变换），**不允许非线性单位** |
| C3.3 | 角度在 meta 中用 **rad**；在载荷中可保留 `_deg` |
| C3.4 | 参考 REP-103：**共享接口守标准，保真载荷不守**——这是有意的、有边界的偏离 |

### C4 — 未知值契约

| 类型 | 表示 |
|---|---|
| 浮点未知 | `NaN`（**绝不填 0**） |
| 结构体未知（如四元数） | 各分量全 `NaN` |
| 整数未知 | 用**状态位**声明无效，而非 magic number |
| 字符串未知 | `"UNASSIGNED"`（沿用现状，全仓统一） |
| 数组未知 | 空数组（长度为 0） |
| 时间未知 | `device_timestamp_ns = 0` + `TS_UNKNOWN` |

**校验规则**：消费端必须能区分"未知"与"真实为零"。二者物理含义完全不同，混淆会产生静默的错误统计。

### C5 — 命名契约

| 规则 | 内容 |
|---|---|
| C5.1 | 字段名 `snake_case`（ROS 2 规范） |
| C5.2 | 单位后缀白名单：`_m` `_mm` `_s` `_ns` `_us` `_hz` `_khz` `_deg` `_rad` `_v` `_n` `_c` `_db` `_counts` |
| C5.3 | 序号/计数：`*_seq`，必须单调递增 |
| C5.4 | 布尔：`is_*` / `has_*` / `*_valid` |
| C5.5 | **禁止跨模态复用同名但语义不同的字段** ⚠️ 见 §6 缺陷 D1 |
| C5.6 | 模态原始数据数组的名称由模态自定（语义不同），但**必须在消息注释里显式声明轴序** |

### C6 — 版本与扩展契约

| 规则 | 内容 |
|---|---|
| C6.1 | 每个消息含 `schema_version`（u16） |
| C6.2 | 新增字段只能**追加在末尾** |
| C6.3 | 删除字段须先标记 deprecated ≥ 1 个版本 |
| C6.4 | **类型变更 = 新类型名**（`*V2`），**不改老类型** —— ROS 2 类型哈希会失效，老 bag 无法反序列化 |
| C6.5 | 枚举一律用消息内 `const` 声明，禁止裸整数 |

### C7 — 校验契约

| 规则 | 内容 |
|---|---|
| C7.1 | 维度自洽：元素数 = 各维度之积 |
| C7.2 | `config_seq` 必须能在对应 config 话题中找到 |
| C7.3 | 状态位与字段值一致（如 `STATUS_SATURATED` 置位 ⟹ `saturated_count > 0`） |
| C7.4 | 时间戳同源单调 |
| C7.5 | 校验器统一落在 `src/inspection_tools/inspection_tools/validators.py`，逐模态扩展，**不 required**（话题存在才校验） |

---

## 4. 公共元数据消息 `InspectionMeta`

> 新增于 `src/inspection_interfaces/msg/InspectionMeta.msg`。所有模态消息的**首字段**。
> **本消息内一律 SI 单位**（C3）。

```
# 跨模态公共元数据 —— 所有检测模态消息的首字段。
# 单位契约: 本消息内一律 SI (m, s, rad, N, degC)。详见 docs/INSPECTION_DATA_SPEC_ZH.md

std_msgs/Header header              # frame_id 指向工件坐标系; stamp 语义见 timestamp_source

uint32 schema_version               # 本消息结构版本

# ---- 时间 (契约 C1) ----
uint64 device_timestamp_ns          # 设备单调时钟 [ns]; 0 = 设备不提供
uint8  timestamp_source             # 见 TS_* 常量
uint32 timestamp_uncertainty_ns     # 时间不确定度 [ns]; 0xFFFFFFFF = 未知

# ---- 位置 (契约 C2, 已变换到工件坐标系, SI) ----
float64 pos_u_m                     # 工件坐标 U [m]; NaN = 未知
float64 pos_v_m                     # 工件坐标 V [m]; NaN = 未知
float64 pos_w_m                     # 工件坐标 W [m]; NaN = 未知
geometry_msgs/Quaternion orientation  # 探头朝向; 各分量全 NaN = 未知

# ---- 状态 (契约 C4/C7) ----
uint16 status_flags                 # 见 STATUS_* 常量
uint32 saturated_count              # 饱和样本/通道数

# ---- 配置引用 ----
uint32 config_seq                   # 关联本模态 config 话题; 见各模态规范

# ---- 常量 ----
uint8 TS_UNKNOWN          = 0
uint8 TS_DEVICE_HARDWARE  = 1       # 设备硬件时戳 (精度最高)
uint8 TS_DEVICE_SOFTWARE  = 2       # 设备软件时戳
uint8 TS_HOST_RECEIVE     = 3       # 主机收包时刻 (精度最低)

uint16 STATUS_HAS_DATA      = 1
uint16 STATUS_SATURATED     = 2
uint16 STATUS_NO_SYNC       = 4
uint16 STATUS_ENCODER_VALID = 8
uint16 STATUS_CALIBRATED    = 32
uint16 STATUS_CONTACT_VALID = 128
uint16 STATUS_COUPLING_GOOD = 256
```

**关键决策：身份信息不放入 meta。**

`sensor_id` / `calibration_id` / `asset_id` / `probe_id` 属**配置类**（一次会话内不变），应放入各模态的 config 话题，由 `config_seq` 关联。理由与 PAUT 侧一致（见 `docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md` §1 D1）。

> 过渡期说明：现有三个模态消息都把 `sensor_id` / `calibration_id` 放在**逐帧消息**里。迁移时移入 config，不保留双份。

### 4.1 `ProbeState` 的归属

`ProbeState`（`contact_force_n` / `lift_off_mm` / `temperature_c` / `coupling_score`）**保持独立话题**，理由：**更新率与数据帧不同**，且可能被多个模态共享。

meta 中的 `STATUS_CONTACT_VALID` / `STATUS_COUPLING_GOOD` 位作为**快速判据**；详细信息去 `ProbeState` 取。

---

## 5. 各模态消息结构模板

```
<Modality>Frame.msg:
    InspectionMeta meta        ← 公共，固定为第一字段
    <模态特有字段>              ← 载荷，原生单位
```

### 5.1 现状 vs 目标

| 模态 | 现状 | 目标 |
|---|---|---|
| PAUT | `PautFrame`（轴语义与字段名有误） | `PautFrameV2` + `PautConfig`，见 `docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md` |
| 多通道超声 | `UltrasoundFrame` | `UltrasoundFrameV2` + `UltrasoundConfig` + `meta` |
| 涡流 | `EddyCurrentFrame` | `EddyCurrentFrameV2` + `EddyCurrentConfig` + `meta` |
| 磁 | 尚无 | 按模板新建 |

---

## 6. 现状差距与既有缺陷

### 6.1 公共字段已在三个模态中重复

| 字段 | EddyCurrent | Ultrasound | PAUT |
|---|---|---|---|
| `header` / `sequence` | ✓ | ✓ | ✓ |
| `sensor_id` / `calibration_id` | ✓ | ✓ | ✓ |
| `channel_count` / `sample_count` | ✓ | ✓ | ✓ ⚠️ |
| `sampling_rate_hz` / `gain_db` | ✓ | ✓ | ✓ |
| `sample_encoding` | — | ✓ | ✓ |

**约 10 个字段逐字重复**——这正是收敛到 `InspectionMeta` 的经验依据。

### 6.2 既有缺陷

| # | 位置 | 问题 | 严重度 |
|---|---|---|---|
| **D1** | `PautFrame.msg` | `channel_count`(167) / `sample_count`(61) 的语义**与其余模态相反**（PAUT 中 167 是深度、61 是波束），违反 C5.5 | 🔴 HIGH |
| **D2** | `PautFrame.msg:2` | 轴语义注释**正好写反** | 🔴 HIGH |
| D3 | `PautFrame` vs `UltrasoundFrame` | `samples` 类型不一致（`int32[]` vs `int16[]`），且无类型声明 | 🟡 MEDIUM |
| D4 | 全部三个消息 | 无 `timestamp_source`，无法区分设备时戳与主机收包时刻，违反 C1.3 | 🟡 MEDIUM |
| D5 | 全部三个消息 | 无 `schema_version`，违反 C6.1 | 🟢 LOW |
| D6 | `paut_udp_receiver.cpp:118` | 按主机字节序 `memcpy`，大小端异构时静默出错 | 🟡 MEDIUM |

> **D1 是最危险的**：同一字段名在两个模态指不同东西，**会静默产生错误数据**，且逐模态扩展后这类分叉会指数增长。这是"必须先把契约冻结再加模态"的最强论据。

---

## 7. 成熟路线图

```
现在   ① 冻结骨架 + 契约（本文）                        ← 纯文档，半天
       ② 抽出 InspectionMeta，三个模态消息接入            ← 小改动
       ③ 修 D1/D2 缺陷，PAUT 对齐契约                     ← 不等设备端
   ↓
接着   ④ 涡流接真机：只遵守契约，字段自由                ← 词典开始生长
       ⑤ 探清时间契约可行性（设备时戳）← 🔴 阻塞 L2/L3
   ↓
然后   ⑥ 涡流真机数据到位 → 两个模态对照                 ← 唯一有资格定统一语义模型的时刻
   ↓
最后   ⑦ 写 exporter，大一统规范在 L2 导出层落地
```

**关键约束：不要跳步。** 第 ⑥ 步之前定义的"统一语义模型"都是把 PAUT 的特例误当通用规律。

---

## 8. 待定事项

| ID | 事项 | 阻塞 |
|---|---|---|
| Q1 | 工件坐标系命名与变换链定义（C2.4） | L2/L3 全部 |
| Q2 | 各模态设备能否提供硬件时戳（C1） | 🔴 多模态融合能否成立 |
| Q3 | `timestamp_uncertainty_ns` 的具体取值规则 | C1.5 |
| Q4 | `geometry_msgs` 依赖是否可接受（`InspectionMeta` 引入） | 消息定义 |
| Q5 | 磁模态的具体所指（漏磁 MFL / 其他）——影响 ①③④ 词典 | 该模态接入 |
| Q6 | 迁移方案：原地改 vs 新增 `*V2`（C6.4 倾向后者） | 各模态接入 |

---

*相关文档：`docs/PAUT_DATA_FORMAT_SPEC_ZH.md`（PAUT 线格式）、`docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md`（PAUT 消息层）、`docs/PAUT_DATA_STORAGE_FORMATS_ZH.md`（行业规范对照）、`docs/PAUT_IMPLEMENTATION_PLAN_ZH.md`（开发任务）、`docs/README.md`（目录索引）*
