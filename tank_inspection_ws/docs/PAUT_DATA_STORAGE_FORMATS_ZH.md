# 超声相控阵（PAUT）数据存储行业规范对照

- **文档日期**：2026-09-11
- **用途**：为 `tank_inspection_ws` 的超声数据契约与导出格式选型提供行业参照
- **取材**：公开规范文档与厂商中立格式文档（详见 §9 参考来源）。**付费标准（ASTM）未取得全文**，相关条目仅按公开摘要记述，引用前须核对原文。

---

## 1. 规范总览

| 规范 | 主导方 | 容器 | 面向 | 现行状态 |
|---|---|---|---|---|
| **DICONDE** | ASTM（E2339 通用 / E2663 超声专用） | DICOM 文件格式（二进制标签流） | 全模态 NDE 归档与厂商间互操作 | 现行，E2663-25 |
| **ONDE**（NDE Open File Format） | 厂商中立联盟（Evident 等参与） | **HDF5** | 相控阵/UT 检测数据与重建结果 | 现行，文档版本 3.3 / 4.0 / 4.2 |
| **UFF**（Ultrasound File Format） | 学术界（USTB / pyuff 生态） | MATLAB `.mat` 结构体 | 超声研究数据交换 | v0.2 / v0.3 |
| **MFMC-CFF** | 布里斯托大学 NDT 组 | **HDF5** | 全矩阵捕获 FMC / 多帧 MFMC | v2.0.0；FMC 部分标注为实验性 |
| **MCAP / ROS 2** | ROS 2 生态 | MCAP | 机器人多模态时序采集 | 本工作区自用，非 NDE 行业标准 |

**关键分野**：DICONDE 面向**检测结果归档**（合规、可追溯、跨厂商打开）；ONDE/UFF/MFMC-CFF 面向**信号级数据交换**（可再处理、可重建）。本工作区当前落在后者，但用的是机器人侧的 MCAP 容器。

---

## 2. PAUT 数据必然包含的六类信息

> 本节是对上述规范的**跨规范归纳**，不是任何单一规范的官方分类。

| 类 | 名称 | 含义 | 缺失后果 |
|---|---|---|---|
| ① | **信号本体** | A 扫 / FMC / TFM 数组 | 文件无意义 |
| ② | **时间轴** | 采样率、首样本时刻 | 深度/声程轴无法定义 |
| ③ | **阵列几何** | 阵元数、间距、频率、坐标表 | 无法波束合成 |
| ④ | **聚焦律** | 声速、偏转角、延时、波束位置 | 波束编号无法映射到工件位置 |
| ⑤ | **位置轴** | 探头/扫查位置坐标及步长 | 多模态无法对齐 |
| ⑥ | **状态位** | 饱和 / 丢同步 / 无效点标记 | 只能靠阈值猜，误判真值 |

**覆盖率**（据各规范公开文档）：

| 类 | DICONDE | ONDE | UFF | MFMC-CFF |
|---|---|---|---|---|
| ① 信号本体 | ✅ | ✅ | ✅ | ✅ |
| ② 时间轴 | ✅ | ✅ | ✅ | ✅ |
| ③ 阵列几何 | ✅ | ✅ | ✅ `probe` | ✅ |
| ④ 聚焦律 | ✅ | ✅ `beams[]` | ✅ `sequence` | ✅ |
| ⑤ 位置轴 | ✅ | ✅ U/V/W | ⚠️ 隐含 | ✅ `xVector/zVector` |
| ⑥ 状态位 | ✅ | ✅ `AScanStatus` | ❌ | ❌ |

---

## 3. 逐规范详表

### 3.1 DICONDE（ASTM E2663 + E2339）

| 项目 | 内容 |
|---|---|
| 通用标准 | ASTM E2339《DICONDE》，改编自 DICOM（NEMA PS3 / ISO 12052） |
| 超声专用 | ASTM E2663《…for Ultrasonic Test Methods》 |
| E2663 定义 | **Information Object Definitions + Information Modules + Data Dictionary**（超声专用部分） |
| 容器 | DICOM 文件格式（二进制标签流），**非 HDF5** |
| 标签体系 | 沿用 DICOM 的 `(group, element)` 数据元素寻址 |
| 现行版本 | E2663-25（历任 -23 / -14R18 / -14 / -11 / -08） |
| 归口 | ASTM Committee E07.11；Book of Standards Vol. 03.04；13 页；DOI 10.1520/E2663-25 |
| 明确**不**规定 | 一致性测试/验证程序；厂商实现细节；多设备集成时的完整特性集 |
| 设计目标 | 摆脱厂商私有的传输与存储方式，使数据在技术迭代后仍可读 |

> ⚠️ **本文未列具体标签号**。E2663 为付费标准，公开摘要未给出数据字典条目；如需按 DICONDE 落地，必须取得原文核对标签定义，不建议凭二手资料实现。

### 3.2 ONDE（HDF5，路径与字段已核实）

**HDF5 路径层次**

```
/Public/Setup                                        ← 必填；全部元数据，内容为 JSON 字符串
/Public/Groups/<GroupId>/Datasets/<DatasetId>-<DataClass>   ← 信号本体
```

**`DataClass` 枚举**

| 取值 | 含义 |
|---|---|
| `AScanAmplitude` | A 扫幅值 |
| `AScanStatus` | A 扫状态位 |
| `TfmValue` | TFM 重建值 |
| `TfmStatus` | TFM 状态位 |
| `CScanPeak` | C 扫峰值 |
| `CScanStatus` | C 扫状态位 |
| `CScanTime` | C 扫时间 |
| `FiringSource` | 激发源（文档标注为内部使用） |

**取值编码（关键设计）**：信号本体存**整数**，物理量由量程映射还原。

| 字段 | 说明 | 示例 |
|---|---|---|
| `dataSampling.min` / `.max` | 整数取值域 | `0` / `32767` |
| `dataValue.min` / `.max` | 物理取值域 | `0` / `200` |
| `dataValue.unit` | 物理单位 | `Percent` \| `Coherence` \| `Seconds` |

**状态位**：整数 bitfield，`hasData=1` / `saturated=2` / `noSynchro=4`。

> ⚠️ **规范文档自身存在矛盾**：正文给出上述位值，但示例称「`hasData+saturated` → 十进制 `5`」，而按位或应为 `1|2 = 3`。实现前须以真实文件实测确认。

**维度轴**

| 轴名 | 单位 | 附加字段 |
|---|---|---|
| `UCoordinate` / `VCoordinate` / `WCoordinate` | m | `quantity`（点数）、`resolution`（步长）、`offset`（起点） |
| `Ultrasound`（声程） | s | 同上 |
| `Beam`（波束） | — | 额外含 `beams[]`，见下 |
| `StackedAScan` | — | FMC 专用，各 A 扫按发振顺序串联 |

**`Beam` 轴内 `beams[]` 字段（即聚焦律）**

| 字段 | 单位 | 说明 |
|---|---|---|
| `id` | — | 波束标识 |
| `velocity` | m/s | 声速 |
| `skewAngle` | ° | 偏转角 |
| `refractedAngle` | ° | 折射角 |
| `uCoordinateOffset` / `vCoordinateOffset` | m | 波束落点偏移（仅当楔块面与工件面重合时适用） |
| `ultrasoundOffset` | s | 声程延时 |

**`storageMode` 枚举**：`Independent` | `Paintbrush`。

### 3.3 UFF（`ChannelData` 对象字段）

| 字段 | 类型 | 说明 |
|---|---|---|
| `data` | ndarray | **信号本体**，维度序 `[time, channel, wave, frame]` |
| `N_samples` | int | 时间采样点数 |
| `N_channels` / `N_elements` / `N_active_elements` | int | 通道数 / 阵元数 / 实际收发阵元数 |
| `N_waves` / `N_frames` | int | 发射波数 / 帧数 |
| `sampling_frequency` | float | 采样频率 [Hz] |
| `initial_time` | float | **首个样本的时刻 [s]** |
| `sound_speed` | float | 参考声速 [m/s] |
| `PRF` | float | 脉冲重复频率 [Hz]（**注意：是帧率，非采样率**） |
| `modulation_frequency` | float | 调制频率 [Hz] |
| `wavelength` | float | 波长 [m] |
| `probe` | `Probe` | 阵列几何 |
| `pulse` | `Pulse` | 激励波形 |
| `phantom` | `Phantom` | 试块 |
| `sequence` | `Wave` / `Wave[]` | 发射序列（聚焦律） |
| `name` / `author` / `version` / `reference` / `info` | str | 溯源信息 |

### 3.4 MFMC-CFF（布里斯托大学）

**派生数据（`derived_data.h5`）**

| 组 / 字段 | 形状 | 说明 |
|---|---|---|
| `derived_data.xVector` / `.zVector` | `[NX]` / `[NZ]` | 图像像素坐标向量 |
| `derived_data.sens_maps` | `[2, NZ, NX, NView]` | 每像素每视角的灵敏度；**首维 2 = 实部/虚部** |
| `derived_data.surfaces.curved_fw` | — | 前表面点云：`nfrontwall`、`coordX/coordY/coordZ` |
| `derived_data.surfaces.curved_bw` | — | 后表面点云：同上 |

**备注**

- 该数据集 `NView = 21`，视角顺序按论文图序（左→右、上→下）排列。
- 坐标换算：**文件名中的整数 × 0.0125 = mm**（缩放系数写在文档里而非元数据里——**此为反面教材，缩放系数应写入文件**）。
- `coordY` 在二维（非变 Y）情形下为奇异。

---

## 4. 六类信息 × 规范：字段级对照

### ① 信号本体

| 规范 | 路径 / 字段 | 轴序 | 典型 shape | 典型 dtype |
|---|---|---|---|---|
| DICONDE | UT 信息对象（标签流） | 见 E2663 数据字典 | — | — |
| ONDE | `/Public/Groups/0/Datasets/0-AScanAmplitude` | 空间轴 → Beam → Ultrasound | `[22, 16, 620]` | 整数（文档未明确；`0–32767` 暗示 int16） |
| UFF | `ChannelData.data` | **time → channel → wave → frame** | `[1024, 64, 1, 1]` | int16 / float32 |
| MFMC-CFF | FMC 原始矩阵（惯例） | tx → rx → samples | `[64, 64, 2048]` | int16 / float32 |

> ⚠️ **轴序各家不一致**，ONDE 与 UFF 恰好相反。契约中必须显式声明，不可依赖惯例。

### ② 时间轴

| 规范 | 字段 | 单位 | 示例 |
|---|---|---|---|
| ONDE | `Ultrasound` 轴的 `resolution` / `offset` | s | `1.3e-7` / `0.0` |
| UFF | `sampling_frequency` | Hz | `5.0e7` |
| UFF | `initial_time` | s | `5.0e-6` |
| UFF | `PRF`（**帧率，非采样率**） | Hz | `1000` |

> **`initial_time` 是唯一不可省的字段**：它把「数组下标 0」与真实声程时刻钉死。数据下标 → 声程时间 → 深度：`depth = velocity × (initial_time + i / fs) / 2`。

### ③ 阵列几何

| 规范 | 字段 | 单位 | 示例 |
|---|---|---|---|
| ONDE | 位于 `/Public/Setup` 的 JSON（`probes`） | — | — |
| UFF | `N_elements` / `N_active_elements` / `N_channels` | 个 | `64` / `16` / `64` |
| UFF | 阵元坐标表 | m | `[[0,0,0],[0.0005,0,0],…]` |
| MFMC-CFF | 见原始 MFMC 规范 | — | — |

### ④ 聚焦律

| 规范 | 字段 | 单位 | 示例 |
|---|---|---|---|
| ONDE | `beams[].velocity` | m/s | `5900`（钢纵波） |
| ONDE | `beams[].skewAngle` / `.refractedAngle` | ° | `0.0` / `45.0` |
| ONDE | `beams[].ultrasoundOffset` | s | `1.0e-6` |
| UFF | `sequence`（`Wave` 列表） | — | 每根波束一条 |

### ⑤ 位置轴

| 存法 | 规范 | 字段 | 单位 | 示例 |
|---|---|---|---|---|
| 紧凑式 | ONDE | `quantity` / `resolution` / `offset` | 个 / m / m | `351` / `0.001` / `-0.07455` |
| 三维 | ONDE | `WCoordinate` | m | `1055`（TFM） |
| 直白式 | MFMC-CFF | `xVector` / `zVector` | m（×系数） | 见 §3.4 备注 |

### ⑥ 状态位

| 规范 | 字段 | 编码 | 示例 |
|---|---|---|---|
| ONDE | `AScanStatus` | bitfield `hasData=1` / `saturated=2` / `noSynchro=4` | `3`（有数据且饱和） |
| UFF | — | 无 | — |
| MFMC-CFF | — | 无（仅有派生 `sens_maps`） | — |

> 状态位**仅 ONDE 一家提供**，属加分项而非行业底线。

---

## 5. 维度轴取值举例（源自 ONDE 文档）

| 扫描方式 | storageMode | 维度构成 | 举例 |
|---|---|---|---|
| 相控阵单线扫查 | `Independent` | UCoordinate + Beam + Ultrasound | U: `22` @ `0.001`；Beam: 列表；US: `620` @ `1.3e-7` |
| 相控阵零度光栅扫查 | `Paintbrush` | UCoordinate + VCoordinate + Ultrasound | U: `351` @ `0.001`；V: `114` @ `0.001` offset `-0.07455`；US: `568` @ `2e-8` |
| TFM 全聚焦 | `Paintbrush` | UCoordinate + VCoordinate + WCoordinate | U: `1055`；V: `175`；W: `64` |
| FMC 全矩阵 | （实验性） | UCoordinate + **StackedAScan** | 各 A 扫按发振顺序串联 |

---

## 6. 本工作区现状（`PautFrame`）

| 类 | 字段 | 当前值 | 状态 |
> **本节已按厂商《相控阵数据格式》表（`docs/image.png`）更新**——该表说明大量"缺失"实为**未传输**而非**不存在**。

| 类 | 字段 | 当前值 | 状态 |
|---|---|---|---|
| ① | `samples`（`int32[]`） | 长 10187 = 167 × 61 | ✅ 有值 |
| ① | 真实轴语义 | **167=采样点数(深度)，61=电子扫查波束数** | ⚠️ 与 `.msg` 注释相反 |
| ② | 采样点数 | `167`（现藏在名为 `channel_count` 的字段里） | ✅ 有值，但字段名错 |
| ② | `sampling_rate_hz` | `0.0` | ❌ 未知（**设备表亦未提供**） |
| ② | 时基起始延时 `initial_time` | 字段不存在 | ❌ 设备表亦无 |
| ③ | 阵元总数（32）、pitch、第一阵元中心距、模块角、模块声速、板卡通道 | — | ⚠️ **设备端有，协议未传** |
| ④ | 扫查模式、聚焦起止角、聚焦深度、孔径、步进角度、试件声速、主偏偏移 | — | ⚠️ **设备端有，协议未传** |
| ④ | `sound_velocity_m_s` | `NaN` | ❌ 未知 |
| ④ | `gain_db` | `NaN` | ❌ 未知 |
| ⑤ | 扫查轴编码器值、步进轴编码器值 | 字段不存在 | ⚠️ **设备端有，协议未传** |
| ⑤ | `frame_id` | `"paut_probe_link"` | ⚠️ 空壳 |
| ⑥ | 状态位 | 字段不存在 | ❌ **设备格式里也没有** |
| — | `calibration_id` | `"UNASSIGNED"` | ⚠️ 未标定占位 |
| — | `header.stamp` | 主机收包完成时刻 | ⚠️ 非声程时间，非硬件时戳 |

**结论**：缺失分两种性质，处置完全不同——

| 性质 | 涉及类别 | 处置 |
|---|---|---|
| **设备有、协议没传** | ③ 阵列几何、④ 聚焦律、⑤ 双轴编码器 | **改协议即可**，设备端无需新增采集能力 |
| **设备侧也不存在** | ② 采样率/时基起点、⑥ 状态位 | 需**设备端新增**，工作量与风险都更大 |

其中 **② 的采样率与 ④ 的声速是「补上才能做任何定量分析」的卡口**；③⑤ 是「补上才能做融合/成像」的。

---

## 7. 选型建议

| 场景 | 建议格式 | 理由 |
|---|---|---|
| 机器人多模态在线采集（现状） | **MCAP** | 多话题、带时间戳、可回放、生态成熟 |
| 超声数据对外交付 / 归档 | **DICONDE** | 行业通用、跨厂商可读、合规可追溯 |
| 超声信号级交换 / 再处理 | **ONDE（HDF5）** | 开放、结构清晰、信号与元数据分离完备 |
| 训练样本集 | HDF5 / zarr | 分块流式读、numpy/PyTorch 直接消费 |

**落地路径**：MCAP 作为**采集主干**保持不变，另写 exporter 转出 ONDE 风格的 HDF5。转换时按 §4 六类逐项对照，**缺失项显式写「未知」占位**（如 `NaN` / `"UNASSIGNED"`），不要默认零值——零值与未知在物理上含义完全不同。

---

## 8. 待确认事项

> 本节原列 5 条，厂商《相控阵数据格式》表（`docs/image.png`）已解决其中 2 条。剩余问题统一维护在 `docs/PAUT_DATA_FORMAT_SPEC_ZH.md` §14，此处不再重复。

**已由厂商表解决**：

| 原问题 | 结论 |
|---|---|
| ~~能否开放未波束合成的原始阵元数据？~~ | ✅ **能**。`扫查模式` 含 `全聚焦`，其数据结构为 `[阵元总数]×[阵元总数]`，即 FMC 全矩阵 |
| ~~61 个波束位置的生成方式？~~ | ⚠️ 部分。`扫查模式=线扫` 时结构为 `[波束个数]×[采样点数]`，故 61=波束数、167=采样点数**已确证**；但 61 个波束**如何由 32 阵元生成**仍待确认 |

**仍待确认**（详见 `docs/PAUT_DATA_FORMAT_SPEC_ZH.md` §14）：采样率是否为设备原生可提供、样本值域为何仅 `0–255`、全聚焦模式的物理链路、编码器分辨率等。

**本工作区内部待决**：`.msg` 轴语义的修正方式（仅改注释 vs 改字段名，后者会使已有 MCAP 类型哈希失效，见 `docs/PAUT_ROS2_MESSAGE_SPEC_ZH.md` §5.1）。

---

## 9. 参考来源

- [Dataset — NDE Open File Format (ONDE)](https://ndeformat.com/3.3/general-concepts/dataset/)
- [ONDE HDF5 structure — Public Group](https://ndeformat.com/4.2/hdf5-structure/public-group/)
- [ONDE Datasets data model](https://ndeformat.com/4.0/json-metadata/setup/data-model/groups/datasets/)
- [ASTM E2663-25 — DICONDE for Ultrasonic Test Methods](https://store.astm.org/e2663-25.html)
- [pyuff_ustb.objects.ChannelData（UFF 字段表）](https://pyuff-ustb.readthedocs.io/en/latest/_autosummary/pyuff_ustb.objects.ChannelData.html)
- [UFF wiki (v0.2 / v0.3)](https://bitbucket.org/ultrasound_file_format/uff/wiki/Home)
- [MFMC File Specification v2.0.0 — University of Bristol](https://data.bris.ac.uk/datasets/1c46xjhyp8dz221zsfg622y8r9/MFMC%20File%20Specification%20(v2.0.0).pdf)
- [Multi-Frame Matrix Capture Common File Format — Bristol NDT](https://www.bristol.ac.uk/research/groups/ndt/projects/multi-frame-matrix/)
- [The OpenNDE data format (ECNDT 2026)](https://www.ndt.net/article/ecndt2026/papers/ECNDT_2026_ID_191.pdf)

> **可信度说明**：ONDE 与 UFF 的字段表逐条取自公开文档，已核实；DICONDE 相关条目取自 ASTM 公开摘要，**未取得标准全文，未列具体标签号**，落地前须核对原文；MFMC-CFF 条目部分取自其数据集说明，原始 FMC 规范细节未完整核实。
