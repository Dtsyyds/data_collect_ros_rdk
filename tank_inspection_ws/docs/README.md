# docs/ 目录索引

本目录存放**设计规格与专题参考**。工作区级的说明与流程文档在仓库根目录。

---

## 文档结构：一份跨模态顶层规范 + 一条 PAUT 实例链

```
  INSPECTION_DATA_SPEC_ZH.md            ★ 跨模态顶层：骨架 + 七条契约
        │                                  （多通道超声 / 相控阵 / 涡流 / 磁）
        │                                  ← 加新模态时只看这份
        │
        └──► PAUT 实例链（PAUT 已先行，其余模态照此办理）

  docs/image.png                        ← 起点：厂商《相控阵数据格式》原表
        │
        ▼
  PAUT_DATA_STORAGE_FORMATS_ZH.md       ← 行业规范调研：别人怎么存
        │   （ONDE / UFF / DICONDE / MFMC-CFF 对照 + 六类信息归纳）
        ▼
  PAUT_DATA_FORMAT_SPEC_ZH.md           ← 线格式设计：设备 → 主机 UDP 怎么发
        │   （字节级字段清单，可直接交设备端实现）
        ▼
  PAUT_ROS2_MESSAGE_SPEC_ZH.md          ← 消息层设计：落到 ROS 2 / MCAP 怎么组织
        │   （.msg 定义 + QoS + 话题布局）
        │   ⚠ 其字段定义基于设计阶段假想的线格式，已被下方位文档 §0 修正
        ▼
  PAUT_12346_ROS2_MAPPING_ZH.md         ★ 实现记录：线格式 → ROS 的**真实**映射
        │   （逐字段映射表、未知量处置、config_seq 决策、C7.2 竞态例外、实测结果）
        ▼
  PAUT_IMPLEMENTATION_PLAN_ZH.md        ← 落地：拆成任务，供后续开发
            （阶段 0–4、T1–T14、V1–V9 验收、已知陷阱）
   ─────────────────────────────────────────────────────────────
  PAUT_DEVICE_CHANGES_REPORT_ZH.md      ← 设备端改造的阶段汇报（已完成）
  PAUT_DEVICE_INFO_INVENTORY_ZH.md      ← 设备端可获取信息全量清单
```

### 各文档用途速查

| 文档 | 回答什么问题 | 读者 |
|---|---|---|
| `INSPECTION_DATA_SPEC_ZH.md` | 各模态**哪些字段该公共、哪些该特有**？七条契约是什么？ | **所有人**（顶层） |
| `PAUT_DATA_STORAGE_FORMATS_ZH.md` | 行业里 PAUT 数据**有哪些**规范？各存了**什么字段**？我们缺什么？ | 技术选型、方案汇报 |
| `PAUT_DATA_FORMAT_SPEC_ZH.md` | 设备端**该发什么**？每个字段的偏移/类型/单位是什么？ | 设备端开发者 |
| `PAUT_ROS2_MESSAGE_SPEC_ZH.md` | 采集端**该定义什么消息**？话题与 QoS 怎么配？（**字段定义已过时**） | 采集端（ROS）开发者 |
| **`PAUT_12346_ROS2_MAPPING_ZH.md`** | **实际**映射是什么？为什么这么定？哪些量不可信？验证结果如何？ | **采集端开发者（首选）** |
| `PAUT_IMPLEMENTATION_PLAN_ZH.md` | **先做什么后做什么**？验收怎么判？有哪些坑？ | 项目推进 / 开发者本人 |
| `PAUT_DEVICE_CHANGES_REPORT_ZH.md` | 设备端改了什么？代码清单？ | 汇报 / 溯源 |
| `PAUT_DEVICE_INFO_INVENTORY_ZH.md` | 设备端**能拿到哪些信息**？ | 协议设计 |

### 核心结论（四句话）

1. **分层**：**统一发生在导出层（L2），不在采集层（L1）**。采集要保真、导出要统一，目标冲突，不能塞进同一个类型。
2. **轴语义**：`167 = 采样点数（深度）`，`61 = 电子扫查波束数`，`32 = 阵元总数`。当前 `PautFrame.msg` 的注释与字段名**正好写反**。
3. **数据已波束合成**：61 个波束由 32 阵元电子扫查生成，**原始阵元数据未传出**，属单向门。但厂商表显示 `扫查模式` 含 `全聚焦`，即 FMC 全矩阵**可得**。
4. **"缺失"分两种**：③阵列几何 / ④聚焦律 / ⑤双轴编码器是**设备有、协议没传**（改协议即可）；②采样率、⑥状态位是**设备侧也没有**（需设备端新增）。

### 最该先探清的一件事

**时间契约**（`INSPECTION_DATA_SPEC_ZH.md` §3 C1）。PAUT 设备无硬件时戳，只有主机收包时刻——若涡流与磁同样如此，多模态融合的时间精度就只有网络抖动级别，**融合会名存实亡**。这条可能反过来约束设备选型，建议优先确认。

---

## 其他文档

| 文档 | 说明 |
|---|---|
| `RERUN_PORTABILITY.md` | `inspection_rerun` 包从 x86 Jazzy 迁移到 S100P Humble 的可移植性说明 |

---

## 仓库根目录的相关文档

| 文档 | 说明 |
|---|---|
| `../ULTRASOUND_PHASED_ARRAY_INTEGRATION_GUIDE_ZH.md` | 超声相控阵**设备集成**指南 |
| `../TRAINING_DATA_ROADMAP_ZH.md` | 采集数据 → 训练样本的路线图（含 HDF5/zarr 选型） |
| `../DATA_ACQUISITION_ZH.md` | 数据采集机制总览 |
| `../MCAP_AND_ROSBAG_ZH.md` | mcap 与 rosbag 的说明 |
| `../HARDWARE_INTEGRATION_GUIDE.md` | 硬件集成指南 |
| `../RK3588_PORTING_GUIDE.md` | RK3588 平台移植指南 |
| `../STATUS.md` | 工作区状态 |

---

## 相关实测报告

| 文档 | 说明 |
|---|---|
| `../reports/paut_real_analysis/PAUT_相控阵超声采集测试报告.md` | 真机采集 958 帧的分析报告（含 A 扫热图） |
| `../reports/paut_real_analysis/paut_a_scan_heatmap.png` | A 扫波形与热图 |
| `../reports/paut_real_analysis/paut_echo_vs_time.png` | 回波峰值随时间变化 |

---

*注：`docs/image.png` 是厂商提供的《相控阵数据格式》原始表格截图，为本链条的输入依据，请勿删除。*
