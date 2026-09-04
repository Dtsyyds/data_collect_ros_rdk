# 从真机采集到算法训练：数据管线总体方案（中文）

> 定位：本文是在 `README.md` / `STATUS.md` / `DATA_ACQUISITION_ZH.md` 之上的**路线图/设计稿**，
> 讲清"ROS 采集 → 方便保存 → 方便训练"整条链路怎么做、缺什么、按什么顺序补。
> **算法任务类型与真值来源尚未定稿**，本文按"任务无关、数据侧优先"组织，两处定稿点显式标出。
> 文档创建于 2026-09，基于仓库现有能力（MCAP 采集、校验/哈希/NAS、FusionIndex 时间索引、mock 仿真）。

## 0. 背景、目标与铁律

**现状能力（已具备，见 STATUS.md）**：ROS2 Jazzy 采集链（MID-360 点云/IMU + D405 深度/彩色 + 可选涡流/超声/机器人位姿）→ 多分片 MCAP → 0 错校验 → SHA256 manifest → `CLOSED→HASHED→COPIED→VERIFIED` 生命周期 → 本地 NAS 拷贝；另有 `FusionIndex` 把多模态锚到同一时刻（有界缓存 + 最近帧匹配）。

**两个终点，需求不同**：
- **保存/归档**：可靠、完整可校验、可按语义检索、可追溯。
- **训练数据**：规整张量、可回链源、防泄漏划分、DataLoader 友好。

**贯穿全链路的铁律**（延续本仓库哲学）：
1. 原始 `.mcap` 视为**不可变源**，永不原地改动。
2. 训练用的规整样本是**可重生成的派生副本**，由**版本化转换脚本**产生。
3. 每条样本可**反查回源 bag + FusionIndex + 标定/真值版本**（用 id，不内嵌易变的元数据）。
4. 数据、标定、真值都是"带版本的资产"，样本只认 id。

## 1. 全链路总览

```text
现场真机 ROS2 话题流
   │ ① 采集  ros2 bag record ─▶ .mcap 分片           【已有，完善中】
   ▼
② 归档  校验 + SHA256 manifest + NAS 拷贝 + 生命周期   【已有，较扎实】
   ▼
③ 目录化  登记表/语义索引(日期/罐/板/校准/QC)           【待补】
   ▼
④ 样本化  按 FusionIndex 抽帧 → 反序列化 → 张量导出      【待补 · 核心缺口】
   ▼
⑤ 真值/标注（决策点① 人工 / 探头弱标签 / 仿真 GT）       【待补 · 取决于任务】
   ▼
⑥ 数据集管理  防泄漏划分 + sample manifest(回链)        【待补】
   ▼
⑦ 训练接入  DataLoader ─▶ 训练/评测 ─▶ 缺口 ─▶ 加采     【后续】
```

## 2. 层① 采集（现状与前提）

现状见 `DATA_ACQUISITION_ZH.md`：真机 `hardware_pipeline.launch.py`、一键 `baseline_capture.launch.py`、仿真 `mock_pipeline.launch.py`；16 个话题、生产分片 10GB/900s。

**进入"可用于算法"的采集前提**（STATUS.md 明列，未做不应急于攒数据）：
1. D405 内参/深度尺度 + 各 base→camera/lidar/probe 外参标定；
2. MID-360 外参标定与逐点时间戳排序文档化；
3. 跨设备时钟同步方案（PTP/NTP/硬触发）+ 漂移监控；
4. 涡流/超声 激励/增益/提离/楔块延时定标（当前涡流 `pose_valid=0`、Q 为 NaN）；
5. 力/接触判定阈值；
6. 罐 course/plate/weld 资产坐标对勘测数据的对齐与置信；
7. 长时间吞吐、热、丢包、磁盘压测。

## 3. 层②③ 保存侧：归档 + 目录化（数据侧地基 A）

**② 已有**：MCAP 分片、`validate_bag` 0 错校验、SHA256 manifest、生命周期、NAS 拷贝。**保持**。

**③ 要补的"语义索引"**：目录里能"找得到"，但目前不能"查得到"。建议加一份**登记表**（SQLite 起步即可，行数在万级以下用 CSV 也行）：

```text
bag_id(uuid) | collected_at | tank_id | plate_id | course_id | weld_id
sensors[set] | mid360_cal_id | d405_cal_id | eddy_cal_id | sync_cal_id
pose_valid | qc_pass(bool) | validation_report | mcap_count | size_bytes | remote_store_path | notes
```

- 每份 bag 在采集结束 + 校验通过后**自动追加一行**；QC 失败标 `qc_pass=false`，训练检索时排除。
- 用 `uuid` 绑定 bag（不止时间戳目录），跨盘/跨人不出歧义。
- 定义保留/备份策略：远端 NAS/对象存储 + 增量同步，延续"先本地 SHA256 校验再拷"。
- 这一步产出：`catalog.sqlite` + 归档目录，作为"全库我有哪些数据、哪些能用"的入口。

## 4. 层④ 样本化：bag → 张量导出（数据侧地基 B，核心缺口）

**方法论**：归档保存最原始；训练用规整副本；二者靠**可重现、可版本化**的 exporter 连接。

**抽帧键**：直接复用/下沉 `FusionIndex` 语义——以 anchor（涡流/超声，或降到 50Hz 采样）为帧时刻，最近帧匹配 D405/MID-360。离线可更精细（在 anchor 时刻对各模态做插值）。原则不变：**源 header 时间戳不改，只在派生层对齐**。

**exporter 职责与技术选型**（Python，建议独立于 ROS 运行，仅读 .mcap，避免部署时依赖 ROS）：
- 读 .mcap：`mcap` 库 + `rosbags` 反序列化；
- 张量化：深度/彩色 → numpy `(H,W[,C])`，点云 → `(N,3)` 或 `(N,4)`，IMU → 窗口数组，位姿 → `(s,v,n)` + 四元数；
- 输出落盘：小/中规模用 **HDF5(.h5)**（或 zarr / 目录 + 每帧 .npy + index）。任选的前提一致：流式可切、不必全载入、支持 numpy/pytorch 直接消费。

**任务无关"最小样本"草案**（扩展槽留给任务定稿）：

```yaml
sample:
  sample_id: uuid               # 回链键
  source: {bag_id, fusion_index_start, fusion_index_end, exporter_version}
  calibration: {mid360_cal_id, d405_cal_id, eddy_cal_id, pose_valid}
  frames:                       # 每 anchor 时刻一帧
    - {t_anchor, t_d405, t_mid360, dt_d405_ms, dt_mid360_ms, pose_valid}
  depth:      (H,W) uint16       # 原始深度
  color:      (H,W,3) uint8
  lidar:      (N,3) float32      # 最近帧点云
  imu_window:(M,7) float32       # 锚点前后窗口 加速度+角速度
  position:   {s_m, v_m, n_m, pose_valid}   # 罐壁圆柱坐标(任务关键)
  labels:     null               # 决策点① 定稿后填充
```

**重生成纪律**：exporter 带版本号与参数（锚降采样、是否插值、抽帧范围）进 manifest；改导出逻辑后全量或增量重放重导出，不碰源。

## 5. 层⑤ 真值/标注（决策点①，混合/待定）

三种来源对比（结合油罐检测特点）：

| 来源 | 原理 | 成本 | 精度 | 适配阶段 |
|---|---|---|---|---|
| **仿真 GT** | mock/物理确定性几何里"埋"已知缺陷，导出精确位置 | 低 | 高但 sim2real 有差 | 起步：先把 网络→训练→评测 闭环跑通 |
| **涡流/超声 弱标签** | 接触式探头信号当缺陷真值（幅值/相位异常→区域异常） | 中 | 受探针定位与标定限制 | 中期：真机伪标签，需解决 ④ 中探针位姿与标定 |
| **人工标注** | 图像/点云上标缺陷框/分割/时序区间 | 高 | 高 | 后期：抽检校准 + 少量精标 |

**建议混合策略（逐步收敛）**：先用**仿真 GT** 打通管线 → 再以**涡流/超声弱标签**做大样本真机集 → 人工只做**抽检与校正**，把人工成本压到最低。这一步要求先定任务类型（检测框/分割/(s,v,n) 坐标/判定），故此处**先留决策记录、不锁设计**；样本结构已为 `labels` 留扩展槽。

## 6. 层⑥⑦ 数据集管理与训练接入

- **防泄漏划分**：按 **tank/资产** 切 train/val/test，**同一资产绝不跨集**（否则几何记忆导致虚高指标）。
- **过滤**：训练时仅取 `qc_pass`、`pose_valid`、标定齐全、任务相关段的样本。
- **sample manifest**：每条样本 uuid → 源 bag/FusionIndex/标定/真值版本回链；任何指标问题都能反查到源与采集配置。
- **DataLoader**：按需随机采样 + 流式读（h5/zarr 分块或目录 lazy load），避免整库载入；多模态各自 transform，落到统一 `(s,v,n)` 网格或 2D 展开即可训。
- **闭环**：训练暴露的盲区/长尾 → 定向补采 → 回到 ②③④ 增量。

## 7. 决策点与开放问题（记录，待定稿）

- 决策① 任务类型：缺陷检测/定位 / 缺陷分类 / BEV 表征 / 其他 —— 决定 labels 与样本几何组织。
- 决策① 真值主源：仿真 / 探头弱标签 / 人工，及混合配比。
- ③ 登记表载体：SQLite vs CSV 起步（建议先 CSV，行数破万再迁 SQLite）。
- ④ 落盘格式：h5 vs zarr vs 目录+npy（建议先 h5 单文件，够用且生态成熟）。
- exporter 增量更新策略（全量重放 vs 增量）。

## 8. 分阶段落地路线（建议顺序与最小验收）

| 阶段 | 内容 | 产出/验收 |
|---|---|---|
| **P0 采集前提** | 完成 §2 的标定与时间同步、7 项硬件压测 | 标定/同步证据入档；采集样本带有效 pose |
| **P1 目录化** | §3 登记表脚本 + 采集后自动入库 | 任一 bag 可检索、可回溯、可按 QC 过滤 |
| **P2 exporter 最小闭环** | §4 读 latest bag + FusionIndex → h5 样本集；含 schema/版本 | 一键把一份合格 bag 导出样本；样本可回链 |
| **P3 真值管线** | 决策①定稿后：仿真 GT 或探头弱标签或标注导入 | 样本带 labels；划分脚本给出 train/val/test |
| **P4 训练接入** | DataLoader + 基线训练/评测脚本 | 能起一轮训练并复现指标，能反向定位失败样本 |

## 9. 与本仓库代码/文档的对应

- **直接复用**：`inspection_sync`（FusionIndex 抽帧键）、`validate_bag`/`finalize_bag`（归档链路）、mock 仿真（P3 仿真 GT 来源）、STATUS.md 实测数据（P0 依据）。
- **建议新增（独立于 ROS，或 `inspection_tools` 扩展）**：`catalog.py`（③）、`export_dataset.py`（④）、`split.py`（⑥）、`dataset.py`（⑦ DataLoader）。
- 采集侧现成：`baseline_capture.launch.py`（P2 的 latest-bag 输入）、`hardware_pipeline.launch.py`（真机源）、`mock_pipeline.launch.py`（仿真源）。

> 一句话：**先补"能查的归档索引(③)"和"能产的样本导出(④)"这两个数据地基，任务与真值定稿后再接"标签(⑤)+划分(⑥)+训练(⑦)"**——数据侧不依赖算法侧即可先行，避免等任务想清楚才动手。
