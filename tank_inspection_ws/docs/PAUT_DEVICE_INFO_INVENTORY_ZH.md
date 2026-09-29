# PAUT 设备可获取信息清单

- **文档日期**：2026-09-11
- **来源**：`/media/zyj/Data/研究生1/超声/UDT/phased_array/ultrasonic-phased-array/Demo/`（C# WinForms，.NET 4.8，约 41k 行）
- **依据代码**：`Src/Model/`、`Src/My_Params/`、`Src/Common/`、`Src/My_Algorithm/Linear_Scan.cs`、外部程序集 `DpiBoard.dll`
- **用途**：为「设备端加什么字段」提供全量候选清单

---

## 0. 三条数据通道

```
┌─────────────────────────────────────────────────────────────────┐
│ 通道①  板卡 → 上位机 UDP   大端  RawDataStruct                  │
│        信息最全，含编码器/闸门/帧号，但上位机程序没有外发          │
├─────────────────────────────────────────────────────────────────┤
│ 通道②  上位机 → 外部 UDP 广播 :12345   小端  int[,] 图像         │
│        ★ 我们现在在用的通道                                       │
├─────────────────────────────────────────────────────────────────┤
│ 通道③  上位机 → 磁盘   .bin / .csv / .png                        │
│        现成的导出能力，需人工点按钮                                │
└─────────────────────────────────────────────────────────────────┘
```

> ⚠️ **注意字节序不一致**：通道①是**大端（网络序）**，通道②是**小端**（`.NET BinaryWriter` 默认）。两条通道的解析规则完全不同。

---

## 1. 通道②：`:12345` UDP 广播（我们当前在用）

**唯一发送点**：`Src/Common/ImageBroadcaster.cs`，仅被 `Linear_Scan.cs:108` 调用 —— **只有线扫会广播，扇扫/TFM 不广播**。

### 1.1 线格式

```
offset 0   int32 height          // beamQty，波束数，实测 61
offset 4   int32 width           // region_row_nums，深度行数，实测 167
offset 8   int32[height×width]   // 行优先 [beam][row]，小端
```

实测：`8 + 61×167×4 = 40,756 B`

### 1.2 分包协议（已实现但从未触发）

| 条件 | 行为 |
|---|---|
| `data.Length ≤ 64000` | 整包直发（当前情况） |
| `data.Length > 64000` | 先发一包 **4 字节 `int32 totalSize`**，`Sleep(10)`，再按 64000 分片，**每片前缀 4 字节 `int32 offset`** |

⚠️ 我们的接收端**没有实现分片解析**。切 TFM 会立刻触发。

### 1.3 已确定字段（实测可得）

| 字段 | 值 | 来源 |
|---|---|---|
| `beamQty` | 61 | `(elementNum - little_aperture)/step + 1` |
| `region_row_nums` | 167 | `(int)(img_hight / resolution)` |
| `region_row_begin` | 未发 | `(int)(img_hight_bias / resolution)` |
| `resolution` | 未发 | `0.001 / sample_rate * speed` [mm/行] |
| `img_width` | 未发 | `pitch * step * (beamQty - 1)` [mm] |

**可得结论**：波束 i 的位置 `x_i = i × pitch × step`，**只要 pitch 和 step 就能算**。

---

## 2. 通道①：板卡上报（未开发，信息最全）

**接收点**：`Src/Model/ReciveDataProcessManager.cs`（注意 `Recive` 是拼写错误），回调注册于 `:53`。
**实际 socket 在** `DpiBoard.dll` 的 `DplBoard.Udp.UtilsParameter`。

### 2.1 包头 `RawDataHeader` 全字段（**大端**）

| 字段 | 类型 | 含义 | 源码行 |
|---|---|---|---|
| `magic` | byte[2] | 魔数（TFM 包为 `0x5A 0xA5`） | :102 |
| `boardId` | byte | 板卡 ID（"ip地址后三位"） | :235, :306 |
| `reserved0` | byte | 保留 | — |
| `typeId` | byte | 包类型 | — |
| **`frameId`** | uint32 | **帧号** | :307 |
| `groupId` | ushort | 组 ID | — |
| **`unionId`** | ushort | **波束索引** | :308 |
| `segmentId` | ushort | 段 ID | — |
| **`protocolVersion`** | ushort | **实为包序号 packageId**（从 0 计数） | :243 |
| `fpgaVersion` | uint | FPGA 版本 | — |
| `tofdFpgaVersion` | uint | TOFD FPGA 版本 | — |
| `groupPointNum` | ushort | 采样点数 | :241, :244 |
| `reserved1` | byte[] | 保留（MultiscanPar 下 `[2]` 复用为通道 ID） | :259 |
| **`aStrobeState`** | byte | **A 闸门状态** | :309 |
| **`aStrobeData`** | ushort | **A 闸门数据** | :310 |
| **`aStrobeSite`** | uint | **A 闸门位置** | :311 |
| **`bStrobe*`** | byte/ushort/uint | **B 闸门 状态/数据/位置** | :312-314 |
| **`cStrobe*`** | byte/ushort/uint | **C 闸门 状态/数据/位置** | :315-317 |
| **`encodeX`** | int32 | **编码器 A 计数** | :318 |
| **`encodeY`** | int32 | **编码器 B 计数** | :319 |
| `reserved` | byte[] | 保留 | — |

**★ 三路闸门（A/B/C Gate）是现成的缺陷检测结果**——状态 + 峰值 + 位置，设备已经算好了。

### 2.2 数据体 `RawDataStruct`

| 字段 | 类型 | 说明 |
|---|---|---|
| `header` | `RawDataHeader` | 见上 |
| **`data`** | **`byte[]`** | **A 扫波形，`byte` 类型（8 位）** |

> **`RawDataStruct.data` 是 `byte[]`——`0–255` 之谜的答案。**
> 定义为编译好的 `DpiBoard.dll`（`my_resource/Sdk/DpiBoard.dll`），源码树中无定义。

`data` 长度 = `DplGlobal.WaveDataQty` = **896**。

### 2.3 TFM/FMC 专用包（魔数 `0x5A 0xA5`）

| 偏移 | 类型 | 字段 | 说明 | 行 |
|---|---|---|---|---|
| 0-1 | byte×2 | magic | `0x5A 0xA5` | :102 |
| 3 | byte | `transmitId` | 发射阵元 | :109 |
| 4 | byte | `receiveId` | 接收阵元 | :110 |
| 5 | byte | `index` | **包序号**，写入 `origin_data[tx,rx][index*1000]` | :116 |
| 8-11 | int32 BE | **`encoderX`** | 编码器 A | :105 |
| 12-15 | int32 BE | **`encoderY`** | 编码器 B | :106 |
| **24+** | byte[1000] | A 扫数据 | **每包固定 1000 点** | :156 |

---

## 3. ★ 设备内部有、两条通道都没发的（改代码即可加）

**这是清单里最有价值的部分** —— 全部是"设备本来就有，只是没序列化进包"。

### 3.1 采集参数 `GeneralParamsByBeam`

| 字段 | 类型 | 单位 | 含义 | 默认 | 来源 |
|---|---|---|---|---|---|
| `checkMode` | uint | 枚举 | 检测模式 `Pa=0/Ut=1/Tofd=2` | Pa | Beam.cs:23 |
| `trMode` | uint | 枚举 | 收发模式 | Pe | Beam.cs:24 |
| **`prf`** | uint | **Hz** | 脉冲重复频率，范围 100–9999 | 60 | Beam.cs:25 |
| `signalMode` | uint | 枚举 | 检波模式 `双波/正/负/RF` | 0 | Beam.cs:26 |
| `aperture` | uint | 阵元数 | 孔径 | 4 | Beam.cs:27 |
| **`startNs`** | uint | **ns** | **采样起点** ★ | 0 | Beam.cs:28 |
| **`rangeNs`** | uint | **ns** | **采样范围** ★ | 100000 | Beam.cs:29 |
| `filter` | uint | 档位 | 滤波模式 | 3 | Beam.cs:30 |
| **`gain`** | float | **dB** | 增益 | 20 | Beam.cs:31 |
| `avg` | uint | 次 | 平均次数 1–16 | 1 | Beam.cs:32 |
| `envelope` | uint | 0/1 | 是否开包络 | 1 | Beam.cs:33 |
| `pulseWidth` | uint | **ns** | 脉宽 | 80 | Beam.cs:34 |
| `deNoise` | uint | 0/1 | 是否开降噪 | 1 | Beam.cs:35 |
| **`samplePointNum`** | int | 点数 | 采样点数 | 1000 / 896 | Beam.cs:36 |

> ### ★★★ `startNs` / `rangeNs` 就是我一直在要的「②时间轴」
>
> 单位已确证为 **ns**（`ConfigHelper.cs:157-160` 把 `startNs`、`rangeNs` 与 ns 量级的 `wedgeDelay`、常量 `32500`（"3250个周期，每个周期10ns"）直接相加）。
>
> **不需要设备端新增任何采集能力——这三个数本来就在。**

### 3.2 探头 `Probe`

| 字段 | 类型 | 单位 | 内置预设值 |
|---|---|---|---|
| `name` | string | — | `5L64-0.6x10` / `5L32-0.5x10` |
| `type` | int | 0:UT 1:PA | 0 |
| `frequency` | float | **MHz** | 5 |
| **`elementNum`** | int | 阵元数 | **64** / 32 |
| **`pitch`** | float | **mm** | **0.6** / 0.5 |
| `firstChInSys` | int | 系统编号（1 起） | 1 |

### 3.3 楔块 `Wedge`

| 字段 | 类型 | 单位 | 内置预设 |
|---|---|---|---|
| `name` | string | — | `SD3-N0L-IHC` / `SD3-N55S-IHC` / `SD2-N55S-IHC` / `Null` |
| `angle` | float | **°** | 0 / 36 / 36 / 0 |
| `velocity` | float | **m/s** | 2337 |
| `firstEleHeight` | float | **mm** | 20 / 11 / 7.97 / 0 |
| `primaryOffset` | float | **mm** | 49.9 / 61.64 / 30.38 / 0 |

### 3.4 聚焦律 `PaFocalLawParamsByBeam`

| 字段 | 类型 | 含义 |
|---|---|---|
| `txDelay` | uint[] | 发射延时（长度 = aperture） |
| `rxDelay` | uint[] | 接收延时 |
| `txUseElements` | int[] | 发射阵元索引（**从 1 开始**） |
| `rxUseElements` | int[] | 接收阵元索引（**从 1 开始**） |
| `wedgeDelay`（Beam 级） | float (ns) | 楔块延迟 |

**聚焦律计算中间结果**（`FocalLawBeam`，可用于还原几何）：

| 字段 | 类型 | 含义 |
|---|---|---|
| `fstTxElem` / `fstRxElem` | int | 首发射/接收阵元 |
| `refractAngle` | float | 折射角 |
| `screwAngle` | float | 偏转角 |
| `focusPoint` | Vector3 | 焦点坐标 |
| `centerElemPos` | Vector3 | 中心阵元位置 |
| `centerIncidentPos` | Vector3 | 中心入射点位置 |

### 3.5 TCG（时间增益补偿）

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `onOff` | bool | — | 开关 |
| `pointNum` | uint | 点 | 1–16 |
| `beamTcgConfig.positions` | uint[] | 采样位置 | 长度 16 |
| `beamTcgConfig.dGains` | float[] | dB | 增益增量，精度 0.1，范围 0–40 |

### 3.6 TFM 参数

| 字段 | 类型 | 取值 | 说明 |
|---|---|---|---|
| `transmitElementNum` | int | 阵元数 | 发射阵元数（实测 32） |
| `transmitStep` | int | 间隔 | 发射阵元间隔（实测 2） |
| `receiveElementNum` | int | 阵元数 | 接收阵元数（实测 32） |
| `receiveType` | 枚举 | `First=0/Transmit=1` | 接收类型 |
| `pulseWidth` | int | ns | 脉宽 |
| `gain` | float | dB | 增益 |
| `captureStart` | int | 采样点 | 采集起始点 |
| `sampleRate` | 枚举 | `R100Mhz=0/R50/R25/R12.5` | 采样率档位 |
| `pointNum` | 枚举 | `P4000=0/P2000/P1000` | 采样点数档位 |

### 3.7 板卡全局参数 `GlobalPara`

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `utVoltage` | int | **V** | 激励电压，50–400 |
| `waveHeight` | 枚举 | — | `A100=0 / A200=1` |
| `paVoltage` | 枚举 | — | `V100=0 / V50=1` |
| `adcPhases` | int | — | ADC 相位 |
| `encoderTriggerMode` | int | — | 编码器触发模式 |
| **`encoder1.resolution`** | **double** | **step/mm** | **编码器 1 分辨率** |
| `encoder1.polarity` | 枚举 | — | `Normal=0 / Inverse=1` |
| `encoder1.encoderMode` | 枚举 | — | `Down=0 / Up=1 / Quad=2` |
| `encoder2.*` | 同上 | | 编码器 2 |

### 3.8 全局

| 字段 | 类型 | 单位 | 默认 | 来源 |
|---|---|---|---|---|
| `longitudinalVelocity` | float | **m/s** | 5900 | BoardModel.cs:30 |
| `transverseVelocity` | float | **m/s** | 3240 | BoardModel.cs:31 |
| `frameAcquisitionRate` | int | 帧/s | 60 | BoardModel.cs:32 |
| `boardType` | 枚举 | — | `Robust3264Tofd=3` | Board.cs:8 |

> `EBoardType` 枚举：`Robust3264=0, Robust32128=1, Robust32256=2, Robust3264Tofd=3, Robust32128Tofd=4, Multiscan=5, Robust64128=6, Robust64128Tofd=7, MultiscanPar=8`

---

## 4. 系统里**根本没有**的

| 项 | 结论 | 证据 |
|---|---|---|
| **硬件绝对时戳** | ❌ **完全没有** | 全树搜索：`DateTime.Now` 仅用于看门狗与文件名；`Stopwatch` 仅性能计时；**数据包内无任何时钟字段**。时间轴只能靠 `frameId` / `packageId` / `encodeX` 重建 |
| 完整的原始 A 扫外发 | ⚠️ 有，但没全发 | 原始 896 点（`WaveDataQty`），`:12345` 只发 167 点窗口 |
| 自动持续录制 | ❌ 无 | 所有存盘都是**人工点按钮**触发单次 |

---

## 5. 通道③：现成的存盘格式

| # | 触发 | 数据 | 格式 | 位置 |
|---|---|---|---|---|
| 1 | 导出 A 扫 | A 扫波形 | **CSV**（序号+波高两行） | `DemoView.cs:211-281` |
| 2 | 导出聚焦延时 | PA 延时 | **CSV**（探头/楔块/阵元/孔径/角度/深度/声速） | `PaTxParamItem.cs:154+` |
| 3 | 单次保存 | `origin_linear_data` / `origin_sector_data` / `TFM.origin_data` | **`.bin` 纯字节流** + `.png` | `MainForm.cs:460-554` |
| 4 | 增量保存 | 同上 | **`.bin`** + `.png`，文件名 = 递增计数 | `MainForm.cs:556-660` |
| 5 | C 扫 3D | 体数据 | **`.bin` 自定义头**：`double resolution, Dimy, Dimz, Dimx` + byte 体素 | `MainForm.cs:1125-1240` |
| 6 | x 轴扫查 | `Reconstruct_list` | **`.bin` 自定义头**：`double Count, beamqty, data_length, distance` + 逐帧 byte | `SectorScan_Setting.cs:433-459` |
| 7 | 配置 | `BoardModel` / `ProbeWedgeModel` | **JSON**（Newtonsoft） | `BoardModel.cs:159-207` |

---

## 6. 关键结论与行动项

### 6.1 三条结论

1. **`startNs` / `rangeNs` / `samplePointNum` = ②时间轴，设备本来就有**，不需要新增采集能力，只需序列化并发出来。这是本次最大的发现——之前判断「②需设备端新增」是**错的**。
2. **`RawDataStruct.data` 是 `byte[]`**，所以 `0–255` 是设备内部的原生位宽，**不是传输截断**。（但 ADC 位深是否更高、在哪里被压成 byte，仍需确认 `DpiBoard.dll` 内部实现。）
3. **编码器、三路闸门、帧号都在板卡包头里**（通道①），上位机收得到但没转发。⑤位置轴的信息实际已在手边。

### 6.2 建议的字段优先级（全部"设备已有、只需发出来"）

| 优先 | 字段 | 解锁 |
|---|---|---|
| 🔴 1 | `startNs` + `rangeNs` + `samplePointNum` | **深度可换算成毫米** |
| 🔴 2 | `longitudinalVelocity` / 试件声速 | 同上 |
| 🟠 3 | `encodeX` + `encodeY` + `encoder1.resolution` | 位置轴、多模态融合 |
| 🟠 4 | `pitch` + `elementNum` + `step` | **波束位置 x_i（只需 2 个数）** |
| 🟡 5 | `img_hight_bias` / `region_row_begin` | **窗口起点**（第 0 行对应多深） |
| 🟡 6 | `gain` + `pulseWidth` + `prf` + `signalMode` | 定量分析前提 |
| 🟢 7 | 三路闸门 A/B/C | 现成的缺陷检测结果 |
| 🟢 8 | `dataLength`(896) | 说明当前是裁剪窗口 |
| 🟢 9 | 探头/楔块全字段 | 完整溯源 |

### 6.3 未解决的不确定项

| 项 | 状态 |
|---|---|
| **实际采样率** | ⚠️ 两处证据冲突：`config.json` 的 `rangeNs=100000`/`samplePointNum=896` 推出 8.96 MHz；`Sector_Param.SampleNs=35840`/896 与 TFM `sampleRate=2` 推出 **25 MHz**。**必须在运行时读取或试块实测反推** |
| ADC 原生位深 | `byte[]` 已确认，但 16→8 的转换点需查 `DpiBoard.dll` 内部 |
| `resolution` 是否漏除 2 | 代码 `0.001/sample_rate*speed` 未除 2，而脉冲回波应为 `v/(2·fs)`；源码注释留有 `//暂时有问题，需要在真实的情况下乘2，因为往返的问题` |
| 两份 `config.json` | `Demo/config.json`（boardType=5，samplePointNum=896）与 `Demo/bin/Debug/config.json`（boardType=3，samplePointNum=1000，encoder=51.2）**配置不同**，运行时用后者 |

---

*相关文档：`docs/PAUT_DATA_FORMAT_SPEC_ZH.md`（线格式设计）、`docs/INSPECTION_DATA_SPEC_ZH.md`（跨模态规范）、`docs/PAUT_IMPLEMENTATION_PLAN_ZH.md`（开发任务）*
