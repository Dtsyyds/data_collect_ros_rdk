# MCAP 与 ros2 bag：录制格式详解（中文）

> 配套文档：采集链路总览见 [DATA_ACQUISITION_ZH.md](DATA_ACQUISITION_ZH.md)。
> 本文澄清一个常见混淆——**MCAP 不是"录制动作"，而是录出来的文件格式**；
> `ros2 bag record`（工具）→ `rosbag2`（框架）→ MCAP（存储后端/格式）→ `.mcap` 文件（产物）是一条链。

读到了项目实际的写盘参数。下面帮你把"MCAP"和你的理解对齐——你的方向对，但要拆成两层：**`ros2 bag record` 是"录制动作"，MCAP 是"录出来的文件格式"**，两者不是一回事。

## 一、MCAP 到底是什么

**MCAP（Managed Collated Application Protocol）是一种开源的机器人数据"文件容器格式"**，扩展名 `.mcap`。它**不是 ROS 专有的**——由 Foxglove 主导制定，ROS1 / ROS2 / 非 ROS 生态都能读写，目标是"高性能记录 + 快速回放 + 便于下游分析"。

类比帮你建立心智模型：

| 层       | 现实中                   | 本项目里                                    |
| -------- | ------------------------ | ------------------------------------------- |
| 录制工具 | 摄像机                   | `ros2 bag record`                           |
| 录制框架 | 录像/存储系统            | **rosbag2**（ROS2 的 bag 框架）             |
| 存储后端 | 录像带标准（VHS / 数字） | **MCAP 存储插件**（`rosbag2_storage_mcap`） |
| 最终产物 | .mp4 这类文件格式        | **`.mcap` 文件**                            |

所以 MCAP 更接近"视频的 mp4/H264 文件规范"，而不是"拿摄像机录"这个动作本身。

## 二、三个概念的关系：record ≠ rosbag2 ≠ MCAP

- **rosbag2**：ROS2 的录制/回放**框架**，它抽象出一层 Storage 插件接口。同一套 `ros2 bag` 命令，可插不同存储后端。
- **存储后端**有两种：
  - 经典默认：SQLite → 产出 `.db3` 文件（`rosbag2_storage_default_plugin`）
  - 本项目：**MCAP 插件** → 产出 `.mcap` 文件，命令里显式写 `--storage mcap`（`record_only.launch.py:22`）
- **`ros2 bag record --storage mcap --topics …`** = 用 rosbag2 框架 + MCAP 后端，把你列的 16 个话题订阅并写进 `.mcap`。你理解的"用 record 录各话题"完全没错——只是 MCAP 具体指那个**存储产物**。

进一步证明"格式 ≠ record"：你**可以完全不用 `ros2 bag record`**，而用 Foxglove MCAP SDK（C++/Python）直接手写 `.mcap`；反之 `.mcap` 也能拿到非 ROS 平台回放分析。ROS bag 只是它的一个常用入口。

## 三、一次录制写进 .mcap 的是什么

以本项目 `record_topics.yaml` 的 16 个话题为例，MCAP 文件内部是按**结构化的记录流**组织的：

```text
.mcap 文件 ≈
 ├─ schemas   各话题的消息类型定义（如 sensor_msgs/Image、PointCloud2…）
 ├─ channels  每个"话题"对应一个逻辑通道
 ├─ messages  逐条消息（头 + 时间戳 + 序列化字节）
 │     └─ 数据被切成 chunk（本项目 chunkSize 4 MiB），每块用 Zstd 压缩
 ├─ CRC       分块/整体校验
 ├─ message index  每条消息在文件里的位置索引
 └─ summary（含 statistics） 文件统计信息，放在尾部便于快速读取元数据
```

对应项目 `mcap_writer_options.yaml` 的开关正好逐个对上：

```yaml
noChunking: false        # 启用分块（大文件分块压缩，读时不必全解压）
compression: Zstd        # 用 Zstd 压缩（非 ROS 也能解）
chunkSize: 4194304       # 4 MiB 一块
noMessageIndex: false    # 启用逐消息索引
noSummary: false         # 尾部写 summary/统计
noChunkCRC: false        # 分块带 CRC
```

## 四、为什么本项目（和数据采集）偏要选 MCAP

1. **顺序追加写、抗中断**：数据流式往文件尾追加，中途断电/按停最多丢尾部，不易整文件损坏——对现场采集很重要。
2. **索引 + 压缩兼顾**：`SUMMARY`+消息索引让 10GB 级大文件可快速定位，不必全载入内存；Zstd 压缩省盘（README 实测统计都基于此）。
3. **单一文件便于做"不可变 + 哈希"**：一个 bag 目录里是一个个完整自含的 `.mcap` 分片，才能做全生命周期 `CLOSED→HASHED→COPIED→VERIFIED`、SHA256 逐一比对（见 STATUS.md）。
4. **跨生态、面向下游 AI**：MCAP 不绑 ROS，读工具（Foxglove、各种 SDK）广泛，方便后续 Physics-BEV 等分析直接消费。

顺带一提项目里 MCAP 使用上的一个"身份"：**原始源数据与派生 `FusionIndex` 分文件/分路径管理**——原始 `.mcap` 视为不可变源，融合索引回放时另行重生成，不在原始文件里写入（详见 STATUS）。

## 小结

> **你的理解 + 一处修正**：`ros2 bag record` 确实是录制手段 ✅；但 **MCAP 是它所写出的 `.mcap` 文件格式**（一种存储容器标准），对应命令里 `--storage mcap` 那一项，与默认的 SQLite `.db3` 平级。`record`（工具）→ `rosbag2`（框架）→ `MCAP`（后端/格式）→ `.mcap` 文件（产物），是一条链。
