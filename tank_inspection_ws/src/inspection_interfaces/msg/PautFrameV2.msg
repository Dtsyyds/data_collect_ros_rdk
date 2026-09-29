# PAUT 相控阵超声单帧**完整原始数据**（v1，本仓第一个 V2 类型）。
#
# 线格式权威实现: scripts/recv_paut_12346.py (帧块绝对偏移 24..72 + 数据段 [72, 72+beam*sample))
# 设备端实现: PautRawBroadcaster.cs / PautWireFormat.cs
# 发布: /inspection/paut/raw_v2
#   ⚠ 刻意不复用 /inspection/paut/raw —— 同一话题下新旧两个消息类型会导致
#     DDS 订阅不匹配，且按话题名分派的校验器会反序列化失败。
# QoS: sensor data (best_effort, volatile, keep_last(100))
#
# 契约（docs/INSPECTION_DATA_SPEC_ZH.md）:
#   C1.3 时间必须标明来源（见 timestamp_source）
#   C4   浮点未知填 NaN；整数未知用状态位/命名常量
#   C6.2 新增字段只允许追加在末尾（保证老 bag 可读）
#   C6.5 枚举一律用 const 声明
#
# 与旧 PautFrame（v0）的关系: 旧类型**冻结保留**，字段名与轴语义有误但不再改动——
#   bags/paut_real（958 帧真机数据）依赖它的类型哈希。

std_msgs/Header header              # ⚠ 设备无硬件时戳，本 stamp = **主机收包完成时刻**
                                    #   见 timestamp_source；不要当作测量时刻用于融合

uint8  schema_version               # 本消息结构版本，当前 1

# ---- 配置配对（契约 C7.2）----
uint32 config_seq                   # 指向 PautConfig.config_seq
                                    #   ⚠ 设备启动初期本帧的 config_seq 可能尚未收到对应
                                    #     CONFIG 包（CONFIG 每 2s 重发、FRAME ~86Hz）。
                                    #     校验器应把这种情况记为 warning 而非 error。

# ---- 时间（契约 C1）----
uint64 device_timestamp_ns          # 设备单调时钟 [ns]；**恒 0 = 设备不提供**
                                    #   全设备源码树已确认无任何时钟字段，故恒 0（契约 C4：
                                    #   0 配合 timestamp_source 是无歧义的「未知」，不是「零时刻」）
uint8  timestamp_source             # header.stamp 的实际来源，见 TS_* 常量

# ---- 帧标识 ----
uint64 frame_seq                    # 设备帧编号。⚠ 已知不可靠：设备给同一帧内多个波束打的
                                    #   frameId 可能不一致，故**不可**用它做帧分组键

# ---- 位置（契约 C2.2：raw counts 不进 meta，由消费端结合 PautConfig 换算）----
int32 scan_axis_encoder             # 扫查轴编码器计数
int32 index_axis_encoder            # 步进轴编码器计数
                                    #   ⚠ 单位方向未实测（PautConfig.enc1_unit 当前为
                                    #     ENCODER_UNIT_UNKNOWN），换算前必须先确认

# ---- 维度（显式声明，不推导，契约 C7.1）----
uint16 beam_count                   # 波束数（实测 61）
uint16 sample_count                 # 每波束采样点数（实测 896）
                                    #   ⚠ 与 PautConfig.sample_point_num（实测 1000）**可能不等**。
                                    #     深度换算**只能信本字段**。差值疑似设备端固定按
                                    #     DplGlobal.WaveDataQty(896) 截取所致，待试块验证。
uint16 element_total                # 阵元总数（实测 64）

# ---- 状态 ----
uint16 status_flags                 # 见 STATUS_* 常量
uint32 saturated_count              # 本帧值 == 255 的样本数
                                    #   已与接收端实算值逐帧比对一致（真机验证）。
                                    #   语义局限：恰好等于 255 的有效回波也会被计入，
                                    #   只能当**上界估计**用。

# ---- 数据 ----
# 展平存放（ROS 2 无多维数组）。行优先，轴序 [beam_count][sample_count]：
#   samples[beam * sample_count + sample]
# 元素类型: 线上即 u8（设备 RawDataStruct.data 是 byte[]，非传输截断），
#   故本字段恒为 uint8[]，与 PautConfig.sample_dtype == SAMPLE_DTYPE_U8 绑定。
#   若设备将来开放 16 位位深，按契约 C6.4 新增 PautFrameV3，不改本消息。
# 长度必须等于 beam_count * sample_count（契约 C7.1）。
uint8[] samples

# ================= 常量 =================

# header.stamp 来源（契约 C1.3）
uint8 TS_UNKNOWN = 0
uint8 TS_DEVICE_HARDWARE = 1
uint8 TS_DEVICE_SOFTWARE = 2
uint8 TS_HOST_RECEIVE = 3          # 本设备恒为此值

# status_flags 位定义（线格式 STATUS_BITS，位号勿改）
uint16 STATUS_HAS_DATA = 1
uint16 STATUS_SATURATED = 2
uint16 STATUS_NO_SYNC = 4          # 设备端未实现，恒不置位
uint16 STATUS_ENCODER_VALID = 8    # 启发式：编码器值非零。⚠ 单位未定时不应据此做换算
uint16 STATUS_IS_FRAGMENTED = 16   # 设备端本轮不分片，恒不置位
uint16 STATUS_CALIBRATED = 32      # 采样率等标定量已验证。当前恒不置位（采样率是反推值）
uint16 STATUS_START_NS_TRUSTED = 64  # 恒不置位（设备端 startNs 有累积漂移）
uint16 STATUS_SAMPLE_DTYPE_U8 = 128  # 恒置位
