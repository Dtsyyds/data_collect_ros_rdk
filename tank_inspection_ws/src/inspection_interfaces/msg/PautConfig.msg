# PAUT 相控阵超声采集配置（v1）—— 设备端「启动 + 变更时 + 每 2s 重发」的 CONFIG 包。
#
# 线格式权威实现: scripts/recv_paut_12346.py (CONFIG_FIELDS, 绝对偏移 24..140 + 变长区)
# 设备端实现: PautRawBroadcaster.cs / PautWireFormat.cs
# 发布: /inspection/paut/config
# QoS: transient_local + reliable + keep_last(1)   ← 晚加入的订阅者自动补发
#
# 契约（docs/INSPECTION_DATA_SPEC_ZH.md）:
#   C3.4 本消息内保留设备原生单位，字段名内嵌单位后缀（不转 SI）
#   C4   浮点未知填 NaN；整数未知用状态位/命名常量，不填 0
#   C5.2 单位后缀白名单
#   C6.5 枚举一律用 const 声明，禁止裸整数
#
# 编号配对: 本消息的 config_seq 与 PautFrameV2.config_seq 配对（契约 C7.2）。
#   设备每 2s 重发同一 config_seq；内容变化时才递增。

std_msgs/Header header              # 本配置被采集端收到的时刻（设备无硬件时戳）

# ---- 溯源（采集端标签，不由设备提供）----
string sensor_id                    # 未填为 "UNASSIGNED"
string calibration_id               # 未填为 "UNASSIGNED"

# ---- 配置编号 ----
uint32 config_seq                   # 与 PautFrameV2.config_seq 配对

# ================= 板卡段（线格式偏移 24..47）=================
uint32 board_ip                     # .NET IPAddress.Address 形式；还原 a.b.c.d 需字节倒序
                                    #   值 0x8201A8C0 -> 192.168.1.130
                                    #   即 a=v&0xFF, b=(v>>8)&0xFF, c=(v>>16)&0xFF, d=(v>>24)&0xFF
uint32 sample_point_num             # 板卡配置的采样点数（实测 1000）
                                    #   ⚠ 与 PautFrameV2.sample_count（实测 896）可能不等，见该消息注释
uint32 range_ns                     # 时基范围 [ns]（实测 100000）
uint16 ut_voltage_v                 # 激励电压 [V]
uint16 pulse_width_ns               # 脉冲宽度 [ns]
uint16 prf_hz                       # 脉冲重复频率 [Hz]（⚠ 不是采样率）
uint16 start_ns_raw                 # 采样起点 [ns]。⚠ 见下方 start_ns_trusted
uint8  board_type                   # 板卡类型，见 BOARD_TYPE_* 常量
uint8  filter_level                 # 硬件滤波档位（0=不滤波）。⚠ 是档位编号，不是 kHz

bool   start_ns_trusted             # false = start_ns_raw 不可信
                                    #   设备端缺陷：SendParamsManager 每次下发参数都会
                                    #   startNs += wedgeDelay（非幂等，累积漂移）

# ================= 探头段（线格式偏移 48..59）=================
float32 element_pitch_mm            # 相邻阵元中心间距 [mm]
float32 frequency_mhz               # 中心频率 [MHz]
uint16  element_total               # 阵元总数（实测 64）
uint8   probe_type                  # 探头类型（设备侧枚举 0=UT 1=PA）

# ================= 楔块段（线格式偏移 60..75）=================
float32 wedge_angle_deg             # 楔块角度 [deg]。⚠ 用 float32 而非 u8 —— 实际角度有小数
float32 wedge_velocity_m_s          # 楔块声速 [m/s]
float32 first_ele_height_mm         # 第一阵元高度 [mm]
float32 primary_offset_mm           # 主轴偏移 [mm]

# ================= 扫查段（线格式偏移 76..107）=================
float32 img_height_mm               # 成像高度 [mm]（⚠ 是显示窗口高度，不是完整声程）
float32 img_height_bias_mm          # 成像起始偏移 [mm]（第 0 行对应多深）
float32 focus_angle_deg             # 聚焦角度 [deg]
float32 focus_depth_mm              # 聚焦深度 [mm]
float32 specimen_vel_m_s            # 试件声速，成像算法**实际使用**的值 [m/s]
float32 board_vel_m_s               # BoardModel 的纵波声速 [m/s]
                                    #   ⚠ 与 specimen_vel_m_s 是两个来源，可能不一致，都发出来便于发现
uint16  little_aperture             # 线扫小孔径 [阵元数]（实测 4）
uint16  beam_step                   # 线扫步进 [个]（实测 1）
                                    #   波束位置 x_i = i * element_pitch_mm * beam_step
uint16  beam_count                  # 波束数（实测 61）= (element_total - little_aperture)/beam_step + 1
uint8   scan_mode                   # 扫查模式，见 SCAN_MODE_* 常量

# ================= 时间基准（线格式偏移 108..115）=================
float32 sampling_rate_hz            # ADC 采样率 [Hz]。⚠ 见 sampling_rate_src
                                    #   设备不直接提供；本值是 sample_point_num / range_ns 反推
uint8   sampling_rate_src           # 采样率来源，见 RATE_SRC_* 常量
uint8   sample_dtype                # 样本类型，见 SAMPLE_DTYPE_* 常量
uint8   adc_bits                    # ADC 位深；ADC_BITS_UNKNOWN(0) = 未知

# ================= 位置基准（线格式偏移 116..131）=================
float32 enc1_resolution             # 编码器 1 分辨率**原始数值**（实测 51.2）
float32 enc2_resolution             # 编码器 2 分辨率**原始数值**（实测 64.0）
uint8   enc1_unit                   # 见 ENCODER_UNIT_* 常量
uint8   enc2_unit                   #   ⚠ 数值为真，但量纲未知时 unit=UNKNOWN。
                                    #   此时**不得**把 resolution 用于任何换算（契约 C2.2/C2.5），
                                    #   且应清除 PautFrameV2 的 STATUS_ENCODER_VALID 位。
int8    enc1_polarity               # +1 / -1
int8    enc2_polarity               # +1 / -1
uint8   enc1_mode                   # 编码器模式，见 ENCODER_MODE_* 常量
uint8   enc2_mode

# ================= 变长区（设备自由文本）=================
string probe_name                   # 探头名（如 "5L64-0.6x10"）；设备未提供时为 ""
string wedge_name                   # 楔块名（如 "SD3-N0L-IHC"）；设备未提供时为 ""
string notes                        # 设备端的「自我说明」自由文本，如
                                    #   "fs=10.000MHz derived(...),UNVERIFIED; startNs ... untrusted"
                                    #   这是设备把「不可信/未验证」显式声明出来的地方，应随 bag 保留

# ================= 常量 =================

# 编码器单位（线格式 ENCODER_UNIT）
uint8 ENCODER_UNIT_UNKNOWN = 0
uint8 ENCODER_UNIT_COUNT_PER_MM = 1
uint8 ENCODER_UNIT_MM_PER_COUNT = 2

# 采样率来源（线格式 RATE_SRC）
uint8 RATE_SRC_UNKNOWN = 0
uint8 RATE_SRC_DERIVED_UNVERIFIED = 1

# 样本类型（线格式 SAMPLE_DTYPE）。本版 PautFrameV2.samples 恒为 uint8[]
uint8 SAMPLE_DTYPE_U8 = 0
uint8 SAMPLE_DTYPE_I16 = 1
uint8 SAMPLE_DTYPE_U16 = 2
uint8 SAMPLE_DTYPE_I32 = 3
uint8 SAMPLE_DTYPE_F32 = 4

# ADC 位深
uint8 ADC_BITS_UNKNOWN = 0

# 扫查模式（线格式顺序，勿改）
uint8 SCAN_MODE_LINEAR = 0
uint8 SCAN_MODE_SECTOR = 1
uint8 SCAN_MODE_TOFM = 2
uint8 SCAN_MODE_TOFD = 3
uint8 SCAN_MODE_PWI = 4

# 板卡类型（线格式 EBoardType 原值，勿改）
uint8 BOARD_TYPE_ROBUST3264 = 0
uint8 BOARD_TYPE_ROBUST32128 = 1
uint8 BOARD_TYPE_ROBUST32256 = 2
uint8 BOARD_TYPE_ROBUST3264TOFD = 3
uint8 BOARD_TYPE_ROBUST32128TOFD = 4
uint8 BOARD_TYPE_MULTISCAN = 5
uint8 BOARD_TYPE_ROBUST64128 = 6
uint8 BOARD_TYPE_ROBUST64128TOFD = 7
uint8 BOARD_TYPE_MULTISCANPAR = 8

# 编码器模式（线格式 EncoderMode）
uint8 ENCODER_MODE_DOWN = 0
uint8 ENCODER_MODE_UP = 1
uint8 ENCODER_MODE_QUAD = 2
