/**
 * @file paut_v1_protocol.hpp
 * @brief PAUT UDP 线格式 v1（:12346）的纯协议层：常量、结构体、CRC32、解析函数。
 *
 * @note 本层**不做任何 I/O**——只把 `const uint8_t*` 解析成结构体，因此可以完全
 *       用合成字节做单元测试，不需要 ROS、不需要设备、不需要 socket。
 * @note 线格式权威实现（两者必须逐字段一致）：
 *         - 采集端参考实现 `scripts/recv_paut_12346.py`
 *         - 设备端 `PautRawBroadcaster.cs` / `PautWireFormat.cs`
 *
 * 报文结构（全包**显式小端**、紧凑排列无填充）：
 *   ┌────────────────────────────────────────────┐
 *   │ A. 包封装  24 B   魔数/版本/类型/长度/CRC32  │
 *   ├────────────────────────────────────────────┤
 *   │ B/C/D. 载荷（按 packet_type 解释）           │
 *   └────────────────────────────────────────────┘
 *
 * 三类包由封装段的 packet_type 区分：
 *   CONFIG(1)    板卡+探头+楔块+扫查+时间基准+位置基准  启动/变更时/每 2s 重发
 *   FRAME(2)     帧块 48 B + 数据段 beam_count*sample_count 字节
 *   HEARTBEAT(3) 仅 u64 计数（设备未开采集时也发，用于判活）
 */

#ifndef PAUT_DRIVER_V1_PROTOCOL_HPP_
#define PAUT_DRIVER_V1_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace paut_driver {
namespace v1 {

// ---------------------------------------------------------------- 常量

constexpr uint32_t kMagic = 0x54554150u;  ///< "PAUT" 小端
constexpr uint16_t kVersionMajor = 1;
constexpr uint16_t kVersionMinor = 0;

constexpr uint8_t kTypeConfig = 1;
constexpr uint8_t kTypeFrame = 2;
constexpr uint8_t kTypeHeartbeat = 3;

constexpr size_t kEnvelopeLen = 24;      ///< A 段
constexpr size_t kFrameBlockLen = 48;    ///< C 段
constexpr size_t kFrameHeaderLen = kEnvelopeLen + kFrameBlockLen;  ///< = 72，FRAME 数据段起点
constexpr size_t kConfigHeaderLen = 140; ///< CONFIG 变长区起点
constexpr size_t kHeartbeatLen = 32;

/// 单包上限（设备端发不出去的上界，留 UDP 65507 的余量）。
constexpr size_t kMaxPacketLen = 60000;

/// 维度上界，防畸形包导致越界。
constexpr int32_t kMaxDimension = 4096;

// status_flags 位（位号与设备端 STATUS_BITS 绑定，勿改）
enum StatusBits : uint16_t {
    kStatusHasData = 1u << 0,
    kStatusSaturated = 1u << 1,
    kStatusNoSync = 1u << 2,
    kStatusEncoderValid = 1u << 3,
    kStatusIsFragmented = 1u << 4,
    kStatusCalibrated = 1u << 5,
    kStatusStartNsTrusted = 1u << 6,
    kStatusSampleDtypeU8 = 1u << 7,
};

/// header.stamp 来源（契约 C1.3）。本设备恒为 kTsHostReceive。
enum TimestampSource : uint8_t {
    kTsUnknown = 0,
    kTsDeviceHardware = 1,
    kTsDeviceSoftware = 2,
    kTsHostReceive = 3,
};

/// 采样率来源（设备不直接提供采样率，本值是反推的）。
enum RateSource : uint8_t {
    kRateSrcUnknown = 0,
    kRateSrcDerivedUnverified = 1,
};

/// 编码器单位。未知时**不得**用 resolution 做任何换算（契约 C2.2/C2.5）。
enum EncoderUnit : uint8_t {
    kEncUnitUnknown = 0,
    kEncUnitCountPerMm = 1,
    kEncUnitMmPerCount = 2,
};

/// 扫查模式（顺序与设备端一致）。本版驱动只接受 kScanModeLinear。
enum ScanMode : uint8_t {
    kScanModeLinear = 0,
    kScanModeSector = 1,
    kScanModeTofm = 2,
    kScanModeTofd = 3,
    kScanModePwi = 4,
};

// ---------------------------------------------------------------- 解析结果

enum class ParseStatus {
    kOk = 0,
    kTooShort,     ///< 长度不足以容纳声明的段
    kBadMagic,     ///< 魔数不符（很可能收到的是别的协议）
    kBadLength,    ///< packet_len 与按内容推算的长度不符
    kBadCrc,       ///< CRC32 校验失败
    kBadDimension, ///< 维度非法（<=0 或超上界）
    kUnsupported,  ///< 本版不支持的取值（如 scan_mode != 线性扫）
};

const char* toString(ParseStatus s);

// ---------------------------------------------------------------- 结构体

/// A 段：包封装。
struct Envelope {
    uint16_t version_major = 0;
    uint16_t version_minor = 0;
    uint8_t packet_type = 0;
    uint8_t flags = 0;
    uint16_t header_len = 0;
    uint32_t packet_len = 0;
    uint32_t crc32 = 0;
};

/// CONFIG 包（39 个线格式字段 + 3 个变长字符串）。
/// 字段顺序与线格式偏移表一致，便于逐条核对。
struct Config {
    // ---- 板卡段 [24,46) ----
    uint32_t board_ip = 0;           ///< .NET IPAddress.Address 形式，还原点分需字节倒序
    uint32_t sample_point_num = 0;   ///< 板卡配置的采样点数（实测 1000）
    uint32_t range_ns = 0;           ///< 时基范围 [ns]
    uint16_t ut_voltage_v = 0;
    uint16_t pulse_width_ns = 0;
    uint16_t prf_hz = 0;             ///< 脉冲重复频率（非采样率）
    uint16_t start_ns_raw = 0;       ///< 采样起点 [ns]，见 start_ns_trusted
    uint8_t board_type = 0;
    uint8_t filter_level = 0;        ///< 硬件滤波档位（0=不滤波），不是 kHz

    // ---- 探头段 [48,60) ----
    float element_pitch_mm = 0.0f;
    float frequency_mhz = 0.0f;
    uint16_t element_total = 0;
    uint8_t probe_type = 0;

    // ---- 楔块段 [60,76) ----
    float wedge_angle_deg = 0.0f;
    float wedge_velocity_m_s = 0.0f;
    float first_ele_height_mm = 0.0f;
    float primary_offset_mm = 0.0f;

    // ---- 扫查段 [76,108) ----
    float img_height_mm = 0.0f;       ///< 成像窗口高度（非完整声程）
    float img_height_bias_mm = 0.0f;  ///< 成像起始偏移（第 0 行对应多深）
    float focus_angle_deg = 0.0f;
    float focus_depth_mm = 0.0f;
    float specimen_vel_m_s = 0.0f;    ///< 成像算法实际使用的声速
    float board_vel_m_s = 0.0f;       ///< BoardModel 的纵波声速（可能与上面不一致）
    uint16_t little_aperture = 0;
    uint16_t beam_step = 0;
    uint16_t beam_count = 0;
    uint8_t scan_mode = 0;

    // ---- 时间基准 [108,116) ----
    float sampling_rate_hz = 0.0f;    ///< 反推值，见 sampling_rate_src
    uint8_t sampling_rate_src = 0;
    uint8_t sample_dtype = 0;
    uint8_t adc_bits = 0;             ///< 0 = 未知

    // ---- 位置基准 [116,132) ----
    float enc1_resolution = 0.0f;
    float enc2_resolution = 0.0f;
    uint8_t enc1_unit = 0;
    int8_t enc1_polarity = 0;
    uint8_t enc1_mode = 0;
    uint8_t enc2_unit = 0;
    int8_t enc2_polarity = 0;
    uint8_t enc2_mode = 0;

    // ---- 尾部 [132,140) ----
    uint32_t config_seq = 0;

    // ---- 变长区（偏移 140 起，各自 u16 长度前缀 + UTF-8）----
    std::string probe_name;
    std::string wedge_name;
    std::string notes;                ///< 设备端「自我说明」：不可信/未验证项都在这里

    /// 内容指纹（不含 config_seq）。设备每 2s 重发同一 seq，
    /// 故**不能**靠 seq 判「配置是否变化」，必须用内容指纹。
    uint32_t contentFingerprint() const;
};

/// FRAME 包的帧块（数据段由调用方按指针+长度取用，避免拷贝）。
struct FrameMeta {
    uint64_t frame_seq = 0;           ///< 设备帧号。已知不可靠，勿用作分组键
    uint64_t device_timestamp_ns = 0; ///< 恒 0 = 设备不提供时钟
    int32_t scan_axis_encoder = 0;
    int32_t index_axis_encoder = 0;
    uint16_t beam_count = 0;
    uint16_t sample_count = 0;
    uint16_t element_total = 0;
    uint16_t status_flags = 0;
    uint32_t saturated_count = 0;
    uint16_t frag_index = 0;
    uint16_t frag_count = 0;
    uint32_t config_seq = 0;
};

// ---------------------------------------------------------------- 解析函数

/// 标准 IEEE 802.3 CRC32（多项式 0xEDB88320，反射）。与 Python 的 zlib.crc32 同标准。
uint32_t crc32(const uint8_t* data, size_t len);

/// 解析 A 段。校验魔数与 packet_len 基本自洽，**不含 CRC 校验**（见 checkCrc）。
ParseStatus parseEnvelope(const uint8_t* buf, size_t len, Envelope& out);

/// 按设计约定校验 CRC：覆盖 [kEnvelopeLen, packet_len)。
/// 返回 kOk / kTooShort / kBadCrc。
ParseStatus checkCrc(const uint8_t* buf, size_t len);

/// 解析 CONFIG 包（含变长字符串）。调用前应先 parseEnvelope + checkCrc。
ParseStatus parseConfig(const uint8_t* buf, size_t len, const Envelope& env, Config& out);

/// 解析 FRAME 包的帧块，并通过 payload / payload_len 返回数据段视图（**不拷贝**）。
/// 数据段格式：[beam_count][sample_count]，行优先，每样本 1 字节（uint8）。
ParseStatus parseFrame(const uint8_t* buf, size_t len, const Envelope& env,
                       FrameMeta& out, const uint8_t** payload, size_t* payload_len);

/// 解析 HEARTBEAT 包的计数值。
ParseStatus parseHeartbeat(const uint8_t* buf, size_t len, const Envelope& env,
                           uint64_t& counter_out);

}  // namespace v1
}  // namespace paut_driver

#endif  // PAUT_DRIVER_V1_PROTOCOL_HPP_
