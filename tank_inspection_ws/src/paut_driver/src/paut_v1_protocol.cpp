/**
 * @file paut_v1_protocol.cpp
 * @brief PAUT UDP 线格式 v1 解析实现。
 *
 * 全部读取都是**显式小端**（逐字节组装），不依赖主机字节序——
 * 现有 paut_udp_receiver.cpp:118 用 host-endian memcpy 是已知反例，本文件刻意不重蹈。
 */

#include "paut_driver/paut_v1_protocol.hpp"

#include <array>
#include <cstring>
#include <utility>

namespace paut_driver {
namespace v1 {

namespace {

// ---------------------------------------------------------------- 显式小端读取

inline uint16_t u16le(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline uint32_t u32le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

inline uint64_t u64le(const uint8_t* p) {
    return static_cast<uint64_t>(u32le(p))
         | (static_cast<uint64_t>(u32le(p + 4)) << 32);
}

inline int32_t i32le(const uint8_t* p) { return static_cast<int32_t>(u32le(p)); }

inline int8_t i8(const uint8_t* p) { return static_cast<int8_t>(p[0]); }

/// 先按小端组装成 uint32（主机序），再按位模式转 float。
/// memcpy 只做位模式搬运，与字节序无关。
inline float f32le(const uint8_t* p) {
    const uint32_t bits = u32le(p);
    float f = 0.0f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

/// 受长度保护的绝对偏移读取。parseConfig 已保证 len >= kConfigHeaderLen，
/// 这些 At 版本是为防御性编程留的（越界返回 0 而不是 UB）。
inline float f32At(const uint8_t* buf, size_t len, size_t off) {
    return (off + 4 <= len) ? f32le(buf + off) : 0.0f;
}
inline uint16_t u16At(const uint8_t* buf, size_t len, size_t off) {
    return (off + 2 <= len) ? u16le(buf + off) : 0;
}
inline uint32_t u32At(const uint8_t* buf, size_t len, size_t off) {
    return (off + 4 <= len) ? u32le(buf + off) : 0;
}
inline uint8_t u8At(const uint8_t* buf, size_t len, size_t off) {
    return (off < len) ? buf[off] : 0;
}

/// 读变长字符串（u16 长度前缀 + UTF-8）。off 会被推进。
/// 长度前缀声明的内容越界时：丢弃内容并把 off 推到末尾（不越界读）。
size_t readStr16(const uint8_t* buf, size_t len, size_t& off, std::string& out) {
    out.clear();
    if (off + 2 > len) return 0;
    const uint16_t n = u16le(buf + off);
    off += 2;
    if (n == 0) return 2;
    if (off + n > len) {
        off = len;
        return 0;
    }
    out.assign(reinterpret_cast<const char*>(buf + off), n);
    off += n;
    return 2 + static_cast<size_t>(n);
}

/// FNV-1a 32 位，用于配置内容指纹。
constexpr uint32_t kFnvOffset = 2166136261u;
constexpr uint32_t kFnvPrime = 16777619u;

inline void fnvAdd(uint32_t& h, uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        h ^= static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
        h *= kFnvPrime;
    }
}

inline void fnvAddF(uint32_t& h, float v) {
    uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    fnvAdd(h, bits);
}

}  // namespace

// ---------------------------------------------------------------- 通用

const char* toString(ParseStatus s) {
    switch (s) {
        case ParseStatus::kOk: return "ok";
        case ParseStatus::kTooShort: return "too_short";
        case ParseStatus::kBadMagic: return "bad_magic";
        case ParseStatus::kBadLength: return "bad_length";
        case ParseStatus::kBadCrc: return "bad_crc";
        case ParseStatus::kBadDimension: return "bad_dimension";
        case ParseStatus::kUnsupported: return "unsupported";
    }
    return "unknown";
}

uint32_t crc32(const uint8_t* data, size_t len) {
    // 标准 IEEE 802.3（多项式 0xEDB88320，反射）。与 Python zlib.crc32 同标准。
    static const std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            t[i] = c;
        }
        return t;
    }();

    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

// ---------------------------------------------------------------- A 段

ParseStatus parseEnvelope(const uint8_t* buf, size_t len, Envelope& out) {
    if (buf == nullptr || len < kEnvelopeLen) return ParseStatus::kTooShort;
    if (u32le(buf) != kMagic) return ParseStatus::kBadMagic;

    out.version_major = u16le(buf + 4);
    out.version_minor = u16le(buf + 6);
    out.packet_type = buf[8];
    out.flags = buf[9];
    out.header_len = u16le(buf + 10);
    out.packet_len = u32le(buf + 12);
    out.crc32 = u32le(buf + 16);

    // packet_len 必须至少容纳封装段，且不超过实际收到的字节数（超了说明被截断）
    if (out.packet_len < kEnvelopeLen || out.packet_len > len) {
        return ParseStatus::kBadLength;
    }
    return ParseStatus::kOk;
}

ParseStatus checkCrc(const uint8_t* buf, size_t len) {
    // 只关心"能不能算 CRC"，故不调用 parseEnvelope（它会因 kBadLength 提前返回）
    if (buf == nullptr || len < kEnvelopeLen) return ParseStatus::kTooShort;
    const uint32_t packet_len = u32le(buf + 12);
    if (packet_len < kEnvelopeLen || packet_len > len) return ParseStatus::kTooShort;

    const uint32_t expect = u32le(buf + 16);
    const uint32_t actual = crc32(buf + kEnvelopeLen, packet_len - kEnvelopeLen);
    return (actual == expect) ? ParseStatus::kOk : ParseStatus::kBadCrc;
}

// ---------------------------------------------------------------- CONFIG 包

uint32_t Config::contentFingerprint() const {
    uint32_t h = kFnvOffset;
    fnvAdd(h, board_ip);
    fnvAdd(h, sample_point_num);
    fnvAdd(h, range_ns);
    fnvAdd(h, ut_voltage_v);
    fnvAdd(h, pulse_width_ns);
    fnvAdd(h, prf_hz);
    fnvAdd(h, start_ns_raw);
    fnvAdd(h, board_type);
    fnvAdd(h, filter_level);

    fnvAddF(h, element_pitch_mm);
    fnvAddF(h, frequency_mhz);
    fnvAdd(h, element_total);
    fnvAdd(h, probe_type);

    fnvAddF(h, wedge_angle_deg);
    fnvAddF(h, wedge_velocity_m_s);
    fnvAddF(h, first_ele_height_mm);
    fnvAddF(h, primary_offset_mm);

    fnvAddF(h, img_height_mm);
    fnvAddF(h, img_height_bias_mm);
    fnvAddF(h, focus_angle_deg);
    fnvAddF(h, focus_depth_mm);
    fnvAddF(h, specimen_vel_m_s);
    fnvAddF(h, board_vel_m_s);
    fnvAdd(h, little_aperture);
    fnvAdd(h, beam_step);
    fnvAdd(h, beam_count);
    fnvAdd(h, scan_mode);

    fnvAddF(h, sampling_rate_hz);
    fnvAdd(h, sampling_rate_src);
    fnvAdd(h, sample_dtype);
    fnvAdd(h, adc_bits);

    fnvAddF(h, enc1_resolution);
    fnvAddF(h, enc2_resolution);
    fnvAdd(h, enc1_unit);
    fnvAdd(h, static_cast<uint32_t>(static_cast<int32_t>(enc1_polarity)));
    fnvAdd(h, enc1_mode);
    fnvAdd(h, enc2_unit);
    fnvAdd(h, static_cast<uint32_t>(static_cast<int32_t>(enc2_polarity)));
    fnvAdd(h, enc2_mode);

    // 名字不参与指纹：换探头时 element_total/pitch 已经会变；
    // 仅改名不影响数据语义，不值得触发一次"配置已变更"。
    return h;
}

ParseStatus parseConfig(const uint8_t* buf, size_t len, const Envelope& env, Config& out) {
    if (buf == nullptr) return ParseStatus::kTooShort;
    if (env.packet_type != kTypeConfig) return ParseStatus::kUnsupported;
    if (len < kConfigHeaderLen) return ParseStatus::kTooShort;

    Config c;
    // ---- 板卡段 ----
    c.board_ip = u32At(buf, len, 24);
    c.sample_point_num = u32At(buf, len, 28);
    c.range_ns = u32At(buf, len, 32);
    c.ut_voltage_v = u16At(buf, len, 36);
    c.pulse_width_ns = u16At(buf, len, 38);
    c.prf_hz = u16At(buf, len, 40);
    c.start_ns_raw = u16At(buf, len, 42);
    c.board_type = u8At(buf, len, 44);
    c.filter_level = u8At(buf, len, 45);

    // ---- 探头段 ----
    c.element_pitch_mm = f32At(buf, len, 48);
    c.frequency_mhz = f32At(buf, len, 52);
    c.element_total = u16At(buf, len, 56);
    c.probe_type = u8At(buf, len, 58);

    // ---- 楔块段 ----
    c.wedge_angle_deg = f32At(buf, len, 60);
    c.wedge_velocity_m_s = f32At(buf, len, 64);
    c.first_ele_height_mm = f32At(buf, len, 68);
    c.primary_offset_mm = f32At(buf, len, 72);

    // ---- 扫查段 ----
    c.img_height_mm = f32At(buf, len, 76);
    c.img_height_bias_mm = f32At(buf, len, 80);
    c.focus_angle_deg = f32At(buf, len, 84);
    c.focus_depth_mm = f32At(buf, len, 88);
    c.specimen_vel_m_s = f32At(buf, len, 92);
    c.board_vel_m_s = f32At(buf, len, 96);
    c.little_aperture = u16At(buf, len, 100);
    c.beam_step = u16At(buf, len, 102);
    c.beam_count = u16At(buf, len, 104);
    c.scan_mode = u8At(buf, len, 106);

    // ---- 时间基准 ----
    c.sampling_rate_hz = f32At(buf, len, 108);
    c.sampling_rate_src = u8At(buf, len, 112);
    c.sample_dtype = u8At(buf, len, 113);
    c.adc_bits = u8At(buf, len, 114);

    // ---- 位置基准 ----
    c.enc1_resolution = f32At(buf, len, 116);
    c.enc2_resolution = f32At(buf, len, 120);
    c.enc1_unit = u8At(buf, len, 124);
    c.enc1_polarity = i8(buf + 125);
    c.enc1_mode = u8At(buf, len, 126);
    c.enc2_unit = u8At(buf, len, 127);
    c.enc2_polarity = i8(buf + 128);
    c.enc2_mode = u8At(buf, len, 129);

    // ---- 尾部 ----
    c.config_seq = u32At(buf, len, 132);

    // ---- 变长区 ----
    size_t off = kConfigHeaderLen;
    readStr16(buf, len, off, c.probe_name);
    readStr16(buf, len, off, c.wedge_name);
    readStr16(buf, len, off, c.notes);

    out = std::move(c);
    return ParseStatus::kOk;
}

// ---------------------------------------------------------------- FRAME 包

ParseStatus parseFrame(const uint8_t* buf, size_t len, const Envelope& env,
                       FrameMeta& out, const uint8_t** payload, size_t* payload_len) {
    if (buf == nullptr) return ParseStatus::kTooShort;
    if (env.packet_type != kTypeFrame) return ParseStatus::kUnsupported;
    if (len < kFrameHeaderLen) return ParseStatus::kTooShort;

    FrameMeta m;
    m.frame_seq = u64le(buf + 24);
    m.device_timestamp_ns = u64le(buf + 32);
    m.scan_axis_encoder = i32le(buf + 40);
    m.index_axis_encoder = i32le(buf + 44);
    m.beam_count = u16le(buf + 48);
    m.sample_count = u16le(buf + 50);
    m.element_total = u16le(buf + 52);
    m.status_flags = u16le(buf + 54);
    m.saturated_count = u32le(buf + 56);
    m.frag_index = u16le(buf + 60);
    m.frag_count = u16le(buf + 62);
    m.config_seq = u32le(buf + 64);
    // 偏移 68 保留 u32

    if (m.beam_count <= 0 || m.sample_count <= 0 ||
        m.beam_count > kMaxDimension || m.sample_count > kMaxDimension) {
        return ParseStatus::kBadDimension;
    }

    const size_t elems = static_cast<size_t>(m.beam_count) * static_cast<size_t>(m.sample_count);
    const size_t need = kFrameHeaderLen + elems;

    // 自洽校验：声明的 packet_len、实际收到的长度、按维度推算的长度三者必须一致
    if (env.packet_len != need || len < need) return ParseStatus::kBadLength;

    out = m;
    if (payload != nullptr) *payload = buf + kFrameHeaderLen;
    if (payload_len != nullptr) *payload_len = elems;
    return ParseStatus::kOk;
}

// ---------------------------------------------------------------- HEARTBEAT 包

ParseStatus parseHeartbeat(const uint8_t* buf, size_t len, const Envelope& env,
                           uint64_t& counter_out) {
    if (buf == nullptr) return ParseStatus::kTooShort;
    if (env.packet_type != kTypeHeartbeat) return ParseStatus::kUnsupported;
    if (len < kHeartbeatLen) return ParseStatus::kTooShort;
    counter_out = u64le(buf + kEnvelopeLen);
    return ParseStatus::kOk;
}

}  // namespace v1
}  // namespace paut_driver
