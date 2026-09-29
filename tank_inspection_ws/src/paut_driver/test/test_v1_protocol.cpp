/**
 * @file test_v1_protocol.cpp
 * @brief PAUT v1 协议解析层的单元测试（gtest，遵循本仓 inspection_sync 的写法）。
 *
 * 核心价值：`kConfigRef` / `kFrameRef` / `kHeartbeatRef` 这三段字节是
 *   由 **Python 参考实现**（scripts/send_paut_12346_synth.py）生成的，
 *   与 C++ 解析器是**两套独立实现**。逐字段断言等于做了一次跨实现比对——
 *   这是唯一能抓出「偏移写错 24 字节」这类静默错误的方法。
 *
 * 不需要 ROS 节点、不需要设备、不需要 socket（只链接纯协议层 paut_v1_protocol.cpp）。
 */

#include "paut_driver/paut_v1_protocol.hpp"

#include <cstring>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using namespace paut_driver::v1;

// ================================================================
// 参考字节：由 scripts/send_paut_12346_synth.py 生成（独立实现）
//   build_config(seq=7, beams=3, samples=4)
// ================================================================
const uint8_t kConfigRef[] = {
    0x50, 0x41, 0x55, 0x54, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x8C, 0x00,
    0xE3, 0x00, 0x00, 0x00, 0xE4, 0x41, 0xC2, 0x5C, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0xA8, 0x01, 0x82, 0x04, 0x00, 0x00, 0x00, 0xA0, 0x86, 0x01, 0x00,
    0x90, 0x01, 0x50, 0x00, 0x7E, 0x14, 0x00, 0x00, 0x03, 0x04, 0x00, 0x00,
    0x9A, 0x99, 0x19, 0x3F, 0x00, 0x00, 0xA0, 0x40, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x12, 0x45, 0x00, 0x00, 0xA0, 0x41,
    0x9A, 0x99, 0x47, 0x42, 0x00, 0x00, 0xA0, 0x41, 0x00, 0x00, 0x20, 0x40,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x41, 0x00, 0x60, 0xB8, 0x45,
    0x00, 0x60, 0xB8, 0x45, 0x04, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x1C, 0x47, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86, 0x42,
    0x00, 0x00, 0x86, 0x42, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x35, 0x4C,
    0x36, 0x34, 0x2D, 0x30, 0x2E, 0x36, 0x78, 0x31, 0x30, 0x0B, 0x00, 0x53,
    0x44, 0x33, 0x2D, 0x4E, 0x30, 0x4C, 0x2D, 0x49, 0x48, 0x43, 0x3B, 0x00,
    0x53, 0x59, 0x4E, 0x54, 0x48, 0x45, 0x54, 0x49, 0x43, 0x20, 0x53, 0x43,
    0x52, 0x49, 0x50, 0x54, 0x53, 0x2F, 0x73, 0x65, 0x6E, 0x64, 0x5F, 0x70,
    0x61, 0x75, 0x74, 0x5F, 0x31, 0x32, 0x33, 0x34, 0x36, 0x5F, 0x73, 0x79,
    0x6E, 0x74, 0x68, 0x2E, 0x70, 0x79, 0x2C, 0x20, 0xE9, 0x9D, 0x9E, 0xE7,
    0x9C, 0x9F, 0xE5, 0xAE, 0x9E, 0xE8, 0xAE, 0xBE, 0xE5, 0xA4, 0x87,
};

// build_frame(seq=12345, beams=3, samples=4, enc_scan=1024, enc_index=-5)
const uint8_t kFrameRef[] = {
    0x50, 0x41, 0x55, 0x54, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x48, 0x00,
    0x54, 0x00, 0x00, 0x00, 0xAD, 0x96, 0x29, 0x2C, 0x00, 0x00, 0x00, 0x00,
    0x39, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0xFB, 0xFF, 0xFF, 0xFF,
    0x03, 0x00, 0x04, 0x00, 0x40, 0x00, 0x8B, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x14, 0x21, 0x2E, 0xFF, 0x14, 0x21, 0x2E, 0xFF, 0x14, 0x21, 0x2E, 0xFF,
};

// build_heartbeat(99)
const uint8_t kHeartbeatRef[] = {
    0x50, 0x41, 0x55, 0x54, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x20, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x31, 0x78, 0x46, 0x3B, 0x00, 0x00, 0x00, 0x00,
    0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// build_frame(..., break_crc=True)：CRC 字段被 XOR 过，校验必须失败
const uint8_t kFrameBadCrcRef[] = {
    0x50, 0x41, 0x55, 0x54, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x48, 0x00,
    0x54, 0x00, 0x00, 0x00, 0xF0, 0xB1, 0x8C, 0x60, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x04, 0x00, 0x40, 0x00, 0x83, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x14, 0x21, 0x2E, 0xFF, 0x14, 0x21, 0x2E, 0xFF, 0x14, 0x21, 0x2E, 0xFF,
};

}  // namespace

// ================================================================

TEST(V1Protocol, Crc32MatchesStandard) {
    const char* s = "123456789";
    EXPECT_EQ(crc32(reinterpret_cast<const uint8_t*>(s), 9), 0xCBF43926u);
    EXPECT_EQ(crc32(nullptr, 0), 0x00000000u);
}

TEST(V1Protocol, ParseConfigCrossImplementation) {
    const uint8_t* buf = kConfigRef;
    const size_t len = sizeof(kConfigRef);

    Envelope env;
    ASSERT_EQ(parseEnvelope(buf, len, env), ParseStatus::kOk);
    EXPECT_EQ(env.packet_type, kTypeConfig);
    EXPECT_EQ(env.version_major, 1);
    EXPECT_EQ(env.version_minor, 0);
    EXPECT_EQ(env.header_len, kConfigHeaderLen);
    EXPECT_EQ(env.packet_len, len);
    EXPECT_EQ(checkCrc(buf, len), ParseStatus::kOk);

    Config c;
    ASSERT_EQ(parseConfig(buf, len, env, c), ParseStatus::kOk);

    // 逐字段比对：期望值 = Python 构造时写入的值
    // ---- 板卡段 ----
    EXPECT_EQ(c.board_ip, 0x8201A8C0u);
    EXPECT_EQ(c.sample_point_num, 4u);
    EXPECT_EQ(c.range_ns, 100000u);
    EXPECT_EQ(c.ut_voltage_v, 400);
    EXPECT_EQ(c.pulse_width_ns, 80);
    EXPECT_EQ(c.prf_hz, 5246);
    EXPECT_EQ(c.start_ns_raw, 0);
    EXPECT_EQ(c.board_type, 3);
    EXPECT_EQ(c.filter_level, 4);

    // ---- 探头段 ----
    EXPECT_FLOAT_EQ(c.element_pitch_mm, 0.6f);
    EXPECT_FLOAT_EQ(c.frequency_mhz, 5.0f);
    EXPECT_EQ(c.element_total, 64);
    EXPECT_EQ(c.probe_type, 0);

    // ---- 楔块段 ----
    EXPECT_FLOAT_EQ(c.wedge_angle_deg, 0.0f);
    EXPECT_FLOAT_EQ(c.wedge_velocity_m_s, 2337.0f);
    EXPECT_FLOAT_EQ(c.first_ele_height_mm, 20.0f);
    EXPECT_FLOAT_EQ(c.primary_offset_mm, 49.9f);

    // ---- 扫查段 ----
    EXPECT_FLOAT_EQ(c.img_height_mm, 20.0f);
    EXPECT_FLOAT_EQ(c.img_height_bias_mm, 2.5f);
    EXPECT_FLOAT_EQ(c.focus_angle_deg, 0.0f);
    EXPECT_FLOAT_EQ(c.focus_depth_mm, 15.0f);
    EXPECT_FLOAT_EQ(c.specimen_vel_m_s, 5900.0f);
    EXPECT_FLOAT_EQ(c.board_vel_m_s, 5900.0f);
    EXPECT_EQ(c.little_aperture, 4);
    EXPECT_EQ(c.beam_step, 1);
    EXPECT_EQ(c.beam_count, 3);
    EXPECT_EQ(c.scan_mode, kScanModeLinear);

    // ---- 时间基准 ----  samples=4, range_ns=100000 → fs = 4/1e-4 = 40000
    EXPECT_FLOAT_EQ(c.sampling_rate_hz, 40000.0f);
    EXPECT_EQ(c.sampling_rate_src, kRateSrcDerivedUnverified);
    EXPECT_EQ(c.sample_dtype, 0);
    EXPECT_EQ(c.adc_bits, 0);

    // ---- 位置基准 ----
    EXPECT_FLOAT_EQ(c.enc1_resolution, 67.0f);
    EXPECT_FLOAT_EQ(c.enc2_resolution, 67.0f);
    EXPECT_EQ(c.enc1_unit, kEncUnitUnknown);
    EXPECT_EQ(c.enc1_polarity, 1);
    EXPECT_EQ(c.enc1_mode, 0);
    EXPECT_EQ(c.enc2_unit, kEncUnitUnknown);
    EXPECT_EQ(c.enc2_polarity, 1);
    EXPECT_EQ(c.enc2_mode, 0);

    EXPECT_EQ(c.config_seq, 7u);

    // ---- 变长区 ----
    EXPECT_EQ(c.probe_name, "5L64-0.6x10");
    EXPECT_EQ(c.wedge_name, "SD3-N0L-IHC");
    EXPECT_EQ(c.notes.rfind("SYNTHETIC", 0), 0u);
    EXPECT_NE(c.notes.find("非真实设备"), std::string::npos);
}

TEST(V1Protocol, ConfigFingerprintSemantics) {
    Envelope env;
    parseEnvelope(kConfigRef, sizeof(kConfigRef), env);
    Config c;
    parseConfig(kConfigRef, sizeof(kConfigRef), env, c);

    const uint32_t fp1 = c.contentFingerprint();

    Config same = c;
    EXPECT_EQ(same.contentFingerprint(), fp1);   // 稳定

    Config seq_changed = c;
    seq_changed.config_seq = 999;                // 设备每 2s 重发同一 seq
    EXPECT_EQ(seq_changed.contentFingerprint(), fp1) << "config_seq 不应影响指纹";

    Config field_changed = c;
    field_changed.beam_count = 99;
    EXPECT_NE(field_changed.contentFingerprint(), fp1) << "真实变化必须被检出";
}

TEST(V1Protocol, ParseFrameCrossImplementation) {
    const uint8_t* buf = kFrameRef;
    const size_t len = sizeof(kFrameRef);

    Envelope env;
    ASSERT_EQ(parseEnvelope(buf, len, env), ParseStatus::kOk);
    EXPECT_EQ(env.packet_type, kTypeFrame);
    EXPECT_EQ(checkCrc(buf, len), ParseStatus::kOk);

    FrameMeta m;
    const uint8_t* payload = nullptr;
    size_t plen = 0;
    ASSERT_EQ(parseFrame(buf, len, env, m, &payload, &plen), ParseStatus::kOk);

    EXPECT_EQ(m.frame_seq, 12345u);
    EXPECT_EQ(m.device_timestamp_ns, 0u);
    EXPECT_EQ(m.scan_axis_encoder, 1024);
    EXPECT_EQ(m.index_axis_encoder, -5);
    EXPECT_EQ(m.beam_count, 3);
    EXPECT_EQ(m.sample_count, 4);
    EXPECT_EQ(m.element_total, 64);
    EXPECT_EQ(m.status_flags, 0x8Bu);
    EXPECT_NE(m.status_flags & kStatusHasData, 0);
    EXPECT_NE(m.status_flags & kStatusSaturated, 0);
    EXPECT_NE(m.status_flags & kStatusEncoderValid, 0);
    EXPECT_NE(m.status_flags & kStatusSampleDtypeU8, 0);
    EXPECT_EQ(m.saturated_count, 3u);
    EXPECT_EQ(m.frag_index, 0);
    EXPECT_EQ(m.frag_count, 1);
    EXPECT_EQ(m.config_seq, 1u);

    ASSERT_NE(payload, nullptr);
    EXPECT_EQ(plen, 3u * 4u);
    // 合成波形: 每波束 4 样本 = {20, 33, 46, 255}
    const uint8_t expect_wave[4] = {20, 33, 46, 255};
    for (size_t b = 0; b < 3; ++b) {
        for (size_t s = 0; s < 4; ++s) {
            EXPECT_EQ(payload[b * 4 + s], expect_wave[s]) << "beam=" << b << " sample=" << s;
        }
    }
}

TEST(V1Protocol, ParseHeartbeat) {
    const uint8_t* buf = kHeartbeatRef;
    const size_t len = sizeof(kHeartbeatRef);
    Envelope env;
    ASSERT_EQ(parseEnvelope(buf, len, env), ParseStatus::kOk);
    EXPECT_EQ(env.packet_type, kTypeHeartbeat);
    EXPECT_EQ(env.packet_len, kHeartbeatLen);
    uint64_t counter = 0;
    ASSERT_EQ(parseHeartbeat(buf, len, env, counter), ParseStatus::kOk);
    EXPECT_EQ(counter, 99u);
}

TEST(V1Protocol, RejectsBadCrc) {
    Envelope env;
    ASSERT_EQ(parseEnvelope(kFrameBadCrcRef, sizeof(kFrameBadCrcRef), env), ParseStatus::kOk);
    EXPECT_EQ(checkCrc(kFrameBadCrcRef, sizeof(kFrameBadCrcRef)), ParseStatus::kBadCrc);
}

TEST(V1Protocol, RejectsBadMagic) {
    uint8_t bad[32];
    std::memcpy(bad, kHeartbeatRef, sizeof(bad));
    bad[0] = 0x00;
    Envelope env;
    EXPECT_EQ(parseEnvelope(bad, sizeof(bad), env), ParseStatus::kBadMagic);
}

TEST(V1Protocol, RejectsShortAndTruncated) {
    Envelope env;
    EXPECT_EQ(parseEnvelope(kHeartbeatRef, 10, env), ParseStatus::kTooShort);
    EXPECT_EQ(parseEnvelope(kHeartbeatRef, kHeartbeatLen - 1, env), ParseStatus::kBadLength);
    EXPECT_EQ(parseEnvelope(kFrameRef, sizeof(kFrameRef) - 4, env), ParseStatus::kBadLength);
}

TEST(V1Protocol, RejectsBadDimension) {
    uint8_t bad[sizeof(kFrameRef)];
    std::memcpy(bad, kFrameRef, sizeof(bad));
    bad[48] = 0x00;  // beam_count low byte
    bad[49] = 0x00;
    Envelope env;
    ASSERT_EQ(parseEnvelope(bad, sizeof(bad), env), ParseStatus::kOk);
    FrameMeta m;
    const uint8_t* p = nullptr;
    size_t pl = 0;
    EXPECT_EQ(parseFrame(bad, sizeof(bad), env, m, &p, &pl), ParseStatus::kBadDimension);
}

TEST(V1Protocol, RejectsInconsistentPacketLen) {
    // 维度合法(3x4 → 应 84B)，但 packet_len 写成 88
    uint8_t buf[sizeof(kFrameRef) + 4];
    std::memcpy(buf, kFrameRef, sizeof(kFrameRef));
    std::memset(buf + sizeof(kFrameRef), 0, 4);
    buf[12] = 0x58;  // packet_len = 88
    buf[13] = 0x00;

    Envelope env;
    ASSERT_EQ(parseEnvelope(buf, sizeof(buf), env), ParseStatus::kOk);
    FrameMeta m;
    const uint8_t* p = nullptr;
    size_t pl = 0;
    EXPECT_EQ(parseFrame(buf, sizeof(buf), env, m, &p, &pl), ParseStatus::kBadLength);
}

TEST(V1Protocol, RejectsMismatchedPacketType) {
    Envelope env;
    parseEnvelope(kFrameRef, sizeof(kFrameRef), env);
    Config c;
    EXPECT_EQ(parseConfig(kFrameRef, sizeof(kFrameRef), env, c), ParseStatus::kUnsupported);
}

TEST(V1Protocol, RealisticSizeMath) {
    // 设备端实发 61 波束 × 896 采样点
    const uint16_t beams = 61, samples = 896;
    const size_t expect = kFrameHeaderLen + static_cast<size_t>(beams) * samples;
    EXPECT_EQ(expect, 54728u);
    EXPECT_LE(expect, kMaxPacketLen);

    std::vector<uint8_t> buf(expect, 0);
    buf[0] = 0x50; buf[1] = 0x41; buf[2] = 0x55; buf[3] = 0x54;  // magic
    buf[8] = 0x02;                                                // type = FRAME
    const uint32_t plen = static_cast<uint32_t>(expect);
    buf[12] = static_cast<uint8_t>(plen & 0xFF);
    buf[13] = static_cast<uint8_t>((plen >> 8) & 0xFF);
    buf[14] = static_cast<uint8_t>((plen >> 16) & 0xFF);
    buf[15] = static_cast<uint8_t>((plen >> 24) & 0xFF);
    buf[48] = static_cast<uint8_t>(beams & 0xFF);
    buf[49] = static_cast<uint8_t>(beams >> 8);
    buf[50] = static_cast<uint8_t>(samples & 0xFF);
    buf[51] = static_cast<uint8_t>(samples >> 8);
    buf[62] = 0x01;  // frag_count = 1

    Envelope env;
    ASSERT_EQ(parseEnvelope(buf.data(), buf.size(), env), ParseStatus::kOk);
    FrameMeta m;
    const uint8_t* p = nullptr;
    size_t pl = 0;
    ASSERT_EQ(parseFrame(buf.data(), buf.size(), env, m, &p, &pl), ParseStatus::kOk);
    EXPECT_EQ(m.beam_count, beams);
    EXPECT_EQ(m.sample_count, samples);
    EXPECT_EQ(pl, static_cast<size_t>(beams) * samples);
}
