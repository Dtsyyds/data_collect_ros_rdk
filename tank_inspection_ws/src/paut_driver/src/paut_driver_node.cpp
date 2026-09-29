/**
 * @file paut_driver_node.cpp
 * @brief PAUT UDP v1(:12346) 驱动节点实现。
 *
 * 收包线程（PautV1Receiver 内部）同步调用 onConfig/onFrame，本文件在这两个回调里
 * **直接构造并发布 ROS 消息**。这是从 v0 继承的约定：不排队、不缓存帧（帧率 ~86Hz，
 * 54 KB/帧，缓存只会引入不必要的内存与延迟）。
 *
 * 唯一例外是 CONFIG：设备每 2s 重发同一份，用**内容指纹**去重后再发布，
 * 靠 QoS 的 transient_local 让晚加入的订阅者也拿得到（UDP 没有这个能力，
 * 设备端才需要周期重发兜底）。
 */

#include "paut_driver/paut_driver_node.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <utility>

namespace paut_driver {

namespace {

/// 契约 C1.2：设备无硬件时戳，header.stamp 取主机收包时刻。
int64_t systemTimeNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

const char* scanModeName(uint8_t m) {
    switch (m) {
        case v1::kScanModeLinear: return "linear";
        case v1::kScanModeSector: return "sector";
        case v1::kScanModeTofm: return "tofm";
        case v1::kScanModeTofd: return "tofd";
        case v1::kScanModePwi: return "pwi";
        default: return "unknown";
    }
}

}  // namespace

PautDriverNode::PautDriverNode(const rclcpp::NodeOptions& options)
    : Node("paut_driver_node", options) {
    declareParameters();
    loadParameters();

    // 数据话题：与 qos_overrides.yaml 的 raw_v2 条目保持一致
    // （best_effort + volatile + keep_last(100)）。若不一致，rosbag2 会静默收不到数据。
    auto frame_qos = rclcpp::SensorDataQoS();
    frame_qos.keep_last(100);
    frame_pub_ = create_publisher<inspection_interfaces::msg::PautFrameV2>(
        "/inspection/paut/raw_v2", frame_qos);

    // 配置话题：transient_local 闩锁 —— 晚加入的订阅者（如采集中途启动的录制器、
    // 回放时的分析脚本）自动补发最后一份。注意**不要**在 qos_overrides.yaml 里
    // 给它加 durability: volatile，那会让配置消息静默丢失。
    rclcpp::QoS config_qos(rclcpp::KeepLast(1));
    config_qos.transient_local();
    config_qos.reliable();
    config_pub_ = create_publisher<inspection_interfaces::msg::PautConfig>(
        "/inspection/paut/config", config_qos);

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10).reliable());
    diagnostics_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&PautDriverNode::publishDiagnostics, this));

    receiver_ = std::make_unique<PautV1Receiver>(port_);
    receiver_->setOnConfig(std::bind(&PautDriverNode::onConfig, this, std::placeholders::_1));
    receiver_->setOnFrame(std::bind(&PautDriverNode::onFrame, this, std::placeholders::_1,
                                    std::placeholders::_2, std::placeholders::_3));
    bind_ok_ = receiver_->start();
    if (!bind_ok_) {
        RCLCPP_ERROR(get_logger(),
            "PAUT UDP bind 端口 %u 失败 (errno=%d: %s)。端口可能被其他进程占用"
            "（例如 Python 参考实现 scripts/recv_paut_12346.py，或残留的旧节点）。",
            static_cast<unsigned>(port_), receiver_->lastError(),
            std::strerror(receiver_->lastError()));
    } else {
        RCLCPP_INFO(get_logger(),
            "PAUT v1 节点就绪: 监听端口 %u -> /inspection/paut/raw_v2 + /inspection/paut/config"
            " (sensor_id=%s, calibration_id=%s)",
            static_cast<unsigned>(port_), sensor_id_.c_str(), calibration_id_.c_str());
        RCLCPP_WARN(get_logger(),
            "设备无硬件时戳: header.stamp 为主机收包时刻, device_timestamp_ns 恒 0; "
            "采样率为反推值(未验证), 编码器单位未知 -- 详见 PautConfig 的 notes 字段。");
    }
}

PautDriverNode::~PautDriverNode() {
    // 先 join 收包线程，确保不再有回调访问成员后，成员才被析构。
    if (receiver_) {
        receiver_->stop();
    }
}

void PautDriverNode::declareParameters() {
    declare_parameter("port", 12346);
    declare_parameter("sensor_id", "UNASSIGNED");
    declare_parameter("calibration_id", "UNASSIGNED");
    declare_parameter("frame_id", "paut_probe_link");
    // 注: v0 的 sample_encoding / sampling_rate_hz / gain_db / sound_velocity_m_s 已删除 --
    //     这些量现在的权威值来自设备的 CONFIG 包（PautConfig），不再需要节点参数。
}

void PautDriverNode::loadParameters() {
    port_ = static_cast<uint16_t>(get_parameter("port").as_int());
    sensor_id_ = get_parameter("sensor_id").as_string();
    calibration_id_ = get_parameter("calibration_id").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
}

void PautDriverNode::onConfig(const v1::Config& cfg) {
    const uint32_t fp = cfg.contentFingerprint();
    const bool first = !config_seen_.load();
    const uint32_t prev_fp = config_fingerprint_.load();

    // 无论是否对外发布，都要更新这些供诊断与帧过滤使用
    scan_mode_.store(cfg.scan_mode);
    config_seq_.store(cfg.config_seq);

    if (!first && fp == prev_fp) {
        return;  // 设备每 2s 的重发，内容没变
    }
    config_fingerprint_.store(fp);
    config_seen_.store(true);

    using inspection_interfaces::msg::PautConfig;
    PautConfig msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.sensor_id = sensor_id_;
    msg.calibration_id = calibration_id_;
    msg.config_seq = cfg.config_seq;

    // ---- 板卡段 ----
    msg.board_ip = cfg.board_ip;
    msg.sample_point_num = cfg.sample_point_num;
    msg.range_ns = cfg.range_ns;
    msg.ut_voltage_v = cfg.ut_voltage_v;
    msg.pulse_width_ns = cfg.pulse_width_ns;
    msg.prf_hz = cfg.prf_hz;
    msg.start_ns_raw = cfg.start_ns_raw;
    msg.board_type = cfg.board_type;
    msg.filter_level = cfg.filter_level;
    // 设备端缺陷: SendParamsManager 每次下发参数都做 startNs += wedgeDelay（累积漂移），
    // 故 start_ns_raw 不可信。设备目前不提供该标记，这里恒填 false（保守）。
    msg.start_ns_trusted = false;

    // ---- 探头段 ----
    msg.element_pitch_mm = cfg.element_pitch_mm;
    msg.frequency_mhz = cfg.frequency_mhz;
    msg.element_total = cfg.element_total;
    msg.probe_type = cfg.probe_type;

    // ---- 楔块段 ----
    msg.wedge_angle_deg = cfg.wedge_angle_deg;
    msg.wedge_velocity_m_s = cfg.wedge_velocity_m_s;
    msg.first_ele_height_mm = cfg.first_ele_height_mm;
    msg.primary_offset_mm = cfg.primary_offset_mm;

    // ---- 扫查段 ----
    msg.img_height_mm = cfg.img_height_mm;
    msg.img_height_bias_mm = cfg.img_height_bias_mm;
    msg.focus_angle_deg = cfg.focus_angle_deg;
    msg.focus_depth_mm = cfg.focus_depth_mm;
    msg.specimen_vel_m_s = cfg.specimen_vel_m_s;
    msg.board_vel_m_s = cfg.board_vel_m_s;
    msg.little_aperture = cfg.little_aperture;
    msg.beam_step = cfg.beam_step;
    msg.beam_count = cfg.beam_count;
    msg.scan_mode = cfg.scan_mode;

    // ---- 时间基准（采样率为反推值，见 PautConfig 注释）----
    msg.sampling_rate_hz = cfg.sampling_rate_hz;
    msg.sampling_rate_src = cfg.sampling_rate_src;
    msg.sample_dtype = cfg.sample_dtype;
    msg.adc_bits = cfg.adc_bits;

    // ---- 位置基准 ----
    msg.enc1_resolution = cfg.enc1_resolution;
    msg.enc2_resolution = cfg.enc2_resolution;
    msg.enc1_unit = cfg.enc1_unit;
    msg.enc2_unit = cfg.enc2_unit;
    msg.enc1_polarity = cfg.enc1_polarity;
    msg.enc2_polarity = cfg.enc2_polarity;
    msg.enc1_mode = cfg.enc1_mode;
    msg.enc2_mode = cfg.enc2_mode;

    // ---- 变长区 ----
    msg.probe_name = cfg.probe_name;
    msg.wedge_name = cfg.wedge_name;
    msg.notes = cfg.notes;

    config_pub_->publish(std::move(msg));
    RCLCPP_INFO(get_logger(),
        "CONFIG 已发布: config_seq=%u %s %u阵元 pitch=%.3fmm beam_count=%u scan_mode=%s",
        cfg.config_seq, cfg.probe_name.c_str(), cfg.element_total, cfg.element_pitch_mm,
        cfg.beam_count, scanModeName(cfg.scan_mode));
}

void PautDriverNode::onFrame(const v1::FrameMeta& meta, const uint8_t* payload,
                             size_t payload_len) {
    // 本版只支持线扫。其余模式的**数据段结构不同**（扇扫/全聚焦各自不同），
    // 静默按线扫解释会产出无意义的数据，故丢弃并计数。
    const uint8_t mode = scan_mode_.load();
    if (mode != v1::kScanModeLinear) {
        non_linear_dropped_.fetch_add(1);
        return;
    }

    if (!config_seen_.load()) {
        // 设备启动初期 FRAME 可能早于首份 CONFIG（CONFIG 每 2s 重发、FRAME ~86Hz）。
        // 帧本身仍然发布 —— 它的 config_seq 指向一份稍后会到的配置。
        // 校验器应把这种情况记为 warning 而非 error（契约 C7.2 的竞态例外）。
        frames_without_config_.fetch_add(1);
    }

    using inspection_interfaces::msg::PautFrameV2;
    PautFrameV2 msg;
    const int64_t recv_ns = systemTimeNs();
    msg.header.stamp.sec = static_cast<int32_t>(recv_ns / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(recv_ns % 1000000000LL);
    msg.header.frame_id = frame_id_;

    msg.schema_version = 1;
    msg.config_seq = (meta.config_seq != 0) ? meta.config_seq : config_seq_.load();
    msg.device_timestamp_ns = meta.device_timestamp_ns;  // 恒 0 = 设备不提供
    msg.timestamp_source = v1::kTsHostReceive;
    msg.frame_seq = meta.frame_seq;
    msg.scan_axis_encoder = meta.scan_axis_encoder;
    msg.index_axis_encoder = meta.index_axis_encoder;
    msg.beam_count = meta.beam_count;
    msg.sample_count = meta.sample_count;
    msg.element_total = meta.element_total;
    msg.status_flags = meta.status_flags;
    msg.saturated_count = meta.saturated_count;
    // payload 指向收包缓冲内部，回调返回即失效，故这里必须拷贝
    msg.samples.assign(payload, payload + payload_len);

    frame_pub_->publish(std::move(msg));

    last_beam_.store(meta.beam_count);
    last_sample_.store(meta.sample_count);
    frames_published_.fetch_add(1);
}

void PautDriverNode::publishDiagnostics() {
    using diagnostic_msgs::msg::DiagnosticArray;
    using diagnostic_msgs::msg::DiagnosticStatus;
    using diagnostic_msgs::msg::KeyValue;

    DiagnosticArray array;
    array.header.stamp = now();
    DiagnosticStatus status;
    status.name = "paut_driver/acquisition";
    status.hardware_id = sensor_id_;

    const uint64_t frames = receiver_ ? receiver_->frameCount() : 0;
    const uint64_t crc_fails = receiver_ ? receiver_->crcFailCount() : 0;
    const int64_t hb_ns = receiver_ ? receiver_->lastHeartbeatTimeNs() : 0;
    const double hb_age_s =
        (hb_ns > 0) ? static_cast<double>(systemTimeNs() - hb_ns) / 1e9 : -1.0;

    // 三态 + 心跳龄期：区分「设备不在线」与「在线但未采集」——
    // 这是 v0 做不到的（v0 没有心跳，两者诊断完全一样）。
    if (!bind_ok_) {
        status.level = DiagnosticStatus::ERROR;
        status.message = "udp bind failed; port likely in use";
    } else if (hb_age_s < 0.0) {
        status.level = DiagnosticStatus::ERROR;
        status.message = "no heartbeat yet; device not sending or network unreachable";
    } else if (hb_age_s > 5.0) {
        status.level = DiagnosticStatus::ERROR;
        status.message = "device offline (no heartbeat for >5s)";
    } else if (frames == 0) {
        status.level = DiagnosticStatus::WARN;
        status.message = "device online but not acquiring (heartbeat ok, no frame)";
    } else if (crc_fails > 0) {
        status.level = DiagnosticStatus::WARN;
        status.message = "frames flowing; some datagrams failed CRC";
    } else {
        status.level = DiagnosticStatus::WARN;
        status.message = "ok; host receive timestamps and unverified calibration";
    }

    const auto add_value = [&status](const std::string& key, const std::string& value) {
        KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
    };
    add_value("port", std::to_string(static_cast<unsigned>(port_)));
    add_value("timestamp_source", "host_receive_completion");
    add_value("sample_layout", "beam_major_uint8");
    add_value("parsed_frame_count", std::to_string(frames));
    add_value("published_frame_count", std::to_string(frames_published_.load()));
    add_value("garbage_datagram_count",
        receiver_ ? std::to_string(receiver_->garbageCount()) : "0");
    add_value("crc_fail_count", std::to_string(crc_fails));
    add_value("fragmented_dropped_count",
        receiver_ ? std::to_string(receiver_->fragmentedDroppedCount()) : "0");
    add_value("non_linear_scan_dropped_frames", std::to_string(non_linear_dropped_.load()));
    add_value("frames_without_config", std::to_string(frames_without_config_.load()));
    add_value("config_packet_count",
        receiver_ ? std::to_string(receiver_->configCount()) : "0");
    add_value("config_seq", std::to_string(config_seq_.load()));
    add_value("config_available", config_seen_.load() ? "true" : "false");
    add_value("scan_mode", scanModeName(scan_mode_.load()));
    add_value("last_frame_rate_hz",
        receiver_ ? std::to_string(receiver_->frameRateHz()) : "0.0");
    add_value("heartbeat_count",
        receiver_ ? std::to_string(receiver_->heartbeatCount()) : "0");
    add_value("heartbeat_age_s", (hb_age_s < 0.0) ? "never" : std::to_string(hb_age_s));
    add_value("beam_count", std::to_string(last_beam_.load()));
    add_value("sample_count", std::to_string(last_sample_.load()));
    add_value("sensor_id", sensor_id_);
    add_value("calibration_id", calibration_id_);
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
}

}  // namespace paut_driver
