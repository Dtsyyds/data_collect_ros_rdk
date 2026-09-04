/**
 * @file paut_driver_node.cpp
 * @brief PAUT UDP 驱动节点: 收帧解码后发布 inspection_interfaces/msg/PautFrame 到 /inspection/paut/raw。
 * @note header.stamp = 主机收包完成时刻; 采样率/增益/声速在标定前保持 unknown(NaN/0)。
 */

#include "paut_driver/paut_driver_node.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace paut_driver {

PautDriverNode::PautDriverNode(const rclcpp::NodeOptions& options)
    : Node("paut_driver_node", options) {
    declareParameters();
    loadParameters();

    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(100);
    canonical_pub_ = create_publisher<inspection_interfaces::msg::PautFrame>(
        "/inspection/paut/raw", sensor_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10).reliable());
    diagnostics_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&PautDriverNode::publishDiagnostics, this));

    receiver_ = std::make_unique<PautUdpReceiver>(port_);
    receiver_->setOnFrame(
        std::bind(&PautDriverNode::onFrame, this, std::placeholders::_1));
    bind_ok_ = receiver_->start();
    if (!bind_ok_) {
        RCLCPP_ERROR(get_logger(),
            "PAUT UDP bind 端口 %u 失败 (errno=%d: %s)。端口可能被占用: 例如 PAUT_ros2 的 "
            "paaut_control 节点仍在运行占用了同一端口, 或残留进程未释放。请先释放端口后重启本节点。",
            static_cast<unsigned>(port_), receiver_->lastError(),
            std::strerror(receiver_->lastError()));
    } else {
        RCLCPP_INFO(get_logger(),
            "PAUT UDP 节点就绪: 监听端口 %u, 标准输出 /inspection/paut/raw, sensor_id=%s",
            static_cast<unsigned>(port_), sensor_id_.c_str());
        RCLCPP_WARN(get_logger(),
            "PAUT 协议无硬件时戳/采样时钟: header.stamp 为主机收包完成时刻; "
            "sampling_rate/gain/sound_velocity 在标定前保持 unknown(NaN/0), "
            "未标定前不得当作后处理基线。");
    }
}

PautDriverNode::~PautDriverNode() {
    // 先 join 收帧线程, 确保不再有 onFrame 回调访问成员后, 成员才被析构。
    if (receiver_) {
        receiver_->stop();
    }
}

void PautDriverNode::declareParameters() {
    declare_parameter("port", 12345);
    declare_parameter("sensor_id", "UNASSIGNED");
    declare_parameter("calibration_id", "UNASSIGNED");
    declare_parameter("frame_id", "paut_probe_link");
    declare_parameter("sample_encoding", "signed_int32_host_endian_channel_major");
    declare_parameter("sampling_rate_hz", 0.0);
    declare_parameter("gain_db", -1.0);
    declare_parameter("sound_velocity_m_s", -1.0);
}

void PautDriverNode::loadParameters() {
    port_ = static_cast<uint16_t>(get_parameter("port").as_int());
    sensor_id_ = get_parameter("sensor_id").as_string();
    calibration_id_ = get_parameter("calibration_id").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    sample_encoding_ = get_parameter("sample_encoding").as_string();

    const double sampling_rate = get_parameter("sampling_rate_hz").as_double();
    sampling_rate_hz_ = (std::isfinite(sampling_rate) && sampling_rate > 0.0)
        ? static_cast<float>(sampling_rate) : 0.0F;
    const double gain_db = get_parameter("gain_db").as_double();
    gain_db_ = (std::isfinite(gain_db) && gain_db >= 0.0)
        ? static_cast<float>(gain_db) : std::numeric_limits<float>::quiet_NaN();
    const double sound_velocity = get_parameter("sound_velocity_m_s").as_double();
    sound_velocity_m_s_ = (std::isfinite(sound_velocity) && sound_velocity >= 0.0)
        ? static_cast<float>(sound_velocity) : std::numeric_limits<float>::quiet_NaN();
}

void PautDriverNode::onFrame(const ReceivedPautFrame& frame) {
    using inspection_interfaces::msg::PautFrame;
    PautFrame msg;
    msg.header.stamp.sec = static_cast<int32_t>(frame.host_receive_time_ns / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(frame.host_receive_time_ns % 1000000000LL);
    msg.header.frame_id = frame_id_;
    msg.sequence = frame.sequence;
    msg.sensor_id = sensor_id_;
    msg.calibration_id = calibration_id_;
    msg.channel_count = static_cast<uint32_t>(frame.channel_count);
    msg.sample_count = static_cast<uint32_t>(frame.sample_count);
    msg.sampling_rate_hz = sampling_rate_hz_;
    msg.gain_db = gain_db_;
    msg.sound_velocity_m_s = sound_velocity_m_s_;
    msg.sample_encoding = sample_encoding_;
    msg.samples.assign(frame.samples.begin(), frame.samples.end());
    last_channel_.store(frame.channel_count);
    last_sample_.store(frame.sample_count);
    canonical_pub_->publish(std::move(msg));
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

    const uint64_t parsed = receiver_ ? receiver_->parsedFrameCount() : 0;
    if (!bind_ok_) {
        status.level = DiagnosticStatus::ERROR;
        status.message = "udp bind failed; port likely in use";
    } else if (parsed == 0) {
        status.level = DiagnosticStatus::ERROR;
        status.message = "no frame received yet; is the sender pushing to the port?";
    } else {
        status.level = DiagnosticStatus::WARN;
        status.message = "host receive timestamps; calibration semantics unavailable";
    }

    const auto add_value = [&status](const std::string& key, const std::string& value) {
        KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
    };
    add_value("port", std::to_string(static_cast<unsigned>(port_)));
    add_value("timestamp_source", "host_receive_completion");
    add_value("sample_layout", "channel_major_int32");
    add_value("parsed_frame_count", std::to_string(parsed));
    add_value("garbage_datagram_count",
        receiver_ ? std::to_string(receiver_->garbageDatagramCount()) : "0");
    add_value("last_frame_rate_hz",
        receiver_ ? std::to_string(receiver_->frameRateHz()) : "0.0");
    add_value("channel_count", std::to_string(last_channel_.load()));
    add_value("sample_count", std::to_string(last_sample_.load()));
    add_value("sensor_id", sensor_id_);
    add_value("calibration_id", calibration_id_);
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
}

}  // namespace paut_driver
