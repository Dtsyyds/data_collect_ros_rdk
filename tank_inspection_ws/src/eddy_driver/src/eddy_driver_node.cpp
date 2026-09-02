#include "eddy_driver/eddy_driver_node.hpp"
#include <memory>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace eddy_driver {

EddyDriverNode::EddyDriverNode(const rclcpp::NodeOptions& options)
    : Node("eddy_driver_node", options)
{
    declareParameters();
    loadParameters();

    if (!driver_.open(device_, baud_)) {
        RCLCPP_ERROR(get_logger(), "Eddy 串口 %s 打开失败", device_.c_str());
    } else {
        RCLCPP_INFO(get_logger(), "Eddy 串口 %s 打开, 波特率 %d", device_.c_str(), baud_);
        driver_.startThread();
        // 获取设备上电后已有的配置，使 /eddy/state 从硬件应答中恢复当前值。
        driver_.sendCommand(EddyController::CMD_QUERY_AMPLITUDE);
        driver_.sendCommand(EddyController::CMD_QUERY_FREQUENCY);
    }

    eddy_state_pub_ = create_publisher<EddyState>("/eddy/state", 10);
    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(50);
    canonical_pub_ = create_publisher<inspection_interfaces::msg::EddyCurrentFrame>(
        "/inspection/eddy_current/raw", sensor_qos);
    adc_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("~/adc_data", rclcpp::SystemDefaultsQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10).reliable());
    cmd_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>("/eddy/cmd", 10,
        std::bind(&EddyDriverNode::cmdCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(period, std::bind(&EddyDriverNode::publishData, this));
    auto query_period = std::chrono::duration<double>(1.0 / settings_query_rate_);
    settings_query_timer_ = create_wall_timer(query_period, std::bind(&EddyDriverNode::querySettings, this));
    diagnostics_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&EddyDriverNode::publishDiagnostics, this));

    RCLCPP_INFO(get_logger(),
        "Eddy 节点就绪. 标准输出 /inspection/eddy_current/raw，传感器 ID=%s，参数查询 %.2f Hz",
        sensor_id_.c_str(), settings_query_rate_);
    RCLCPP_WARN(get_logger(),
        "设备协议不含采样时钟和 Q 分量：header.stamp 使用主机帧接收完成时间，"
        "signal_q 写入 NaN，quality_score 保持 0；完成硬件语义确认前不得标记为已标定数据");
}

EddyDriverNode::~EddyDriverNode() { driver_.stopThread(); driver_.close(); }

void EddyDriverNode::declareParameters() {
    declare_parameter("device", "/dev/ttyACM0");
    declare_parameter("baud", 4000000);
    declare_parameter("publish_rate", 10.0);
    declare_parameter("settings_query_rate", 1.0);
    declare_parameter("sensor_id", "UNASSIGNED");
    declare_parameter("calibration_id", "UNASSIGNED");
    declare_parameter("frame_id", "eddy_probe_link");
    declare_parameter("sampling_rate_hz", 0.0);
    declare_parameter("gain_db", -1.0);
    declare_parameter("lift_off_mm", -1.0);
}
void EddyDriverNode::loadParameters() {
    device_ = get_parameter("device").as_string();
    baud_ = get_parameter("baud").as_int();
    publish_rate_ = get_parameter("publish_rate").as_double();
    settings_query_rate_ = get_parameter("settings_query_rate").as_double();
    sensor_id_ = get_parameter("sensor_id").as_string();
    calibration_id_ = get_parameter("calibration_id").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    sampling_rate_hz_ = get_parameter("sampling_rate_hz").as_double();
    gain_db_ = get_parameter("gain_db").as_double();
    lift_off_mm_ = get_parameter("lift_off_mm").as_double();
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "publish_rate 必须大于 0，改用默认值 10.0 Hz");
        publish_rate_ = 10.0;
    }
    if (!std::isfinite(settings_query_rate_) || settings_query_rate_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "settings_query_rate 必须大于 0，改用默认值 1.0 Hz");
        settings_query_rate_ = 1.0;
    }
}

void EddyDriverNode::querySettings() {
    if (!driver_.isOpen()) return;
    if (!driver_.sendCommand(EddyController::CMD_QUERY_AMPLITUDE) ||
        !driver_.sendCommand(EddyController::CMD_QUERY_FREQUENCY)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "查询当前幅值/频率失败");
    }
}

void EddyDriverNode::computeChannelMeans(const EddyController::AdcFrame& frame, int16_t means[8]) {
    int32_t sum[8] = {};
    for (size_t s = 0; s < EddyController::ADC_SAMPLES; ++s)
        for (size_t c = 0; c < EddyController::ADC_CHANNELS; ++c)
            sum[c] += frame.channels[s][c];
    for (size_t c = 0; c < EddyController::ADC_CHANNELS; ++c)
        means[c] = static_cast<int16_t>(sum[c] / static_cast<int32_t>(EddyController::ADC_SAMPLES));
}

void EddyDriverNode::publishData() {
    if (!driver_.isOpen()) return;
    EddyController::ReceivedAdcFrame received;
    EddyController::ReceivedAdcFrame latest;
    bool received_any = false;
    while (driver_.getNextFrame(received)) {
        publishCanonicalFrame(received);
        latest = received;
        received_any = true;
    }
    if (!received_any) return;

    auto raw = std_msgs::msg::UInt8MultiArray();
    raw.data.resize(sizeof(latest.frame));
    std::memcpy(raw.data.data(), &latest.frame, sizeof(latest.frame));
    adc_pub_->publish(raw);

    int16_t means[8]; computeChannelMeans(latest.frame, means);
    auto state = EddyState();
    for (size_t i = 0; i < 8; ++i) state.channels[i] = means[i];
    state.current_amplitude = driver_.getCurrentAmplitude();
    state.current_frequency = driver_.getCurrentFrequency();

    EddyController::StatusFrame status;
    if (driver_.getLatestStatus(status)) {
        state.has_status = true; state.status_cmd = status.cmd; state.status_value = status.value;
    }
    eddy_state_pub_->publish(state);
}

void EddyDriverNode::publishCanonicalFrame(
    const EddyController::ReceivedAdcFrame& received)
{
    using inspection_interfaces::msg::EddyCurrentFrame;
    EddyCurrentFrame message;
    message.header.stamp.sec = static_cast<int32_t>(
        received.host_receive_time_ns / 1000000000LL);
    message.header.stamp.nanosec = static_cast<uint32_t>(
        received.host_receive_time_ns % 1000000000LL);
    message.header.frame_id = frame_id_;
    message.sequence = received.sequence;
    message.sensor_id = sensor_id_;
    message.calibration_id = calibration_id_;
    message.channel_count = static_cast<uint32_t>(EddyController::ADC_CHANNELS);
    message.sample_count = static_cast<uint32_t>(EddyController::ADC_SAMPLES);
    message.sampling_rate_hz = static_cast<float>(sampling_rate_hz_);
    message.excitation_frequency_hz = static_cast<float>(driver_.getCurrentFrequency());
    message.gain_db = gain_db_ >= 0.0
        ? static_cast<float>(gain_db_) : std::numeric_limits<float>::quiet_NaN();
    message.lift_off_mm = lift_off_mm_ >= 0.0
        ? static_cast<float>(lift_off_mm_) : std::numeric_limits<float>::quiet_NaN();
    message.quality_score = 0.0F;

    const size_t value_count = EddyController::ADC_CHANNELS * EddyController::ADC_SAMPLES;
    message.signal_i.reserve(value_count);
    for (size_t channel = 0; channel < EddyController::ADC_CHANNELS; ++channel) {
        for (size_t sample = 0; sample < EddyController::ADC_SAMPLES; ++sample) {
            message.signal_i.push_back(
                static_cast<float>(received.frame.channels[sample][channel]));
        }
    }
    message.signal_q.assign(value_count, std::numeric_limits<float>::quiet_NaN());
    canonical_pub_->publish(std::move(message));
}

void EddyDriverNode::publishDiagnostics() {
    using diagnostic_msgs::msg::DiagnosticArray;
    using diagnostic_msgs::msg::DiagnosticStatus;
    using diagnostic_msgs::msg::KeyValue;

    DiagnosticArray array;
    array.header.stamp = now();
    DiagnosticStatus status;
    status.name = "eddy_driver/acquisition";
    status.hardware_id = sensor_id_;
    if (!driver_.isOpen()) {
        status.level = DiagnosticStatus::ERROR;
        status.message = "serial device is not open";
    } else {
        status.level = DiagnosticStatus::WARN;
        status.message = "host receive timestamps; Q component and calibration quality unavailable";
    }
    const auto add_value = [&status](const std::string& key, const std::string& value) {
        KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
    };
    add_value("device", device_);
    add_value("timestamp_source", "host_receive_completion");
    add_value("signal_layout", "channel_major_i_only; signal_q=NaN");
    add_value("parsed_frame_count", std::to_string(driver_.getParsedFrameCount()));
    add_value("dropped_frame_count", std::to_string(driver_.getDroppedFrameCount()));
    add_value("queue_length", std::to_string(driver_.getQueueSize()));
    add_value("queue_capacity", std::to_string(EddyController::MAX_QUEUE_SIZE));
    add_value("sampling_rate_hz", std::to_string(sampling_rate_hz_));
    add_value("current_amplitude", std::to_string(driver_.getCurrentAmplitude()));
    add_value("current_frequency_hz", std::to_string(driver_.getCurrentFrequency()));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
}

void EddyDriverNode::cmdCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (!msg || msg->data.empty()) return;
    int32_t ci = msg->data[0];
    if (ci < 0 || ci > 255) { RCLCPP_WARN(get_logger(), "无效 cmd=%d", ci); return; }
    uint8_t cmd = static_cast<uint8_t>(ci);
    int32_t value = (msg->data.size() >= 2) ? msg->data[1] : 0;

    switch (cmd) {
        case EddyController::CMD_SET_AMPLITUDE:
            if (value < EddyController::MIN_AMPLITUDE || value > EddyController::MAX_AMPLITUDE) {
                RCLCPP_WARN(get_logger(), "激励幅值超出允许范围 [%d, %d]: %d",
                    EddyController::MIN_AMPLITUDE, EddyController::MAX_AMPLITUDE, value);
                return;
            }
            break;
        case EddyController::CMD_SET_FREQUENCY:
            if (value < EddyController::MIN_FREQUENCY_HZ
                    || value > EddyController::MAX_FREQUENCY_HZ) {
                RCLCPP_WARN(get_logger(), "激励频率超出允许范围 [%d, %d] Hz: %d",
                    EddyController::MIN_FREQUENCY_HZ,
                    EddyController::MAX_FREQUENCY_HZ, value);
                return;
            }
            break;
        case EddyController::CMD_QUERY_AMPLITUDE:
        case EddyController::CMD_QUERY_FREQUENCY:
            break;
        default: RCLCPP_WARN(get_logger(), "不支持 cmd=0x%02X", cmd); return;
    }
    if (!driver_.sendCommand(cmd, value))
        RCLCPP_ERROR(get_logger(), "cmd=0x%02X 下发失败", cmd);
}

}  // namespace eddy_driver
