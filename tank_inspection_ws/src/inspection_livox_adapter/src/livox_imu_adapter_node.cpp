#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "inspection_livox_adapter/imu_conversion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace inspection_livox_adapter
{

class LivoxImuAdapterNode final : public rclcpp::Node
{
public:
  LivoxImuAdapterNode()
  : Node("livox_imu_adapter")
  {
    acceleration_scale_ = declare_parameter(
      "acceleration_scale", kStandardGravityMps2);
    if (!std::isfinite(acceleration_scale_) || acceleration_scale_ <= 0.0) {
      throw std::invalid_argument("acceleration_scale must be finite and positive");
    }

    auto input_qos = rclcpp::SensorDataQoS();
    input_qos.keep_last(200);
    output_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/livox/imu", rclcpp::QoS(200).reliable().durability_volatile());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable().durability_volatile());
    input_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu_raw", input_qos,
      std::bind(&LivoxImuAdapterNode::imu_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LivoxImuAdapterNode::publish_diagnostics, this));

    RCLCPP_INFO(
      get_logger(),
      "Livox IMU adapter ready: /livox/imu_raw [g] -> /livox/imu [m/s^2], scale=%.5f",
      acceleration_scale_);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr input)
  {
    received_count_.fetch_add(1);
    sensor_msgs::msg::Imu output;
    if (!convert_imu_to_si(*input, output, acceleration_scale_)) {
      rejected_count_.fetch_add(1);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected Livox IMU message with zero timestamp or non-finite measurement");
      return;
    }

    const int64_t stamp_ns =
      static_cast<int64_t>(output.header.stamp.sec) * 1000000000LL +
      static_cast<int64_t>(output.header.stamp.nanosec);
    const int64_t previous_ns = last_stamp_ns_.exchange(stamp_ns);
    if (previous_ns > 0 && stamp_ns < previous_ns) {
      non_monotonic_count_.fetch_add(1);
    }
    output_publisher_->publish(std::move(output));
    published_count_.fetch_add(1);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "inspection_livox_adapter/imu_units";
    status.hardware_id = "MID-360";
    status.level = rejected_count_.load() == 0 && non_monotonic_count_.load() == 0 ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK ?
      "converting Livox acceleration from g to m/s^2" :
      "IMU rejection or timestamp regression detected";

    const auto add_value = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
      };
    add_value("input_topic", "/livox/imu_raw");
    add_value("output_topic", "/livox/imu");
    add_value("input_acceleration_unit", "g");
    add_value("output_acceleration_unit", "m/s^2");
    add_value("acceleration_scale", std::to_string(acceleration_scale_));
    add_value("received_count", std::to_string(received_count_.load()));
    add_value("published_count", std::to_string(published_count_.load()));
    add_value("rejected_count", std::to_string(rejected_count_.load()));
    add_value("non_monotonic_count", std::to_string(non_monotonic_count_.load()));
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(std::move(array));
  }

  double acceleration_scale_{kStandardGravityMps2};
  std::atomic<uint64_t> received_count_{0};
  std::atomic<uint64_t> published_count_{0};
  std::atomic<uint64_t> rejected_count_{0};
  std::atomic<uint64_t> non_monotonic_count_{0};
  std::atomic<int64_t> last_stamp_ns_{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr output_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr input_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace inspection_livox_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_livox_adapter::LivoxImuAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
