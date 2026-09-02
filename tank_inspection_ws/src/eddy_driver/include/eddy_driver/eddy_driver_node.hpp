#ifndef EDDY_DRIVER_NODE_HPP_
#define EDDY_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include "eddy_driver/msg/eddy_state.hpp"
#include "eddy_driver/eddy_controller.hpp"
#include "inspection_interfaces/msg/eddy_current_frame.hpp"

namespace eddy_driver {

class EddyDriverNode : public rclcpp::Node {
public:
    using EddyState = eddy_driver::msg::EddyState;
    explicit EddyDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~EddyDriverNode() override;
private:
    void declareParameters();
    void loadParameters();
    void publishData();
    void publishCanonicalFrame(const EddyController::ReceivedAdcFrame& received);
    void publishDiagnostics();
    void querySettings();
    void cmdCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    static void computeChannelMeans(const EddyController::AdcFrame& frame, int16_t means[8]);

    EddyController driver_;
    std::string device_ = "/dev/ttyACM0";
    int baud_ = 4000000;
    double publish_rate_ = 10.0;
    double settings_query_rate_ = 1.0;
    std::string sensor_id_ = "UNASSIGNED";
    std::string calibration_id_ = "UNASSIGNED";
    std::string frame_id_ = "eddy_probe_link";
    double sampling_rate_hz_ = 0.0;
    double gain_db_ = -1.0;
    double lift_off_mm_ = -1.0;

    rclcpp::Publisher<EddyState>::SharedPtr eddy_state_pub_;
    rclcpp::Publisher<inspection_interfaces::msg::EddyCurrentFrame>::SharedPtr canonical_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr adc_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr settings_query_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace eddy_driver
#endif
