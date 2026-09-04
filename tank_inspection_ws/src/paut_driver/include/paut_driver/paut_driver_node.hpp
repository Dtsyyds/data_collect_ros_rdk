/**
 * @file paut_driver_node.hpp
 * @brief PAUT UDP 推流 → /inspection/paut/raw 节点。解码即发布, 不做滤波/增益。
 */

#ifndef PAUT_DRIVER_NODE_HPP_
#define PAUT_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "inspection_interfaces/msg/paut_frame.hpp"
#include "paut_driver/paut_udp_receiver.hpp"

namespace paut_driver {

class PautDriverNode : public rclcpp::Node {
public:
    explicit PautDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PautDriverNode() override;

private:
    void declareParameters();
    void loadParameters();
    void onFrame(const ReceivedPautFrame& frame);
    void publishDiagnostics();

    std::unique_ptr<PautUdpReceiver> receiver_;
    bool bind_ok_ = false;

    uint16_t port_ = 12345;
    std::string sensor_id_ = "UNASSIGNED";
    std::string calibration_id_ = "UNASSIGNED";
    std::string frame_id_ = "paut_probe_link";
    std::string sample_encoding_ = "signed_int32_host_endian_channel_major";
    float sampling_rate_hz_ = 0.0F;   // <=0 表示未知
    float gain_db_ = 0.0F;            // NaN 表示未知
    float sound_velocity_m_s_ = 0.0F; // NaN 表示未知

    std::atomic<int32_t> last_channel_{0};
    std::atomic<int32_t> last_sample_{0};

    rclcpp::Publisher<inspection_interfaces::msg::PautFrame>::SharedPtr canonical_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace paut_driver

#endif  // PAUT_DRIVER_NODE_HPP_
