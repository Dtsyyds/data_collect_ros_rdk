/**
 * @file paut_driver_node.hpp
 * @brief PAUT UDP v1(:12346) → ROS 2 节点。
 *
 * 发布三个话题（契约 C7.2 靠 config_seq 配对）:
 *   /inspection/paut/raw_v2   PautFrameV2  完整 61x896 原始数据 + 帧级元数据
 *   /inspection/paut/config   PautConfig   40 字段采集配置（transient_local 闩锁）
 *   /diagnostics              DiagnosticArray
 *
 * 旧的 v0 链路（:12345 → /inspection/paut/raw → PautFrame）已被本节点停用。
 *   `paut_udp_receiver.*` 与 `PautFrame.msg` 仍保留在仓库中（bags/paut_real 等
 *   5 个 bag 依赖旧消息的类型哈希），但不再被本节点使用。
 *
 * 时间语义（契约 C1）: 设备无硬件时戳, device_timestamp_ns 恒 0,
 *   header.stamp = **主机收包时刻**, timestamp_source = TS_HOST_RECEIVE。
 */

#ifndef PAUT_DRIVER_NODE_HPP_
#define PAUT_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "inspection_interfaces/msg/paut_config.hpp"
#include "inspection_interfaces/msg/paut_frame_v2.hpp"
#include "paut_driver/paut_v1_protocol.hpp"
#include "paut_driver/paut_v1_receiver.hpp"

namespace paut_driver {

class PautDriverNode : public rclcpp::Node {
public:
    explicit PautDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PautDriverNode() override;

private:
    void declareParameters();
    void loadParameters();

    /// CONFIG 包到达。内容指纹变化时才对外发布（设备每 2s 重发同一份）。
    void onConfig(const v1::Config& cfg);
    /// FRAME 包到达。payload 仅在回调期间有效，必须在此完成发布。
    void onFrame(const v1::FrameMeta& meta, const uint8_t* payload, size_t payload_len);

    void publishDiagnostics();

    std::unique_ptr<PautV1Receiver> receiver_;
    bool bind_ok_ = false;

    uint16_t port_ = 12346;
    std::string sensor_id_ = "UNASSIGNED";
    std::string calibration_id_ = "UNASSIGNED";
    std::string frame_id_ = "paut_probe_link";

    // ---- 最近一份配置（收包线程写、诊断线程读）----
    // 用 contentFingerprint 而非 config_seq 判变化：设备每 2s 重发同一 seq，
    // 靠 seq 分不清"重发"与"变更"。
    std::atomic<uint32_t> config_fingerprint_{0};
    std::atomic<bool> config_seen_{false};
    std::atomic<uint8_t> scan_mode_{255};   ///< 255 = 尚未收到配置
    std::atomic<uint32_t> config_seq_{0};

    // ---- 诊断计数（收包线程写、诊断线程读）----
    std::atomic<uint64_t> frames_published_{0};
    std::atomic<uint64_t> frames_without_config_{0};   ///< 帧早于首份 CONFIG（启动竞态）
    std::atomic<uint64_t> non_linear_dropped_{0};      ///< scan_mode != 线性扫，本版不支持
    std::atomic<int32_t> last_beam_{0};
    std::atomic<int32_t> last_sample_{0};

    rclcpp::Publisher<inspection_interfaces::msg::PautFrameV2>::SharedPtr frame_pub_;
    rclcpp::Publisher<inspection_interfaces::msg::PautConfig>::SharedPtr config_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace paut_driver

#endif  // PAUT_DRIVER_NODE_HPP_
