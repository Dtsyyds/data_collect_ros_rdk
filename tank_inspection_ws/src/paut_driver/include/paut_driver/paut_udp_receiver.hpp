/**
 * @file paut_udp_receiver.hpp
 * @brief 相控阵超声(PAUT) UDP 推流接收/解码器
 * @note 复刻 PAUT_ros2 的 udp_receiver 数据协议(自包含, 无厂商 SDK)。
 * @note wire 帧 = int32 height + int32 width + height*width 个 int32(行优先, host-endian)。
 */

#ifndef PAUT_DRIVER_UDP_RECEIVER_HPP_
#define PAUT_DRIVER_UDP_RECEIVER_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace paut_driver {

/// 一帧 PAUT 超声。按 channel-major 保存: samples[channel*sample_count + sample]。
/// channel 轴 = 扫描/波束位置(width, 实测 167); sample 轴 = 深度/时间采样(height, 实测 61)。
/// 与 PAUT_ros2 内部 image[scan][depth] 布局一致。
struct ReceivedPautFrame {
    int32_t channel_count = 0;        ///< 扫描/波束轴长度
    int32_t sample_count = 0;         ///< 深度/采样轴长度
    std::vector<int32_t> samples;     ///< channel-major, 长度 == channel_count*sample_count
    int64_t host_receive_time_ns = 0; ///< 主机收包完成时刻(system_clock, 纳秒)
    uint64_t sequence = 0;            ///< 单调递增, 主机分配
};

/// UDP 收包 + 解码: bind 后跑独立线程, 每帧经回调交付。
/// 不做节流/滤波——解码即回调(由节点线程安全地发布), 下游 DDS 层负责丢弃策略。
class PautUdpReceiver {
public:
    using FrameHandler = std::function<void(const ReceivedPautFrame&)>;

    explicit PautUdpReceiver(uint16_t port = 12345);
    ~PautUdpReceiver();

    void setOnFrame(FrameHandler handler);

    /// bind + 启动接收线程; 失败(如端口被占用)返回 false, lastError() 取 errno。
    bool start();
    /// 停止接收线程(join)并关闭 socket。
    void stop();
    bool isRunning() const { return running_.load(); }

    uint16_t port() const { return port_; }
    int lastError() const { return last_error_; }
    uint64_t parsedFrameCount() const { return parsed_.load(); }
    uint64_t garbageDatagramCount() const { return garbage_.load(); }
    int32_t lastChannelCount() const { return last_channel_.load(); }
    int32_t lastSampleCount() const { return last_sample_.load(); }
    double frameRateHz() const;

private:
    void threadLoop();

    uint16_t port_;
    int sock_ = -1;
    int last_error_ = 0;
    std::atomic<bool> running_{false};
    std::thread thread_;
    FrameHandler on_frame_;

    std::atomic<uint64_t> parsed_{0};
    std::atomic<uint64_t> garbage_{0};
    std::atomic<int32_t> last_channel_{0};
    std::atomic<int32_t> last_sample_{0};

    // 帧率 EWMA, 收帧线程写、诊断线程读。
    mutable std::mutex rate_mtx_;
    double last_frame_time_s_ = 0.0;
    double rate_hz_ = 0.0;
};

}  // namespace paut_driver

#endif  // PAUT_DRIVER_UDP_RECEIVER_HPP_
