/**
 * @file paut_v1_receiver.hpp
 * @brief PAUT UDP 线格式 v1（:12346）的接收器：socket + 收包线程 + 三类包回调。
 *
 * @note 本层只做**线级**判断（长度/魔数/CRC/维度/分片），不做语义判断
 *       （如 scan_mode 是否线扫）——那属于节点的职责，因为只有节点持有配置。
 * @note 与 paut_udp_receiver.hpp（v0, :12345）并存：那个是旧格式，
 *       代码**冻结保留**（bags/paut_real 等 5 个 bag 依赖其产出的消息类型）。
 *
 * 线程模型：start() 起一个独立线程 recvfrom 循环，回调在该线程内**同步**调用。
 *   因此回调必须线程安全且**不得阻塞**——这是从 v0 继承的约定。
 */

#ifndef PAUT_DRIVER_V1_RECEIVER_HPP_
#define PAUT_DRIVER_V1_RECEIVER_HPP_

#include "paut_driver/paut_v1_protocol.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace paut_driver {

/// 一帧到达。payload 指向**收包缓冲内部**，仅在回调执行期间有效，需要留存必须自行拷贝。
using V1FrameHandler =
    std::function<void(const v1::FrameMeta& meta, const uint8_t* payload, size_t payload_len)>;
using V1ConfigHandler = std::function<void(const v1::Config& cfg)>;
using V1HeartbeatHandler = std::function<void(uint64_t counter)>;

/// UDP 收包 + v1 解码。
class PautV1Receiver {
public:
    explicit PautV1Receiver(uint16_t port = 12346);
    ~PautV1Receiver();

    void setOnFrame(V1FrameHandler handler) { on_frame_ = std::move(handler); }
    void setOnConfig(V1ConfigHandler handler) { on_config_ = std::move(handler); }
    void setOnHeartbeat(V1HeartbeatHandler handler) { on_heartbeat_ = std::move(handler); }

    /// bind + 启动接收线程。失败（如端口被占用）返回 false，lastError() 取 errno。
    bool start();
    /// 停止接收线程（join）并关闭 socket。
    void stop();
    bool isRunning() const { return running_.load(); }

    uint16_t port() const { return port_; }
    int lastError() const { return last_error_; }

    // ---- 统计（诊断用，均可在任意线程读）----
    uint64_t frameCount() const { return frames_.load(); }
    uint64_t configCount() const { return configs_.load(); }
    uint64_t heartbeatCount() const { return heartbeats_.load(); }
    /// 畸形/无法解析的数据报数（长度不足、魔数不符、长度不自洽、维度非法）
    uint64_t garbageCount() const { return garbage_.load(); }
    /// CRC32 校验失败数
    uint64_t crcFailCount() const { return crc_fail_.load(); }
    /// 因 frag_count != 1 被丢弃的帧数（本版不做分片重组）
    uint64_t fragmentedDroppedCount() const { return frag_dropped_.load(); }
    int32_t lastBeamCount() const { return last_beam_.load(); }
    int32_t lastSampleCount() const { return last_sample_.load(); }
    /// 最近一次收到心跳的时刻（system_clock 纳秒）；0 表示从未收到
    int64_t lastHeartbeatTimeNs() const { return last_heartbeat_ns_.load(); }
    /// 帧率 EWMA（α=0.2，超过 5s 无帧重置为 0）
    double frameRateHz() const;

private:
    void threadLoop();
    void handleDatagram(const uint8_t* buf, size_t len);

    uint16_t port_;
    int sock_ = -1;
    int last_error_ = 0;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::vector<uint8_t> buffer_;

    V1FrameHandler on_frame_;
    V1ConfigHandler on_config_;
    V1HeartbeatHandler on_heartbeat_;

    std::atomic<uint64_t> frames_{0};
    std::atomic<uint64_t> configs_{0};
    std::atomic<uint64_t> heartbeats_{0};
    std::atomic<uint64_t> garbage_{0};
    std::atomic<uint64_t> crc_fail_{0};
    std::atomic<uint64_t> frag_dropped_{0};
    std::atomic<int32_t> last_beam_{0};
    std::atomic<int32_t> last_sample_{0};
    std::atomic<int64_t> last_heartbeat_ns_{0};

    // 帧率 EWMA：收包线程写、诊断线程读，故加锁
    mutable std::mutex rate_mtx_;
    double last_frame_time_s_ = 0.0;
    double rate_hz_ = 0.0;
};

}  // namespace paut_driver

#endif  // PAUT_DRIVER_V1_RECEIVER_HPP_
