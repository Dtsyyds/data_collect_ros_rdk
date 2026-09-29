/**
 * @file paut_v1_receiver.cpp
 * @brief PAUT UDP v1 接收器实现。
 *
 * socket 参数与 v0（paut_udp_receiver.cpp）保持一致，仅新增 SO_REUSEADDR ——
 * 便于与 Python 参考实现 scripts/recv_paut_12346.py 同时监听做对照调试。
 *
 * ⚠ SO_REUSEADDR 在 Linux 上对 UDP **广播**通常允许两个 socket 同时收到同一数据报，
 *   但对**单播**无效（第二个 bind 会 EADDRINUSE，除非用 SO_REUSEPORT）。
 *   因此"能双收"不是保证，只是便于广播场景下调试。
 */

#include "paut_driver/paut_v1_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <mutex>
#include <utility>

namespace paut_driver {

namespace {

/// 接收缓冲。UDP 数据报上限 65507 B，取 64 KiB 足够；
/// 若真出现 n == buffer.size() 说明被截断，计为 garbage。
constexpr size_t kReceiveBufferBytes = 64 * 1024;

/// 收包超时，便于 running_ 置位后及时退出线程。
constexpr int kRecvTimeoutUsec = 100000;  // 100 ms

/// 帧率 EWMA 平滑系数
constexpr double kRateAlpha = 0.2;
/// 超过该秒数无帧则重置 EWMA
constexpr double kRateResetTimeoutS = 5.0;

int64_t systemTimeNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

}  // namespace

PautV1Receiver::PautV1Receiver(uint16_t port) : port_(port) {}

PautV1Receiver::~PautV1Receiver() { stop(); }

bool PautV1Receiver::start() {
    if (running_.load()) return true;

    sock_ = static_cast<int>(::socket(AF_INET, SOCK_DGRAM, 0));
    if (sock_ < 0) {
        last_error_ = errno;
        return false;
    }

    // 允许接收广播（设备端默认发往 255.255.255.255）
    int broadcast = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    // 便于与 Python 参考实现同时监听做对照调试（广播场景有效，单播不保证）
    int reuse = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // 放大内核接收缓冲，缓解高频突发丢包（上限受 rmem_max 约束，失败可忽略）
    int rcvbuf = 4 * 1024 * 1024;
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port_);
    if (::bind(sock_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
        last_error_ = errno;
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = kRecvTimeoutUsec;
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    buffer_.assign(kReceiveBufferBytes, 0);
    running_.store(true);
    thread_ = std::thread([this]() { threadLoop(); });
    return true;
}

void PautV1Receiver::stop() {
    running_.store(false);
    if (thread_.joinable()) {
        thread_.join();
    }
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

void PautV1Receiver::threadLoop() {
    while (running_.load()) {
        sockaddr_in client{};
        socklen_t client_len = sizeof(client);
        const ssize_t n = ::recvfrom(sock_, buffer_.data(), buffer_.size(), 0,
                                     reinterpret_cast<sockaddr*>(&client), &client_len);
        if (n < 0) {
            if (!running_.load()) break;
            // 超时（EAGAIN/EWOULDBLOCK）或被信号打断属正常；其余错误短暂跳过。
            continue;
        }
        if (n == 0) continue;
        if (static_cast<size_t>(n) >= buffer_.size()) {
            garbage_.fetch_add(1);  // 数据报被缓冲截断（理论上不会发生，UDP 上限 65507）
            continue;
        }
        handleDatagram(buffer_.data(), static_cast<size_t>(n));
    }
}

void PautV1Receiver::handleDatagram(const uint8_t* buf, size_t len) {
    v1::Envelope env;
    if (v1::parseEnvelope(buf, len, env) != v1::ParseStatus::kOk) {
        garbage_.fetch_add(1);
        return;
    }

    if (v1::checkCrc(buf, len) != v1::ParseStatus::kOk) {
        crc_fail_.fetch_add(1);
        return;
    }

    switch (env.packet_type) {
        case v1::kTypeConfig: {
            v1::Config cfg;
            if (v1::parseConfig(buf, len, env, cfg) != v1::ParseStatus::kOk) {
                garbage_.fetch_add(1);
                return;
            }
            configs_.fetch_add(1);
            if (on_config_) on_config_(cfg);
            break;
        }
        case v1::kTypeFrame: {
            v1::FrameMeta meta;
            const uint8_t* payload = nullptr;
            size_t payload_len = 0;
            const v1::ParseStatus st =
                v1::parseFrame(buf, len, env, meta, &payload, &payload_len);
            // kBadDimension（维度非法）与 kBadLength（长度不自洽）都算畸形包
            if (st != v1::ParseStatus::kOk) {
                garbage_.fetch_add(1);
                return;
            }
            // 本版不做分片重组：设备端当前 54,728 B < 60000 上限，永不分片。
            // 若将来出现分片（换大探头 / FMC），这里会丢弃并计数，不会静默错拼。
            if (meta.frag_count != 1) {
                frag_dropped_.fetch_add(1);
                return;
            }

            frames_.fetch_add(1);
            last_beam_.store(meta.beam_count);
            last_sample_.store(meta.sample_count);

            // 帧率 EWMA（仅收包线程写）
            {
                const double now_s = static_cast<double>(systemTimeNs()) / 1e9;
                std::lock_guard<std::mutex> lock(rate_mtx_);
                if (last_frame_time_s_ <= 0.0 ||
                    now_s - last_frame_time_s_ > kRateResetTimeoutS) {
                    rate_hz_ = 0.0;
                } else {
                    const double dt = now_s - last_frame_time_s_;
                    if (dt > 0.0) {
                        rate_hz_ = kRateAlpha * (1.0 / dt) + (1.0 - kRateAlpha) * rate_hz_;
                    }
                }
                last_frame_time_s_ = now_s;
            }

            if (on_frame_) on_frame_(meta, payload, payload_len);
            break;
        }
        case v1::kTypeHeartbeat: {
            uint64_t counter = 0;
            if (v1::parseHeartbeat(buf, len, env, counter) != v1::ParseStatus::kOk) {
                garbage_.fetch_add(1);
                return;
            }
            heartbeats_.fetch_add(1);
            last_heartbeat_ns_.store(systemTimeNs());
            if (on_heartbeat_) on_heartbeat_(counter);
            break;
        }
        default:
            // 未知包类型：可能是设备端将来新增的。计 garbage 而不是 CRC 失败，
            // 便于区分"版本不匹配"与"链路损坏"。
            garbage_.fetch_add(1);
            break;
    }
}

double PautV1Receiver::frameRateHz() const {
    std::lock_guard<std::mutex> lock(rate_mtx_);
    return rate_hz_;
}

}  // namespace paut_driver
