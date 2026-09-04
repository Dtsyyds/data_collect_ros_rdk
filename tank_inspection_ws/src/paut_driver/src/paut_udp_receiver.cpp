/**
 * @file paut_udp_receiver.cpp
 * @brief PAUT UDP 接收/解码实现。复刻 PAUT_ros2 的 udp_receiver 逻辑并做健壮化校验。
 */

#include "paut_driver/paut_udp_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

namespace paut_driver {

namespace {
/// 接收缓冲: 1 MiB, 远大于实测单帧 ~40.7KB。
constexpr std::size_t kReceiveBufferBytes = 1u << 20;
/// 维度上界, 防溢出/恶意帧(height/width 不得越界)。
constexpr int32_t kMaxDimension = 4096;
/// 帧率 EWMA 平滑系数。
constexpr double kRateAlpha = 0.2;
/// 超过该秒数无帧则重置 EWMA。
constexpr double kRateResetTimeoutS = 5.0;

int64_t systemTimeNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
}  // namespace

PautUdpReceiver::PautUdpReceiver(uint16_t port) : port_(port) {}

PautUdpReceiver::~PautUdpReceiver() { stop(); }

void PautUdpReceiver::setOnFrame(FrameHandler handler) { on_frame_ = std::move(handler); }

bool PautUdpReceiver::start() {
    if (running_.load()) {
        return true;
    }
    sock_ = static_cast<int>(::socket(AF_INET, SOCK_DGRAM, 0));
    if (sock_ < 0) {
        last_error_ = errno;
        return false;
    }

    // 允许广播(镜像 PAUT_ros2 发送端行为)。
    int broadcast = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    // 放大内核接收缓冲, 缓解高频突发丢包(上限受 rmem_max 约束, 失败可忽略)。
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

    // 100ms 收包超时, 便于 running_ 置位后及时退出线程。
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    running_.store(true);
    thread_ = std::thread([this]() { threadLoop(); });
    return true;
}

void PautUdpReceiver::stop() {
    running_.store(false);
    if (thread_.joinable()) {
        thread_.join();
    }
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

void PautUdpReceiver::threadLoop() {
    std::vector<uint8_t> buffer(kReceiveBufferBytes);
    while (running_.load()) {
        sockaddr_in client{};
        socklen_t client_len = sizeof(client);
        const ssize_t n = ::recvfrom(sock_, buffer.data(), buffer.size(), 0,
            reinterpret_cast<sockaddr*>(&client), &client_len);
        if (n < 0) {
            if (!running_.load()) {
                break;
            }
            // 超时/被信号打断属正常; 其余错误短暂跳过。
            continue;
        }
        if (n == static_cast<ssize_t>(buffer.size())) {
            garbage_.fetch_add(1);  // 数据报被缓冲截断
            continue;
        }
        if (n < 8) {
            garbage_.fetch_add(1);  // 缺 8 字节头
            continue;
        }

        int32_t height = 0;
        int32_t width = 0;
        // host-endian 假设: 与 PAUT_ros2 相同(纯 memcpy)。若未来 sender 大小端异构需改。
        std::memcpy(&height, buffer.data(), 4);
        std::memcpy(&width, buffer.data() + 4, 4);
        if (height <= 0 || width <= 0 || height > kMaxDimension || width > kMaxDimension) {
            garbage_.fetch_add(1);
            continue;
        }
        const int64_t expected = 8LL + static_cast<int64_t>(height) * width * 4;
        if (n != expected) {
            garbage_.fetch_add(1);  // 分片/截断/畸形数据报
            continue;
        }

        ReceivedPautFrame frame;
        frame.channel_count = width;
        frame.sample_count = height;
        frame.samples.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
        // wire 行优先: value at buffer[8 + row*width + col], row∈[0,height) 深度/sample,
        //   col∈[0,width) 扫描/channel。目标 channel-major: samples[col*height + row]。
        const uint8_t* body = buffer.data() + 8;
        for (int32_t row = 0; row < height; ++row) {
            for (int32_t col = 0; col < width; ++col) {
                int32_t value = 0;
                std::memcpy(&value, body + (static_cast<std::size_t>(row) * width + col) * 4, 4);
                frame.samples[static_cast<std::size_t>(col) * height + row] = value;
            }
        }
        frame.host_receive_time_ns = systemTimeNs();
        frame.sequence = parsed_.fetch_add(1);
        last_channel_.store(width);
        last_sample_.store(height);

        // 帧率 EWMA(仅收帧线程写)。
        {
            const double now_s = static_cast<double>(frame.host_receive_time_ns) / 1e9;
            std::lock_guard<std::mutex> lock(rate_mtx_);
            if (last_frame_time_s_ <= 0.0 || now_s - last_frame_time_s_ > kRateResetTimeoutS) {
                rate_hz_ = 0.0;
            } else {
                const double dt = now_s - last_frame_time_s_;
                if (dt > 0.0) {
                    const double instant = 1.0 / dt;
                    rate_hz_ = kRateAlpha * instant + (1.0 - kRateAlpha) * rate_hz_;
                }
            }
            last_frame_time_s_ = now_s;
        }

        if (on_frame_) {
            on_frame_(frame);
        }
    }
}

double PautUdpReceiver::frameRateHz() const {
    std::lock_guard<std::mutex> lock(rate_mtx_);
    return rate_hz_;
}

}  // namespace paut_driver
