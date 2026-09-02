/**
 * @file eddy_controller.cpp
 * @brief 便携式激励采集装置（STM32）USB转串口控制驱动实现源文件
 * @note 协议适配为 8通道涡流数据格式（与 huasu eddy_equipment 一致）。
 *       ADC帧: 55 AA + 320字节(20样本×8通道×int16) + 7E FE
 *       命令帧: AA + CMD + value[4] + 55
 */

#include "eddy_driver/eddy_controller.hpp"

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cerrno>
#include <chrono>
#include <numeric>

namespace eddy_driver {

// 帧头尾常量
static constexpr uint8_t ADC_HEAD0 = 0x55;
static constexpr uint8_t ADC_HEAD1 = 0xAA;
static constexpr uint8_t ADC_TAIL0 = 0x7E;
static constexpr uint8_t ADC_TAIL1 = 0xFE;
static constexpr uint8_t CMD_HEAD  = 0xAA;
static constexpr uint8_t CMD_TAIL  = 0x55;
static constexpr size_t  ADC_PAYLOAD_SIZE = EddyController::ADC_SAMPLES * EddyController::ADC_CHANNELS * 2;  // 320

EddyController::EddyController() : fd_(-1), running_(false) {}

EddyController::~EddyController() {
    stopThread();
    close();
}

// ========== 生命周期管理 ==========

bool EddyController::open(const std::string& device, int baud) {
    if (isOpen()) {
        printf("[EDDY] 设备串口已处于打开状态\n");
        return true;
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        fprintf(stderr, "[EDDY_ERROR] 无法打开物理设备 %s: %s\n", device.c_str(), strerror(errno));
        return false;
    }

    int flags = fcntl(fd_, F_GETFL);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    struct termios options;
    tcgetattr(fd_, &options);

    speed_t baud_const = (baud == 4000000) ? B4000000 : B115200;
    cfsetispeed(&options, baud_const);
    cfsetospeed(&options, baud_const);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= (CREAD | CLOCAL);

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNBRK);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;

    tcsetattr(fd_, TCSANOW, &options);
    tcflush(fd_, TCIOFLUSH);

    printf("[EDDY] 设备 %s 打开成功, 当前适配波特率 %d (8通道协议)\n", device.c_str(), baud);
    return true;
}

void EddyController::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        printf("[EDDY] 物理串口释放完毕\n");
    }
}

// ========== 线程生命周期控制 ==========

void EddyController::startThread() {
    if (running_) return;
    running_ = true;
    thread_ = std::thread(&EddyController::threadLoop, this);
}

void EddyController::stopThread() {
    if (!running_) return;
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

// ========== 核心状态机数据流解析（8通道协议） ==========

void EddyController::threadLoop() {
    std::vector<uint8_t> raw_buf;
    uint8_t buffer[4096];

    while (running_) {
        int bytes_read = read(fd_, buffer, sizeof(buffer));

        if (bytes_read > 0) {
            raw_buf.insert(raw_buf.end(), buffer, buffer + bytes_read);

            while (!raw_buf.empty()) {

                // ============================================================
                // 分支 1：ADC 采样帧解析 (324 字节) — 8通道格式
                // 帧格式: 55 AA | 320字节数据 | 7E FE
                // 数据: 20样本 × 8通道 × int16 (小端)
                // ============================================================
                if (raw_buf[0] == ADC_HEAD0) {
                    if (raw_buf.size() >= RX_ADC_FRAME_SIZE) {
                        if (raw_buf[1] == ADC_HEAD1 &&
                            raw_buf[RX_ADC_FRAME_SIZE - 2] == ADC_TAIL0 &&
                            raw_buf[RX_ADC_FRAME_SIZE - 1] == ADC_TAIL1)
                        {
                            AdcFrame current_frame;
                            const uint8_t* payload = &raw_buf[2];  // 跳过 55 AA
                            for (size_t s = 0; s < ADC_SAMPLES; ++s) {
                                for (size_t c = 0; c < ADC_CHANNELS; ++c) {
                                    const size_t byte_idx = (s * ADC_CHANNELS + c) * 2;
                                    current_frame.channels[s][c] = static_cast<int16_t>(
                                        static_cast<uint16_t>(payload[byte_idx]) |
                                        (static_cast<uint16_t>(payload[byte_idx + 1]) << 8));
                                }
                            }
                            pushFrame(current_frame);
                            raw_buf.erase(raw_buf.begin(), raw_buf.begin() + RX_ADC_FRAME_SIZE);
                        } else {
                            raw_buf.erase(raw_buf.begin());
                        }
                    } else {
                        break;
                    }
                }
                // ============================================================
                // 分支 2：命令应答/状态反馈帧解析 (7 字节)
                // 帧格式: AA | CMD | value[4] LE | 55
                // ============================================================
                else if (raw_buf[0] == CMD_HEAD) {
                    if (raw_buf.size() >= TX_CMD_FRAME_SIZE) {
                        if (raw_buf[TX_CMD_FRAME_SIZE - 1] == CMD_TAIL) {
                            uint8_t  cmd   = raw_buf[1];
                            int32_t  value = static_cast<int32_t>(
                                static_cast<uint32_t>(raw_buf[2]) |
                                (static_cast<uint32_t>(raw_buf[3]) << 8) |
                                (static_cast<uint32_t>(raw_buf[4]) << 16) |
                                (static_cast<uint32_t>(raw_buf[5]) << 24));

                            printf("[EDDY] STM32硬件应答 -> 命令字: 0x%02X | 核心负载数据: %d\n", cmd, value);
                            setStatus(cmd, value);

                            raw_buf.erase(raw_buf.begin(), raw_buf.begin() + TX_CMD_FRAME_SIZE);
                        } else {
                            raw_buf.erase(raw_buf.begin());
                        }
                    } else {
                        break;
                    }
                }
                // ============================================================
                // 分支 3：抛弃未知垃圾字节
                // ============================================================
                else {
                    raw_buf.erase(raw_buf.begin());
                }
            }
        } else {
            usleep(100);
        }
    }
}

// ========== 线程安全的数据交付 ==========

void EddyController::pushFrame(const AdcFrame& frame) {
    const auto receive_time = std::chrono::system_clock::now().time_since_epoch();
    ReceivedAdcFrame received;
    received.frame = frame;
    received.host_receive_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(receive_time).count();
    received.sequence = parsed_frame_count_.fetch_add(1);

    std::lock_guard<std::mutex> lock(queue_mtx_);
    data_queue_.push(received);
    if (data_queue_.size() > MAX_QUEUE_SIZE) {
        data_queue_.pop();
        dropped_frame_count_.fetch_add(1);
    }
}

bool EddyController::getLatestFrame(AdcFrame& frame) {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    if (data_queue_.empty()) return false;

    frame = data_queue_.back().frame;

    std::queue<ReceivedAdcFrame> empty_queue;
    std::swap(data_queue_, empty_queue);

    return true;
}

bool EddyController::getNextFrame(ReceivedAdcFrame& frame) {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    if (data_queue_.empty()) return false;
    frame = data_queue_.front();
    data_queue_.pop();
    return true;
}

size_t EddyController::getQueueSize() const {
    std::lock_guard<std::mutex> lock(queue_mtx_);
    return data_queue_.size();
}

// ========== 状态反馈 ==========

void EddyController::setStatus(uint8_t cmd, int32_t value) {
    // 高频的其他状态帧可能在 ROS 定时发布前覆盖 latest_status_，因此在
    // 串口解析线程中直接保存设置/查询应答，确保当前激励参数不会丢失。
    if (cmd == CMD_SET_AMPLITUDE || cmd == CMD_QUERY_AMPLITUDE) {
        current_amplitude_.store(value);
    } else if (cmd == CMD_SET_FREQUENCY || cmd == CMD_QUERY_FREQUENCY) {
        current_frequency_.store(value);
    }

    std::lock_guard<std::mutex> lock(status_mtx_);
    latest_status_.cmd = cmd;
    latest_status_.value = value;
    has_new_status_ = true;
}

bool EddyController::getLatestStatus(StatusFrame& status) {
    std::lock_guard<std::mutex> lock(status_mtx_);
    if (!has_new_status_) return false;
    status = latest_status_;
    has_new_status_ = false;
    return true;
}

// ========== 命令下发（uint8_t cmd 版本） ==========

bool EddyController::sendCommand(uint8_t cmd, int32_t value) {
    if (!isOpen()) return false;

    uint8_t tx_packet[TX_CMD_FRAME_SIZE];
    tx_packet[0] = CMD_HEAD;
    tx_packet[1] = cmd;
    tx_packet[2] = static_cast<uint8_t>(value & 0xFF);
    tx_packet[3] = static_cast<uint8_t>((value >> 8) & 0xFF);
    tx_packet[4] = static_cast<uint8_t>((value >> 16) & 0xFF);
    tx_packet[5] = static_cast<uint8_t>((value >> 24) & 0xFF);
    tx_packet[6] = CMD_TAIL;

    std::lock_guard<std::mutex> lock(tx_mtx_);
    int n = write(fd_, tx_packet, TX_CMD_FRAME_SIZE);
    if (n < 0) {
        fprintf(stderr, "[EDDY_ERROR] 指令下发失败 (cmd=0x%02X): %s\n", cmd, strerror(errno));
        return false;
    }
    return true;
}

// ========== 命令下发（字符串版本，保留兼容） ==========

bool EddyController::sendCommand(const std::string& cmd_str, uint32_t value) {
    uint8_t cmd_word = 0x00;
    uint32_t core_data = value;

    if (cmd_str == "SET_AMP") {
        cmd_word = CMD_SET_AMPLITUDE;
        if (core_data < static_cast<uint32_t>(MIN_AMPLITUDE)) {
            core_data = MIN_AMPLITUDE;
        }
        if (core_data > static_cast<uint32_t>(MAX_AMPLITUDE)) {
            core_data = MAX_AMPLITUDE;
        }
    }
    else if (cmd_str == "SET_FREQ") {
        cmd_word = CMD_SET_FREQUENCY;
        if (core_data < static_cast<uint32_t>(MIN_FREQUENCY_HZ)) {
            core_data = MIN_FREQUENCY_HZ;
        }
        if (core_data > static_cast<uint32_t>(MAX_FREQUENCY_HZ)) {
            core_data = MAX_FREQUENCY_HZ;
        }
    }
    else if (cmd_str == "QUERY_AMP")  { cmd_word = CMD_QUERY_AMPLITUDE; core_data = 0; }
    else if (cmd_str == "QUERY_FREQ") { cmd_word = CMD_QUERY_FREQUENCY; core_data = 0; }
    else {
        fprintf(stderr, "[EDDY_WARN] 试图下发未知指令描述符: %s\n", cmd_str.c_str());
        return false;
    }

    return sendCommand(cmd_word, static_cast<int32_t>(core_data));
}

}  // namespace eddy_driver
