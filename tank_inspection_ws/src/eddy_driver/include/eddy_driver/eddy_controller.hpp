/**
 * @file eddy_controller.h
 * @brief 便携式激励采集装置（STM32）USB转串口控制驱动头文件
 * @note 本类全面兼容硬件 UART 与 USB 虚拟串口（如 /dev/ttyUSB0），采用线程安全双向队列设计。
 * @note 协议适配为 8通道涡流数据格式（与 huasu eddy_equipment 一致）。
 */

#ifndef EDDY_DRIVER_CONTROLLER_HPP_
#define EDDY_DRIVER_CONTROLLER_HPP_

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <queue>
#include <cstdint>

namespace eddy_driver {

class EddyController {
public:
    /**
     * @brief 构造函数：初始化控制器状态
     */
    EddyController();

    /**
     * @brief 析构函数：确保安全释放线程与物理串口句柄
     */
    ~EddyController();

    // ========================================================================
    // 协议基本常量定义
    // ========================================================================
    static constexpr size_t ADC_SAMPLES    = 20;   ///< 每帧采样点数
    static constexpr size_t ADC_CHANNELS   = 8;    ///< 物理通道数
    static constexpr size_t RX_ADC_FRAME_SIZE = 324; ///< 下行 ADC 采样数据帧大小（55 AA + 320字节数据 + 7E FE）
    static constexpr size_t TX_CMD_FRAME_SIZE = 7;   ///< 上行/反馈 控制指令帧大小（AA + CMD + 4字节参数 + 55）
    static constexpr size_t MAX_QUEUE_SIZE = 512;    ///< 有界突发缓冲，防止高频下内存积压

    // ========================================================================
    // 业务数据结构定义（8通道振幅格式，与 huasu eddy_equipment 一致）
    // ========================================================================

    /// 完整 ADC 帧：20 组采样 × 8 通道 × int16_t 振幅
    struct AdcFrame {
        int16_t channels[ADC_SAMPLES][ADC_CHANNELS];  // [sample][channel]
    };

    /// ADC 帧及其在主机完成协议解析时记录的接收元数据。
    struct ReceivedAdcFrame {
        AdcFrame frame{};
        int64_t host_receive_time_ns{0};
        uint64_t sequence{0};
    };

    /// 状态反馈数据（设备应答帧）
    struct StatusFrame {
        uint8_t cmd;      ///< 命令字
        int32_t value;    ///< 返回值
    };

    // ========================================================================
    // 命令字常量（与 huasu 一致）
    // ========================================================================
    static constexpr uint8_t CMD_SET_AMPLITUDE   = 0x01;
    static constexpr uint8_t CMD_SET_FREQUENCY   = 0x02;
    static constexpr uint8_t CMD_QUERY_AMPLITUDE = 0x11;
    static constexpr uint8_t CMD_QUERY_FREQUENCY = 0x12;
    static constexpr int32_t MIN_AMPLITUDE = 10;
    static constexpr int32_t MAX_AMPLITUDE = 450;
    static constexpr int32_t MIN_FREQUENCY_HZ = 10;
    static constexpr int32_t MAX_FREQUENCY_HZ = 420000000;

    // ========================================================================
    // 生命周期与通信管理接口
    // ========================================================================

    /**
     * @brief 打开指定的通信设备
     * @param device 串口设备路径
     * @param baud 通信波特率，针对高速 USB 传输默认推荐 4000000 (4Mbps)
     * @return true 打开并配置成功; false 初始化失败
     */
    bool open(const std::string& device, int baud = 4000000);

    /**
     * @brief 关闭设备并释放文件描述符
     */
    void close();

    /**
     * @brief 检查当前设备是否处于打开挂载状态
     */
    bool isOpen() const { return fd_ >= 0; }

    // ========================================================================
    // 多线程及异步数据交付接口
    // ========================================================================

    /**
     * @brief 启动底层高频数据接收与状态机解析线程
     */
    void startThread();

    /**
     * @brief 停止接收线程并安全回收资源
     */
    void stopThread();

    /**
     * @brief 向 STM32 下发控制或查询指令（线程安全）
     * @param cmd 命令字（CMD_SET_AMPLITUDE / CMD_SET_FREQUENCY / ...）
     * @param value 附加的 32 位控制核心数据
     * @return true 发送成功; false 发送失败或串口未打开
     */
    bool sendCommand(uint8_t cmd, int32_t value = 0);

    /**
     * @brief 向 STM32 下发控制指令（字符串版本，保留兼容）
     * @param cmd_str 指令文本描述（如 "SET_AMP", "SET_FREQ" 等）
     * @param value 附加的 32 位控制核心数据
     * @return true 发送成功; false 发送失败或串口未打开
     */
    bool sendCommand(const std::string& cmd_str, uint32_t value = 0);

    /**
     * @brief 外部算法消费接口：提取当前缓冲区中最新的一帧 ADC 数据（非阻塞、清空历史）
     * @param[out] frame 用于带出最新高频数据帧的结构体引用
     * @return true 成功提取到新数据; false 缓冲区为空
     */
    bool getLatestFrame(AdcFrame& frame);

    /**
     * @brief 按接收顺序提取一帧 ADC 数据及主机接收元数据（非阻塞）
     * @param[out] frame 帧数据、接收完成时间和驱动序号
     * @return true 成功提取到新数据; false 缓冲区为空
     */
    bool getNextFrame(ReceivedAdcFrame& frame);

    uint64_t getParsedFrameCount() const { return parsed_frame_count_.load(); }
    uint64_t getDroppedFrameCount() const { return dropped_frame_count_.load(); }
    size_t getQueueSize() const;

    /**
     * @brief 获取最新设备状态反馈（非阻塞）
     * @param[out] status 状态帧引用
     * @return true 有新的状态反馈; false 无新数据
     */
    bool getLatestStatus(StatusFrame& status);

    /**
     * @brief 获取设备最近一次确认的激励幅值
     * @return 幅值；尚未收到设置/查询应答时为 0
     */
    int32_t getCurrentAmplitude() const { return current_amplitude_.load(); }

    /**
     * @brief 获取设备最近一次确认的激励频率
     * @return 频率 [Hz]；尚未收到设置/查询应答时为 0
     */
    int32_t getCurrentFrequency() const { return current_frequency_.load(); }

private:
    /**
     * @brief 底层接收线程循环体：负责高速解析串口字节流状态机
     */
    void threadLoop();

    /**
     * @brief 内部安全压栈函数：将解析完的数据帧推入队列（限制最大长度）
     */
    void pushFrame(const AdcFrame& frame);

    /**
     * @brief 存储最新状态反馈
     */
    void setStatus(uint8_t cmd, int32_t value);

private:
    int fd_;                        ///< 物理文件描述符句柄
    std::atomic<bool> running_;     ///< 线程运行状态原子守护旗标
    std::thread thread_;            ///< 后台解析线程对象

    std::mutex tx_mtx_;             ///< 独占锁：保护串口 write 写入
    mutable std::mutex queue_mtx_;  ///< 独占锁：保护接收数据队列 data_queue_
    std::queue<ReceivedAdcFrame> data_queue_; ///< 线程间共享的数据中转 FIFO 队列
    std::atomic<uint64_t> parsed_frame_count_{0};
    std::atomic<uint64_t> dropped_frame_count_{0};

    std::mutex status_mtx_;         ///< 独占锁：保护状态数据
    StatusFrame latest_status_{};   ///< 最新状态反馈
    bool has_new_status_ = false;   ///< 是否有新状态

    std::atomic<int32_t> current_amplitude_{0}; ///< 设备确认的当前激励幅值
    std::atomic<int32_t> current_frequency_{0}; ///< 设备确认的当前激励频率 [Hz]
};

}  // namespace eddy_driver

#endif // EDDY_DRIVER_CONTROLLER_HPP_
