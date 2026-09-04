# Raw phased-array ultrasonic (PAUT) frame. 保真 int32，不做截断/归一化/增益。
# 轴语义：channel 轴 = scan/波束/扫描位置（width，实测 167）；sample 轴 = depth/时间采样（height，实测 61）。
#   语义来自 UDP 发送端，wire 上仅带 height/width；其余声学细节由 calibration_id 对应文档定义。
# samples 使用 channel-major 布局：samples[channel * sample_count + sample]。
# RAW mcap 不可变；header.stamp = 主机收包完成时刻（设备无硬件时戳）。
std_msgs/Header header
uint64 sequence
string sensor_id
string calibration_id
uint32 channel_count
uint32 sample_count
float32 sampling_rate_hz
float32 gain_db
float32 sound_velocity_m_s
string sample_encoding
int32[] samples
