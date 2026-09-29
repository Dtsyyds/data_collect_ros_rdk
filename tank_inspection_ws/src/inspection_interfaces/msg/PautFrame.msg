# ⚠ 本类型已过时（v0，对应 UDP :12345 的裁剪图像），**请勿用于新的采集**。
#   1) 下方第 2 行的轴语义注释**写反了**：实测 167 = 采样点(深度)、61 = 电子扫查波束数。
#   2) 字段名 `channel_count`(=167) / `sample_count`(=61) 与上述语义相反。
#   3) 只含成像窗口裁剪后的 167 点；完整 A 扫是 896 点。
#   4) 无元数据（探头/楔块/时基/编码器全无）。
#   → 新采集请用 PautFrameV2（完整 61×896 + 帧级元数据）+ PautConfig（39 字段采集配置）。
#
#   本类型**冻结保留**：bags/paut_real（958 帧真机数据）等 5 个 bag 依赖它的类型哈希，
#   改动任何字段都会让那些数据无法反序列化（契约 C6.4）。
#
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
