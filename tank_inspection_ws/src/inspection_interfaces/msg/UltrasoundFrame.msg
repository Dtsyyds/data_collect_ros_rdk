# Samples use channel-major layout: samples[channel * sample_count + sample].
std_msgs/Header header
uint64 sequence
string sensor_id
string calibration_id
uint32 channel_count
uint32 sample_count
float32 sampling_rate_hz
float32 gain_db
float32 sound_velocity_m_s
float32 coupling_score
float32 contact_force_n
string sample_encoding
int16[] samples
