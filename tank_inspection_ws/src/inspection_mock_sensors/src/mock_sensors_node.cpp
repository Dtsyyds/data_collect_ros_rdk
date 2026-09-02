#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "inspection_interfaces/msg/eddy_current_frame.hpp"
#include "inspection_interfaces/msg/probe_state.hpp"
#include "inspection_interfaces/msg/robot_state.hpp"
#include "inspection_interfaces/msg/ultrasound_frame.hpp"
#include "inspection_mock_sensors/mission_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace inspection_mock_sensors
{

constexpr double kPi = 3.14159265358979323846;

class MockSensorsNode final : public rclcpp::Node
{
public:
  MockSensorsNode() : Node("inspection_mock_sensors")
  {
    width_ = positive_int("depth_width", 848);
    height_ = positive_int("depth_height", 480);
    depth_rate_hz_ = positive_double("depth_rate_hz", 15.0);
    lidar_rate_hz_ = positive_double("lidar_rate_hz", 10.0);
    lidar_points_ = positive_int("lidar_point_count", 2400);
    imu_rate_hz_ = positive_double("imu_rate_hz", 200.0);
    odom_rate_hz_ = positive_double("odometry_rate_hz", 50.0);
    robot_rate_hz_ = positive_double("robot_state_rate_hz", 20.0);
    ultrasound_rate_hz_ = positive_double("ultrasound_rate_hz", 50.0);
    ultrasound_channels_ = positive_int("ultrasound_channel_count", 2);
    ultrasound_samples_ = positive_int("ultrasound_sample_count", 2048);
    eddy_rate_hz_ = positive_double("eddy_current_rate_hz", 25.0);
    eddy_channels_ = positive_int("eddy_current_channel_count", 2);
    eddy_samples_ = positive_int("eddy_current_sample_count", 512);
    probe_rate_hz_ = positive_double("probe_state_rate_hz", 50.0);
    linear_speed_m_s_ = declare_parameter<double>("linear_speed_m_s", 0.05);
    vertical_speed_m_s_ = declare_parameter<double>("vertical_speed_m_s", 0.1);
    tank_radius_m_ = positive_double("tank_radius_m", 15.0);
    tank_height_m_ = positive_double("tank_height_m", 18.0);
    tank_start_angle_rad_ = declare_parameter<double>("tank_start_angle_rad", 0.0);
    start_height_m_ = declare_parameter<double>("start_height_m", 1.0);
    robot_standoff_m_ = declare_parameter<double>("robot_standoff_m", 0.3);
    const auto mission_trajectory_path = declare_parameter<std::string>(
      "mission_trajectory_path", "");
    mission_speed_m_s_ = positive_double("mission_speed_m_s", 2.0);
    mission_dwell_s_ = positive_double("mission_dwell_s", 0.5);
    mission_loop_ = declare_parameter<bool>("mission_loop", true);
    if (!std::isfinite(linear_speed_m_s_) || !std::isfinite(vertical_speed_m_s_) ||
      !std::isfinite(tank_start_angle_rad_) || !std::isfinite(start_height_m_) ||
      !std::isfinite(robot_standoff_m_) || start_height_m_ < 0.0 || robot_standoff_m_ < 0.0)
    {
      throw std::invalid_argument("mock tank trajectory parameters must be finite and non-negative");
    }
    if (!mission_trajectory_path.empty()) {
      mission_trajectory_ = std::make_unique<MissionTrajectory>(MissionTrajectory::load(
          mission_trajectory_path, tank_radius_m_, tank_height_m_, tank_start_angle_rad_,
          mission_speed_m_s_, mission_dwell_s_));
    }
    const auto seed = static_cast<uint32_t>(declare_parameter<int64_t>("random_seed", 42));

    std::mt19937 generator(seed);
    std::uniform_int_distribution<int> noise(-2, 2);
    depth_noise_.resize(4096);
    for (auto & value : depth_noise_) {
      value = static_cast<int16_t>(noise(generator));
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto reliable_sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/camera/camera/depth/image_rect_raw", reliable_sensor_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/depth/camera_info", rclcpp::QoS(1).reliable());
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", sensor_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/livox/imu", sensor_qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", sensor_qos);
    robot_pub_ = create_publisher<inspection_interfaces::msg::RobotState>(
      "/robot/state", sensor_qos);
    ultrasound_pub_ = create_publisher<inspection_interfaces::msg::UltrasoundFrame>(
      "/inspection/ultrasound/raw", sensor_qos);
    eddy_pub_ = create_publisher<inspection_interfaces::msg::EddyCurrentFrame>(
      "/inspection/eddy_current/raw", sensor_qos);
    probe_pub_ = create_publisher<inspection_interfaces::msg::ProbeState>(
      "/inspection/probe_state", sensor_qos);

    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    publish_static_transforms();

    timer_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    image_timer_ = make_timer(depth_rate_hz_, [this]() {publish_depth();});
    lidar_timer_ = make_timer(lidar_rate_hz_, [this]() {publish_lidar();});
    imu_timer_ = make_timer(imu_rate_hz_, [this]() {publish_imu();});
    odom_timer_ = make_timer(odom_rate_hz_, [this]() {publish_odometry();});
    robot_timer_ = make_timer(robot_rate_hz_, [this]() {publish_robot_state();});
    ultrasound_timer_ = make_timer(
      ultrasound_rate_hz_, [this]() {publish_ultrasound();});
    eddy_timer_ = make_timer(eddy_rate_hz_, [this]() {publish_eddy_current();});
    probe_timer_ = make_timer(probe_rate_hz_, [this]() {publish_probe_state();});

    std::string trajectory_description = ", fallback=helix";
    if (mission_trajectory_) {
      trajectory_description =
        ", mission=" + mission_trajectory_->mission_id() +
        ", points=" + std::to_string(mission_trajectory_->size()) +
        ", breaks=" + std::to_string(mission_trajectory_->break_count()) +
        ", duration_s=" + std::to_string(mission_trajectory_->duration_s());
    }
    RCLCPP_INFO(
      get_logger(),
      "Deterministic mock sensors started (seed=%u, depth=%dx%d@%.1f Hz%s)",
      seed, width_, height_, depth_rate_hz_, trajectory_description.c_str());
  }

private:
  struct TankPose
  {
    double angle;
    double height;
    double x;
    double y;
    double yaw;
  };

  TankPose tank_pose(double time_s) const
  {
    double s_m = linear_speed_m_s_ * time_s;
    double height = start_height_m_ + vertical_speed_m_s_ * time_s;
    if (mission_trajectory_) {
      const auto trajectory_pose = mission_trajectory_->pose(time_s, mission_loop_);
      s_m = trajectory_pose.s_m;
      height = trajectory_pose.z_m;
    }
    const double angle = tank_start_angle_rad_ + s_m / tank_radius_m_;
    const double pose_radius = tank_radius_m_ + robot_standoff_m_;
    return TankPose{
      angle,
      height,
      pose_radius * std::cos(angle),
      pose_radius * std::sin(angle),
      angle + 0.5 * kPi};
  }

  int positive_int(const std::string & name, int default_value)
  {
    const int value = declare_parameter<int>(name, default_value);
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return value;
  }

  double positive_double(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!(value > 0.0)) {
      throw std::invalid_argument(name + " must be positive");
    }
    return value;
  }

  template<typename CallbackT>
  rclcpp::TimerBase::SharedPtr make_timer(double frequency_hz, CallbackT && callback)
  {
    const auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    return create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::forward<CallbackT>(callback), timer_callback_group_);
  }

  void publish_depth()
  {
    sensor_msgs::msg::Image image;
    image.header.stamp = now();
    image.header.frame_id = "camera_depth_optical_frame";
    image.height = static_cast<uint32_t>(height_);
    image.width = static_cast<uint32_t>(width_);
    image.encoding = "16UC1";
    image.is_bigendian = false;
    image.step = static_cast<uint32_t>(width_ * sizeof(uint16_t));
    image.data.resize(static_cast<size_t>(image.step) * image.height);

    const double phase = 0.025 * static_cast<double>(depth_sequence_);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const double dx_weld = (x - width_ * 0.50) / std::max(1.0, width_ * 0.025);
        const double dx_dent = (x - width_ * 0.72) / std::max(1.0, width_ * 0.055);
        const double dy_dent = (y - height_ * 0.58) / std::max(1.0, height_ * 0.08);
        const double plane_mm = 1050.0 + 0.10 * x + 0.06 * y;
        const double weld_mm = 35.0 * std::exp(-0.5 * dx_weld * dx_weld);
        const double dent_mm = 24.0 * std::exp(-0.5 * (dx_dent * dx_dent + dy_dent * dy_dent));
        const int noise = depth_noise_[(static_cast<size_t>(y) * width_ + x + depth_sequence_) %
          depth_noise_.size()];
        const auto depth_mm = static_cast<uint16_t>(std::clamp(
          plane_mm - weld_mm + dent_mm + noise + 1.5 * std::sin(phase), 1.0, 65535.0));
        const size_t offset = (static_cast<size_t>(y) * width_ + x) * sizeof(uint16_t);
        std::memcpy(image.data.data() + offset, &depth_mm, sizeof(depth_mm));
      }
    }

    sensor_msgs::msg::CameraInfo info;
    info.header = image.header;
    info.height = image.height;
    info.width = image.width;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    const double fx = 0.90 * width_;
    const double fy = 0.90 * width_;
    const double cx = (width_ - 1) * 0.5;
    const double cy = (height_ - 1) * 0.5;
    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    image_pub_->publish(std::move(image));
    camera_info_pub_->publish(std::move(info));
    ++depth_sequence_;
  }

  void publish_lidar()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = "livox_frame";
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(lidar_points_);
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time_offset", 1, sensor_msgs::msg::PointField::UINT32);
    modifier.resize(static_cast<size_t>(lidar_points_));

    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity_it(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<uint32_t> offset_it(cloud, "time_offset");
    const double time_s = lidar_sequence_ / lidar_rate_hz_;
    const auto pose = tank_pose(time_s);
    const double tangent_x = -std::sin(pose.angle);
    const double tangent_y = std::cos(pose.angle);
    const double inward_x = -std::cos(pose.angle);
    const double inward_y = -std::sin(pose.angle);
    const double lidar_origin_x = pose.x + 0.05 * tangent_x;
    const double lidar_origin_y = pose.y + 0.05 * tangent_y;
    const double lidar_origin_z = pose.height + 0.22;
    for (int i = 0; i < lidar_points_; ++i, ++x_it, ++y_it, ++z_it, ++intensity_it, ++offset_it) {
      const double fraction = static_cast<double>(i) / lidar_points_;
      const double angle = pose.angle - 0.04 + 0.08 * fraction;
      const double vertical = -1.2 + 2.4 * ((i * 37) % lidar_points_) / lidar_points_;
      const bool weld_point = (i % 47) < 4;
      const double radius = tank_radius_m_ + (weld_point ? 0.018 : 0.0) +
        0.002 * std::sin(i * 0.17);
      const double global_x = radius * std::cos(angle);
      const double global_y = radius * std::sin(angle);
      const double delta_x = global_x - lidar_origin_x;
      const double delta_y = global_y - lidar_origin_y;
      *x_it = static_cast<float>(delta_x * tangent_x + delta_y * tangent_y);
      *y_it = static_cast<float>(delta_x * inward_x + delta_y * inward_y);
      *z_it = static_cast<float>(pose.height + vertical - lidar_origin_z);
      *intensity_it = static_cast<float>(weld_point ? 180.0 : 70.0 + 20.0 * std::sin(i * 0.1));
      *offset_it = static_cast<uint32_t>(fraction * (1.0e9 / lidar_rate_hz_));
    }
    lidar_pub_->publish(std::move(cloud));
    ++lidar_sequence_;
  }

  void publish_imu()
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = "livox_imu_frame";
    const double t = imu_sequence_ / imu_rate_hz_;
    const double yaw = 0.01 * std::sin(0.2 * t);
    msg.orientation.z = std::sin(yaw * 0.5);
    msg.orientation.w = std::cos(yaw * 0.5);
    msg.angular_velocity.z = 0.002 * std::cos(0.2 * t);
    msg.linear_acceleration.x = 0.01 * std::sin(0.7 * t);
    msg.linear_acceleration.y = 0.01 * std::cos(0.5 * t);
    msg.linear_acceleration.z = 9.80665;
    msg.orientation_covariance[0] = 1.0e-4;
    msg.orientation_covariance[4] = 1.0e-4;
    msg.orientation_covariance[8] = 1.0e-4;
    imu_pub_->publish(std::move(msg));
    ++imu_sequence_;
  }

  void publish_odometry()
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "tank_course";
    msg.child_frame_id = "base_link";
    const double t = odom_sequence_ / odom_rate_hz_;
    const auto pose = tank_pose(t);
    msg.pose.pose.position.x = pose.x;
    msg.pose.pose.position.y = pose.y;
    msg.pose.pose.position.z = pose.height;
    msg.pose.pose.orientation.z = std::sin(pose.yaw * 0.5);
    msg.pose.pose.orientation.w = std::cos(pose.yaw * 0.5);
    msg.twist.twist.linear.x = mission_trajectory_ ? mission_speed_m_s_ : linear_speed_m_s_;
    msg.twist.twist.linear.z = mission_trajectory_ ? 0.0 : vertical_speed_m_s_;
    msg.twist.twist.angular.z =
      (mission_trajectory_ ? mission_speed_m_s_ : linear_speed_m_s_) / tank_radius_m_;
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg.header;
    transform.child_frame_id = msg.child_frame_id;
    transform.transform.translation.x = msg.pose.pose.position.x;
    transform.transform.translation.y = msg.pose.pose.position.y;
    transform.transform.translation.z = msg.pose.pose.position.z;
    transform.transform.rotation = msg.pose.pose.orientation;
    odom_pub_->publish(std::move(msg));
    tf_broadcaster_->sendTransform(transform);
    ++odom_sequence_;
  }

  void publish_robot_state()
  {
    inspection_interfaces::msg::RobotState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    const double t = robot_sequence_ / robot_rate_hz_;
    msg.linear_speed_m_s = static_cast<float>(linear_speed_m_s_);
    msg.angular_speed_rad_s = static_cast<float>(0.0015 * std::cos(0.1 * t));
    msg.battery_voltage = static_cast<float>(48.0 - 0.002 * t);
    msg.adsorption_current_a = static_cast<float>(4.5 + 0.1 * std::sin(0.3 * t));
    msg.slip_ratio = 0.01F;
    msg.adsorption_valid = true;
    msg.scan_enabled = true;
    msg.state_flags = 0;
    robot_pub_->publish(std::move(msg));
    ++robot_sequence_;
  }

  void publish_ultrasound()
  {
    inspection_interfaces::msg::UltrasoundFrame msg;
    msg.header.stamp = now();
    msg.header.frame_id = "ultrasound_frame";
    msg.sequence = ultrasound_sequence_;
    msg.sensor_id = "mock_ultrasound";
    msg.calibration_id = "mock_ut_cal_v1";
    msg.channel_count = static_cast<uint32_t>(ultrasound_channels_);
    msg.sample_count = static_cast<uint32_t>(ultrasound_samples_);
    msg.sampling_rate_hz = 50.0e6F;
    msg.gain_db = 24.0F;
    msg.sound_velocity_m_s = 5900.0F;
    msg.coupling_score = 0.96F;
    msg.contact_force_n = 12.0F;
    msg.sample_encoding = "signed_int16_le_channel_major";
    msg.samples.resize(static_cast<size_t>(ultrasound_channels_) * ultrasound_samples_);

    const double defect_shift = 8.0 * std::sin(ultrasound_sequence_ * 0.015);
    for (int channel = 0; channel < ultrasound_channels_; ++channel) {
      for (int sample = 0; sample < ultrasound_samples_; ++sample) {
        const double front = std::exp(-0.5 * std::pow((sample - 120.0 - 3.0 * channel) / 5.0, 2));
        const double defect = std::exp(-0.5 * std::pow(
          (sample - (820.0 + defect_shift + 10.0 * channel)) / 10.0, 2));
        const double back = std::exp(-0.5 * std::pow((sample - 1650.0 - 6.0 * channel) / 13.0, 2));
        const double baseline = 65.0 * std::sin(0.025 * sample + 0.2 * channel);
        const double value = baseline + 14000.0 * front + 5200.0 * defect + 10500.0 * back;
        msg.samples[static_cast<size_t>(channel) * ultrasound_samples_ + sample] =
          static_cast<int16_t>(std::clamp(value, -32768.0, 32767.0));
      }
    }
    ultrasound_pub_->publish(std::move(msg));
    ++ultrasound_sequence_;
  }

  void publish_eddy_current()
  {
    inspection_interfaces::msg::EddyCurrentFrame msg;
    msg.header.stamp = now();
    msg.header.frame_id = "eddy_current_frame";
    msg.sequence = eddy_sequence_;
    msg.sensor_id = "mock_eddy_current";
    msg.calibration_id = "mock_ec_cal_v1";
    msg.channel_count = static_cast<uint32_t>(eddy_channels_);
    msg.sample_count = static_cast<uint32_t>(eddy_samples_);
    msg.sampling_rate_hz = 200000.0F;
    msg.excitation_frequency_hz = 100000.0F;
    msg.gain_db = 18.0F;
    msg.lift_off_mm = 0.35F;
    msg.quality_score = 0.94F;
    const size_t count = static_cast<size_t>(eddy_channels_) * eddy_samples_;
    msg.signal_i.resize(count);
    msg.signal_q.resize(count);
    for (int channel = 0; channel < eddy_channels_; ++channel) {
      for (int sample = 0; sample < eddy_samples_; ++sample) {
        const size_t index = static_cast<size_t>(channel) * eddy_samples_ + sample;
        const double phase = 2.0 * kPi * sample / 64.0 + 0.15 * channel;
        const double anomaly = std::exp(-0.5 * std::pow((sample - 280.0) / 18.0, 2));
        msg.signal_i[index] = static_cast<float>(std::cos(phase) + 0.22 * anomaly);
        msg.signal_q[index] = static_cast<float>(std::sin(phase) - 0.15 * anomaly);
      }
    }
    eddy_pub_->publish(std::move(msg));
    ++eddy_sequence_;
  }

  void publish_probe_state()
  {
    inspection_interfaces::msg::ProbeState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "probe_frame";
    const double t = probe_sequence_ / probe_rate_hz_;
    msg.contact_force_n = static_cast<float>(12.0 + 0.2 * std::sin(0.4 * t));
    msg.lift_off_mm = static_cast<float>(0.35 + 0.02 * std::sin(0.3 * t));
    msg.temperature_c = static_cast<float>(24.0 + 0.1 * std::sin(0.05 * t));
    msg.coupling_score = 0.96F;
    msg.contact_valid = true;
    msg.quality_flags = 0;
    probe_pub_->publish(std::move(msg));
    ++probe_sequence_;
  }

  void publish_static_transforms()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    const auto stamp = now();
    auto add_transform = [&transforms, &stamp](
      const std::string & child, double x, double y, double z,
      double qx, double qy, double qz, double qw)
      {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "base_link";
        tf.child_frame_id = child;
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = z;
        tf.transform.rotation.x = qx;
        tf.transform.rotation.y = qy;
        tf.transform.rotation.z = qz;
        tf.transform.rotation.w = qw;
        transforms.push_back(tf);
      };
    add_transform("camera_depth_optical_frame", 0.20, 0.0, 0.12, -0.5, 0.5, -0.5, 0.5);
    add_transform("livox_frame", 0.05, 0.0, 0.22, 0.0, 0.0, 0.0, 1.0);
    add_transform("livox_imu_frame", 0.05, 0.0, 0.22, 0.0, 0.0, 0.0, 1.0);
    add_transform("ultrasound_frame", 0.32, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0);
    add_transform("eddy_current_frame", 0.30, 0.05, -0.08, 0.0, 0.0, 0.0, 1.0);
    add_transform("probe_frame", 0.31, 0.0, -0.08, 0.0, 0.0, 0.0, 1.0);
    static_tf_broadcaster_->sendTransform(transforms);
  }

  int width_{0};
  int height_{0};
  int lidar_points_{0};
  int ultrasound_channels_{0};
  int ultrasound_samples_{0};
  int eddy_channels_{0};
  int eddy_samples_{0};
  double depth_rate_hz_{0.0};
  double lidar_rate_hz_{0.0};
  double imu_rate_hz_{0.0};
  double odom_rate_hz_{0.0};
  double robot_rate_hz_{0.0};
  double ultrasound_rate_hz_{0.0};
  double eddy_rate_hz_{0.0};
  double probe_rate_hz_{0.0};
  double linear_speed_m_s_{0.0};
  double vertical_speed_m_s_{0.0};
  double tank_radius_m_{0.0};
  double tank_height_m_{0.0};
  double tank_start_angle_rad_{0.0};
  double start_height_m_{0.0};
  double robot_standoff_m_{0.0};
  double mission_speed_m_s_{0.0};
  double mission_dwell_s_{0.0};
  bool mission_loop_{true};
  std::unique_ptr<MissionTrajectory> mission_trajectory_;
  std::vector<int16_t> depth_noise_;
  uint64_t depth_sequence_{0};
  uint64_t lidar_sequence_{0};
  uint64_t imu_sequence_{0};
  uint64_t odom_sequence_{0};
  uint64_t robot_sequence_{0};
  uint64_t ultrasound_sequence_{0};
  uint64_t eddy_sequence_{0};
  uint64_t probe_sequence_{0};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<inspection_interfaces::msg::RobotState>::SharedPtr robot_pub_;
  rclcpp::Publisher<inspection_interfaces::msg::UltrasoundFrame>::SharedPtr ultrasound_pub_;
  rclcpp::Publisher<inspection_interfaces::msg::EddyCurrentFrame>::SharedPtr eddy_pub_;
  rclcpp::Publisher<inspection_interfaces::msg::ProbeState>::SharedPtr probe_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr image_timer_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr robot_timer_;
  rclcpp::TimerBase::SharedPtr ultrasound_timer_;
  rclcpp::TimerBase::SharedPtr eddy_timer_;
  rclcpp::TimerBase::SharedPtr probe_timer_;
};

}  // namespace inspection_mock_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<inspection_mock_sensors::MockSensorsNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
