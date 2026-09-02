#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "inspection_interfaces/msg/eddy_current_frame.hpp"
#include "inspection_interfaces/msg/fusion_index.hpp"
#include "inspection_interfaces/msg/probe_state.hpp"
#include "inspection_interfaces/msg/robot_state.hpp"
#include "inspection_interfaces/msg/ultrasound_frame.hpp"
#include "inspection_sync/sync_math.hpp"
#include "inspection_sync/timestamped_ring_buffer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace inspection_sync
{

using ImageBuffer = TimestampedRingBuffer<sensor_msgs::msg::Image>;
using LidarBuffer = TimestampedRingBuffer<sensor_msgs::msg::PointCloud2>;
using ImuBuffer = TimestampedRingBuffer<sensor_msgs::msg::Imu>;
using OdomBuffer = TimestampedRingBuffer<nav_msgs::msg::Odometry>;
using UltrasoundBuffer = TimestampedRingBuffer<inspection_interfaces::msg::UltrasoundFrame>;
using EddyBuffer = TimestampedRingBuffer<inspection_interfaces::msg::EddyCurrentFrame>;
using RobotBuffer = TimestampedRingBuffer<inspection_interfaces::msg::RobotState>;
using ProbeBuffer = TimestampedRingBuffer<inspection_interfaces::msg::ProbeState>;

class InspectionSyncNode final : public rclcpp::Node
{
public:
  InspectionSyncNode() : Node("inspection_sync")
  {
    camera_tolerance_ns_ = milliseconds("camera_tolerance_ms", 50.0);
    lidar_tolerance_ns_ = milliseconds("lidar_tolerance_ms", 100.0);
    pose_max_gap_ns_ = milliseconds("pose_max_gap_ms", 30.0);
    imu_before_ns_ = milliseconds("imu_window_before_ms", 20.0);
    imu_after_ns_ = milliseconds("imu_window_after_ms", 20.0);
    const int64_t late_margin_ns = milliseconds("late_message_margin_ms", 150.0);
    process_delay_ms_ = positive_double("fusion_process_delay_ms", 120.0);
    pending_max_count_ = static_cast<size_t>(positive_int("pending_max_count", 200));
    anchor_decimation_ = static_cast<uint64_t>(positive_int("anchor_decimation", 1));
    software_clock_match_enable_ = declare_parameter<bool>(
      "software_clock_match_enable", false);
    software_clock_adaptive_enable_ = declare_parameter<bool>(
      "software_clock_adaptive_enable", false);
    camera_clock_offset_ns_.store(
      signed_milliseconds("camera_clock_offset_ms", 0.0));
    livox_clock_offset_ns_.store(
      signed_milliseconds("livox_clock_offset_ms", 0.0));
    clock_offset_alpha_ = positive_double("clock_offset_alpha", 0.25);
    if (clock_offset_alpha_ > 1.0) {
      throw std::invalid_argument("clock_offset_alpha must not exceed 1.0");
    }
    clock_offset_max_ns_ = milliseconds("clock_offset_max_ms", 20.0);
    clock_offset_update_gate_ns_ = milliseconds("clock_offset_update_gate_ms", 45.0);
    clock_offset_window_matches_ = static_cast<uint64_t>(
      positive_int("clock_offset_window_matches", 50));
    anchor_topic_ = declare_parameter<std::string>(
      "anchor_topic", "/inspection/ultrasound/raw");
    asset_id_ = declare_parameter<std::string>("asset_id", "mock_tank");
    course_id_ = declare_parameter<std::string>("course_id", "course_00");
    plate_id_ = declare_parameter<std::string>("plate_id", "plate_00");
    weld_id_ = declare_parameter<std::string>("weld_id", "weld_00");
    asset_coordinate_valid_ = declare_parameter<bool>("asset_coordinate_valid", false);
    asset_coordinate_mode_ = declare_parameter<std::string>(
      "asset_coordinate_mode", "unwrapped_pose");
    asset_radius_m_ = declare_parameter<double>("asset_radius_m", 1.0);
    asset_start_angle_rad_ = declare_parameter<double>("asset_start_angle_rad", 0.0);
    asset_pose_standoff_m_ = declare_parameter<double>("asset_pose_standoff_m", 0.0);
    if (asset_coordinate_mode_ != "unwrapped_pose" && asset_coordinate_mode_ != "tank_xyz") {
      throw std::invalid_argument(
              "asset_coordinate_mode must be unwrapped_pose or tank_xyz");
    }
    if (asset_coordinate_mode_ == "tank_xyz" &&
      (!(asset_radius_m_ > 0.0) || !std::isfinite(asset_radius_m_) ||
      !std::isfinite(asset_start_angle_rad_) || !std::isfinite(asset_pose_standoff_m_)))
    {
      throw std::invalid_argument("tank_xyz asset geometry parameters must be finite and valid");
    }
    const bool camera_reliable_qos = declare_parameter<bool>("camera_reliable_qos", false);

    camera_buffer_ = make_buffer<ImageBuffer>("camera", 2.0, 100, late_margin_ns);
    lidar_buffer_ = make_buffer<LidarBuffer>("lidar", 2.0, 40, late_margin_ns);
    imu_buffer_ = make_buffer<ImuBuffer>("imu", 5.0, 1500, late_margin_ns);
    odom_buffer_ = make_buffer<OdomBuffer>("odometry", 5.0, 500, late_margin_ns);
    ultrasound_buffer_ = make_buffer<UltrasoundBuffer>("ultrasound", 2.0, 200, late_margin_ns);
    eddy_buffer_ = make_buffer<EddyBuffer>("eddy_current", 2.0, 200, late_margin_ns);
    robot_buffer_ = make_buffer<RobotBuffer>("robot_state", 5.0, 500, late_margin_ns);
    probe_buffer_ = make_buffer<ProbeBuffer>("probe_state", 5.0, 500, late_margin_ns);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    auto eddy_qos = rclcpp::SensorDataQoS();
    eddy_qos.keep_last(200);
    auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    camera_qos.durability_volatile();
    if (camera_reliable_qos) {
      camera_qos.reliable();
    } else {
      camera_qos.best_effort();
    }
    camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/depth/image_rect_raw", camera_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        insert_message(*camera_buffer_, msg);
      });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        insert_message(*lidar_buffer_, msg);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {insert_message(*imu_buffer_, msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/robot/odometry", sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {insert_message(*odom_buffer_, msg);});
    ultrasound_sub_ = create_subscription<inspection_interfaces::msg::UltrasoundFrame>(
      "/inspection/ultrasound/raw", sensor_qos,
      [this](inspection_interfaces::msg::UltrasoundFrame::ConstSharedPtr msg) {
        const auto result = insert_message(*ultrasound_buffer_, msg);
        if (result.accepted && anchor_topic_ == "/inspection/ultrasound/raw") {
          consider_anchor(stamp_ns(msg->header.stamp), msg->sequence, anchor_topic_);
        }
      });
    eddy_sub_ = create_subscription<inspection_interfaces::msg::EddyCurrentFrame>(
      "/inspection/eddy_current/raw", eddy_qos,
      [this](inspection_interfaces::msg::EddyCurrentFrame::ConstSharedPtr msg) {
        const auto result = insert_message(*eddy_buffer_, msg);
        if (result.accepted && anchor_topic_ == "/inspection/eddy_current/raw") {
          consider_anchor(stamp_ns(msg->header.stamp), msg->sequence, anchor_topic_);
        }
      });
    robot_sub_ = create_subscription<inspection_interfaces::msg::RobotState>(
      "/robot/state", sensor_qos,
      [this](inspection_interfaces::msg::RobotState::ConstSharedPtr msg) {
        insert_message(*robot_buffer_, msg);
      });
    probe_sub_ = create_subscription<inspection_interfaces::msg::ProbeState>(
      "/inspection/probe_state", sensor_qos,
      [this](inspection_interfaces::msg::ProbeState::ConstSharedPtr msg) {
        insert_message(*probe_buffer_, msg);
      });

    if (anchor_topic_ != "/inspection/ultrasound/raw" &&
      anchor_topic_ != "/inspection/eddy_current/raw")
    {
      throw std::invalid_argument(
              "anchor_topic must be /inspection/ultrasound/raw or /inspection/eddy_current/raw");
    }

    fusion_pub_ = create_publisher<inspection_interfaces::msg::FusionIndex>(
      "/derived/inspection/fusion_index", rclcpp::QoS(100).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});

    running_.store(true);
    worker_ = std::thread([this]() {worker_loop();});
    RCLCPP_INFO(
      get_logger(),
      "Bounded fusion synchronizer started, anchor=%s, software_clock_match=%s, "
      "camera_offset=%.3f ms, livox_offset=%.3f ms",
      anchor_topic_.c_str(),
      !software_clock_match_enable_ ? "disabled" :
      (software_clock_adaptive_enable_ ? "adaptive" : "fixed"),
      camera_clock_offset_ns_.load() / 1.0e6,
      livox_clock_offset_ns_.load() / 1.0e6);
  }

  ~InspectionSyncNode() override
  {
    running_.store(false);
    pending_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct PendingAnchor
  {
    int64_t stamp_ns;
    uint64_t sequence;
    std::string topic;
    std::chrono::steady_clock::time_point ready_time;
  };

  static int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  static builtin_interfaces::msg::Time time_message(int64_t nanoseconds)
  {
    builtin_interfaces::msg::Time stamp;
    if (nanoseconds <= 0) {
      return stamp;
    }
    stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
  }

  int positive_int(const std::string & name, int default_value)
  {
    const int value = declare_parameter<int>(name, default_value);
    if (value <= 0) {throw std::invalid_argument(name + " must be positive");}
    return value;
  }

  double positive_double(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!(value > 0.0)) {throw std::invalid_argument(name + " must be positive");}
    return value;
  }

  int64_t milliseconds(const std::string & name, double default_value)
  {
    return static_cast<int64_t>(positive_double(name, default_value) * 1.0e6);
  }

  int64_t signed_milliseconds(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value)) {
      throw std::invalid_argument(name + " must be finite");
    }
    return static_cast<int64_t>(std::llround(value * 1.0e6));
  }

  template<typename BufferT>
  std::unique_ptr<BufferT> make_buffer(
    const std::string & prefix, double default_age_s, int default_count, int64_t late_margin_ns)
  {
    const double age = positive_double(prefix + "_max_age_s", default_age_s);
    const int count = positive_int(prefix + "_max_count", default_count);
    return std::make_unique<BufferT>(
      static_cast<size_t>(count), static_cast<int64_t>(age * 1.0e9), late_margin_ns);
  }

  template<typename BufferT, typename MessagePtrT>
  typename BufferT::InsertResult insert_message(BufferT & buffer, MessagePtrT msg)
  {
    const int64_t timestamp = stamp_ns(msg->header.stamp);
    const auto result = buffer.insert(timestamp, std::move(msg));
    if (!result.accepted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejected message with missing/invalid header stamp");
    }
    return result;
  }

  void enqueue_anchor(int64_t timestamp, uint64_t sequence, const std::string & topic)
  {
    PendingAnchor anchor{
      timestamp, sequence, topic,
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double, std::milli>(process_delay_ms_))};
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      if (pending_.size() >= pending_max_count_) {
        pending_.pop_front();
        ++sync_failures_;
        ++pending_dropped_;
      }
      pending_.push_back(std::move(anchor));
    }
    pending_cv_.notify_one();
  }

  void consider_anchor(int64_t timestamp, uint64_t sequence, const std::string & topic)
  {
    const uint64_t received_index = anchors_seen_.fetch_add(1);
    if (received_index % anchor_decimation_ != 0U) {
      ++anchors_decimated_;
      return;
    }
    ++anchors_enqueued_;
    enqueue_anchor(timestamp, sequence, topic);
  }

  void worker_loop()
  {
    std::unique_lock<std::mutex> lock(pending_mutex_);
    while (running_.load()) {
      pending_cv_.wait(lock, [this]() {return !running_.load() || !pending_.empty();});
      if (!running_.load()) {
        break;
      }
      const auto ready_time = pending_.front().ready_time;
      if (std::chrono::steady_clock::now() < ready_time) {
        pending_cv_.wait_until(lock, ready_time);
        continue;
      }
      PendingAnchor anchor = std::move(pending_.front());
      pending_.pop_front();
      lock.unlock();
      fuse_anchor(anchor);
      lock.lock();
    }
  }

  void fuse_anchor(const PendingAnchor & anchor)
  {
    inspection_interfaces::msg::FusionIndex output;
    output.header.stamp = time_message(anchor.stamp_ns);
    output.header.frame_id = "tank_course";
    output.anchor_sequence = anchor.sequence;
    output.anchor_topic = anchor.topic;
    output.interpolated_pose.orientation.w = 1.0;
    output.asset_id = asset_id_;
    output.course_id = course_id_;
    output.plate_id = plate_id_;
    output.weld_id = weld_id_;
    output.asset_coordinate_valid = false;

    const int64_t camera_offset_ns = software_clock_match_enable_ ?
      static_cast<int64_t>(std::llround(camera_clock_offset_ns_.load())) : 0;
    const int64_t livox_offset_ns = software_clock_match_enable_ ?
      static_cast<int64_t>(std::llround(livox_clock_offset_ns_.load())) : 0;

    const auto camera = camera_buffer_->nearest(
      anchor.stamp_ns - camera_offset_ns, camera_tolerance_ns_);
    if (camera) {
      output.camera_valid = true;
      const int64_t corrected_stamp_ns = camera->stamp_ns + camera_offset_ns;
      output.camera_stamp = time_message(corrected_stamp_ns);
      output.camera_time_error_ns = corrected_stamp_ns - anchor.stamp_ns;
      update_clock_match(
        camera_clock_offset_ns_, camera_clock_error_ema_ns_, camera_clock_samples_,
        camera_clock_window_sum_ns_, camera_clock_window_count_,
        output.camera_time_error_ns);
    }
    const auto lidar = lidar_buffer_->nearest(
      anchor.stamp_ns - livox_offset_ns, lidar_tolerance_ns_);
    if (lidar) {
      output.lidar_valid = true;
      const int64_t corrected_stamp_ns = lidar->stamp_ns + livox_offset_ns;
      output.lidar_stamp = time_message(corrected_stamp_ns);
      output.lidar_time_error_ns = corrected_stamp_ns - anchor.stamp_ns;
      update_clock_match(
        livox_clock_offset_ns_, livox_clock_error_ema_ns_, livox_clock_samples_,
        livox_clock_window_sum_ns_, livox_clock_window_count_,
        output.lidar_time_error_ns);
    }

    const auto imu_samples = imu_buffer_->range(
      anchor.stamp_ns - livox_offset_ns - imu_before_ns_,
      anchor.stamp_ns - livox_offset_ns + imu_after_ns_);
    if (!imu_samples.empty()) {
      const int64_t corrected_start_ns = imu_samples.front().stamp_ns + livox_offset_ns;
      const int64_t corrected_end_ns = imu_samples.back().stamp_ns + livox_offset_ns;
      output.imu_start_stamp = time_message(corrected_start_ns);
      output.imu_end_stamp = time_message(corrected_end_ns);
      output.imu_valid = corrected_start_ns <= anchor.stamp_ns &&
        corrected_end_ns >= anchor.stamp_ns;
    }

    const auto pose_pair = odom_buffer_->bracket(anchor.stamp_ns);
    if (pose_pair) {
      const auto & before = pose_pair->first;
      const auto & after = pose_pair->second;
      output.pose_before_stamp = time_message(before.stamp_ns);
      output.pose_after_stamp = time_message(after.stamp_ns);
      const int64_t before_gap = anchor.stamp_ns - before.stamp_ns;
      const int64_t after_gap = after.stamp_ns - anchor.stamp_ns;
      if (before_gap >= 0 && after_gap >= 0 &&
        before_gap <= pose_max_gap_ns_ && after_gap <= pose_max_gap_ns_)
      {
        const int64_t span = after.stamp_ns - before.stamp_ns;
        const double ratio = span == 0 ? 0.0 :
          static_cast<double>(anchor.stamp_ns - before.stamp_ns) / span;
        output.interpolated_pose = interpolate_pose(
          before.message->pose.pose, after.message->pose.pose, ratio);
        output.pose_valid = true;
        output.pose_confidence = static_cast<float>(std::clamp(
          1.0 - static_cast<double>(std::max(before_gap, after_gap)) / pose_max_gap_ns_,
          0.0, 1.0));
        if (asset_coordinate_valid_) {
          output.asset_coordinate_valid = true;
          if (asset_coordinate_mode_ == "tank_xyz") {
            const auto coordinates = tank_xyz_to_asset_coordinates(
              output.interpolated_pose.position, asset_radius_m_,
              asset_start_angle_rad_, asset_pose_standoff_m_);
            output.s_m = coordinates.s_m;
            output.v_m = coordinates.v_m;
            output.n_m = coordinates.n_m;
          } else {
            output.s_m = output.interpolated_pose.position.x;
            output.v_m = output.interpolated_pose.position.y;
            output.n_m = output.interpolated_pose.position.z;
          }
          output.u_m = output.s_m;
        }
      }
    }

    output.quality_flags = quality_flags(
      output.camera_valid, output.lidar_valid, output.imu_valid, output.pose_valid);
    output.invalid_reason = invalid_reason(
      output.camera_valid, output.lidar_valid, output.imu_valid, output.pose_valid);
    if (output.quality_flags == 0U) {
      ++fusion_successes_;
    } else {
      ++sync_failures_;
    }
    fusion_pub_->publish(std::move(output));
  }

  void update_clock_match(
    std::atomic<double> & offset_ns, std::atomic<double> & error_ema_ns,
    std::atomic<uint64_t> & samples, double & window_sum_ns,
    uint64_t & window_count, int64_t corrected_error_ns)
  {
    if (!software_clock_match_enable_) {
      return;
    }
    ++samples;
    if (!software_clock_adaptive_enable_) {
      return;
    }
    if (std::abs(corrected_error_ns) > clock_offset_update_gate_ns_) {
      return;
    }
    window_sum_ns += static_cast<double>(corrected_error_ns);
    ++window_count;
    if (window_count < clock_offset_window_matches_) {
      return;
    }
    const double window_mean_ns = window_sum_ns / static_cast<double>(window_count);
    const double current_offset = offset_ns.load();
    offset_ns.store(updated_clock_offset_ns(
      current_offset, static_cast<int64_t>(std::llround(window_mean_ns)),
      clock_offset_alpha_, clock_offset_max_ns_));
    const double current_ema = error_ema_ns.load();
    error_ema_ns.store(
      current_ema + clock_offset_alpha_ * (window_mean_ns - current_ema));
    window_sum_ns = 0.0;
    window_count = 0;
  }

  template<typename StatsT>
  void add_buffer_values(
    diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & name,
    const StatsT & stats, int64_t now_ns) const
  {
    add_value(status, name + ".length", std::to_string(stats.size));
    const double age_s = stats.oldest_stamp_ns > 0 ?
      std::max(0.0, static_cast<double>(now_ns - stats.oldest_stamp_ns) / 1.0e9) : 0.0;
    std::ostringstream age;
    age << std::fixed << std::setprecision(3) << age_s;
    add_value(status, name + ".oldest_age_s", age.str());
    add_value(status, name + ".received", std::to_string(stats.received));
    add_value(status, name + ".rejected", std::to_string(stats.rejected));
    add_value(status, name + ".late", std::to_string(stats.late));
    add_value(status, name + ".out_of_order", std::to_string(stats.out_of_order));
  }

  static void add_value(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "inspection_sync/bounded_caches";
    status.hardware_id = "software";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "bounded caches active";
    const int64_t now_ns = now().nanoseconds();
    const auto camera = camera_buffer_->stats();
    const auto lidar = lidar_buffer_->stats();
    const auto imu = imu_buffer_->stats();
    const auto odometry = odom_buffer_->stats();
    const auto ultrasound = ultrasound_buffer_->stats();
    const auto eddy = eddy_buffer_->stats();
    const auto robot = robot_buffer_->stats();
    const auto probe = probe_buffer_->stats();
    add_buffer_values(status, "camera", camera, now_ns);
    add_buffer_values(status, "lidar", lidar, now_ns);
    add_buffer_values(status, "imu", imu, now_ns);
    add_buffer_values(status, "odometry", odometry, now_ns);
    add_buffer_values(status, "ultrasound", ultrasound, now_ns);
    add_buffer_values(status, "eddy_current", eddy, now_ns);
    add_buffer_values(status, "robot_state", robot, now_ns);
    add_buffer_values(status, "probe_state", probe, now_ns);
    size_t pending_size = 0;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_size = pending_.size();
    }
    add_value(status, "pending_fusion", std::to_string(pending_size));
    add_value(status, "anchor_decimation", std::to_string(anchor_decimation_));
    add_value(status, "anchors_seen", std::to_string(anchors_seen_.load()));
    add_value(status, "anchors_enqueued", std::to_string(anchors_enqueued_.load()));
    add_value(status, "anchors_decimated", std::to_string(anchors_decimated_.load()));
    add_value(
      status, "software_clock_match_enable",
      software_clock_match_enable_ ? "true" : "false");
    add_value(
      status, "software_clock_adaptive_enable",
      software_clock_adaptive_enable_ ? "true" : "false");
    add_value(
      status, "camera_clock_offset_ns",
      std::to_string(static_cast<int64_t>(std::llround(camera_clock_offset_ns_.load()))));
    add_value(
      status, "camera_clock_error_ema_ns",
      std::to_string(static_cast<int64_t>(std::llround(camera_clock_error_ema_ns_.load()))));
    add_value(status, "camera_clock_samples", std::to_string(camera_clock_samples_.load()));
    add_value(
      status, "livox_clock_offset_ns",
      std::to_string(static_cast<int64_t>(std::llround(livox_clock_offset_ns_.load()))));
    add_value(
      status, "livox_clock_error_ema_ns",
      std::to_string(static_cast<int64_t>(std::llround(livox_clock_error_ema_ns_.load()))));
    add_value(status, "livox_clock_samples", std::to_string(livox_clock_samples_.load()));
    add_value(
      status, "clock_offset_window_matches",
      std::to_string(clock_offset_window_matches_));
    add_value(
      status, "clock_offset_update_gate_ns",
      std::to_string(clock_offset_update_gate_ns_));
    add_value(status, "fusion_success", std::to_string(fusion_successes_.load()));
    add_value(status, "sync_failure", std::to_string(sync_failures_.load()));
    add_value(status, "pending_dropped", std::to_string(pending_dropped_.load()));
    const uint64_t total_late = camera.late + lidar.late + imu.late + odometry.late +
      ultrasound.late + eddy.late + robot.late + probe.late;
    const uint64_t total_out_of_order = camera.out_of_order + lidar.out_of_order +
      imu.out_of_order + odometry.out_of_order + ultrasound.out_of_order +
      eddy.out_of_order + robot.out_of_order + probe.out_of_order;
    add_value(status, "late_messages_total", std::to_string(total_late));
    add_value(status, "out_of_order_total", std::to_string(total_out_of_order));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  int64_t camera_tolerance_ns_{0};
  int64_t lidar_tolerance_ns_{0};
  int64_t pose_max_gap_ns_{0};
  int64_t imu_before_ns_{0};
  int64_t imu_after_ns_{0};
  double process_delay_ms_{0.0};
  size_t pending_max_count_{0};
  uint64_t anchor_decimation_{1};
  bool software_clock_match_enable_{false};
  bool software_clock_adaptive_enable_{false};
  double clock_offset_alpha_{0.25};
  double clock_offset_max_ns_{20000000.0};
  int64_t clock_offset_update_gate_ns_{45000000};
  uint64_t clock_offset_window_matches_{50};
  std::string anchor_topic_;
  std::string asset_id_;
  std::string course_id_;
  std::string plate_id_;
  std::string weld_id_;
  bool asset_coordinate_valid_{false};
  std::string asset_coordinate_mode_{"unwrapped_pose"};
  double asset_radius_m_{1.0};
  double asset_start_angle_rad_{0.0};
  double asset_pose_standoff_m_{0.0};

  std::unique_ptr<ImageBuffer> camera_buffer_;
  std::unique_ptr<LidarBuffer> lidar_buffer_;
  std::unique_ptr<ImuBuffer> imu_buffer_;
  std::unique_ptr<OdomBuffer> odom_buffer_;
  std::unique_ptr<UltrasoundBuffer> ultrasound_buffer_;
  std::unique_ptr<EddyBuffer> eddy_buffer_;
  std::unique_ptr<RobotBuffer> robot_buffer_;
  std::unique_ptr<ProbeBuffer> probe_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<inspection_interfaces::msg::UltrasoundFrame>::SharedPtr ultrasound_sub_;
  rclcpp::Subscription<inspection_interfaces::msg::EddyCurrentFrame>::SharedPtr eddy_sub_;
  rclcpp::Subscription<inspection_interfaces::msg::RobotState>::SharedPtr robot_sub_;
  rclcpp::Subscription<inspection_interfaces::msg::ProbeState>::SharedPtr probe_sub_;
  rclcpp::Publisher<inspection_interfaces::msg::FusionIndex>::SharedPtr fusion_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::atomic<bool> running_{false};
  std::thread worker_;
  std::mutex pending_mutex_;
  std::condition_variable pending_cv_;
  std::deque<PendingAnchor> pending_;
  std::atomic<uint64_t> fusion_successes_{0};
  std::atomic<uint64_t> sync_failures_{0};
  std::atomic<uint64_t> pending_dropped_{0};
  std::atomic<uint64_t> anchors_seen_{0};
  std::atomic<uint64_t> anchors_enqueued_{0};
  std::atomic<uint64_t> anchors_decimated_{0};
  std::atomic<double> camera_clock_offset_ns_{0.0};
  std::atomic<double> camera_clock_error_ema_ns_{0.0};
  std::atomic<uint64_t> camera_clock_samples_{0};
  double camera_clock_window_sum_ns_{0.0};
  uint64_t camera_clock_window_count_{0};
  std::atomic<double> livox_clock_offset_ns_{0.0};
  std::atomic<double> livox_clock_error_ema_ns_{0.0};
  std::atomic<uint64_t> livox_clock_samples_{0};
  double livox_clock_window_sum_ns_{0.0};
  uint64_t livox_clock_window_count_{0};
};

}  // namespace inspection_sync

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_sync::InspectionSyncNode>());
  rclcpp::shutdown();
  return 0;
}
