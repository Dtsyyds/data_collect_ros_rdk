#ifndef INSPECTION_SYNC__SYNC_MATH_HPP_
#define INSPECTION_SYNC__SYNC_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace inspection_sync
{

struct AssetCoordinates
{
  double s_m;
  double v_m;
  double n_m;
};

inline AssetCoordinates tank_xyz_to_asset_coordinates(
  const geometry_msgs::msg::Point & position, double radius_m,
  double start_angle_rad, double pose_standoff_m)
{
  constexpr double two_pi = 6.28318530717958647692;
  double angle = std::atan2(position.y, position.x) - start_angle_rad;
  angle = std::fmod(angle, two_pi);
  if (angle < 0.0) {
    angle += two_pi;
  }
  return AssetCoordinates{
    radius_m * angle,
    position.z,
    std::hypot(position.x, position.y) - radius_m - pose_standoff_m};
}

inline double updated_clock_offset_ns(
  double current_offset_ns, int64_t corrected_error_ns,
  double alpha, double max_abs_offset_ns)
{
  const double candidate = current_offset_ns - alpha * corrected_error_ns;
  return std::clamp(candidate, -max_abs_offset_ns, max_abs_offset_ns);
}

inline geometry_msgs::msg::Quaternion normalized(geometry_msgs::msg::Quaternion q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1.0e-12) {
    q.x = q.y = q.z = 0.0;
    q.w = 1.0;
    return q;
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return q;
}

inline geometry_msgs::msg::Quaternion slerp(
  geometry_msgs::msg::Quaternion first, geometry_msgs::msg::Quaternion second, double ratio)
{
  first = normalized(first);
  second = normalized(second);
  ratio = std::clamp(ratio, 0.0, 1.0);
  double dot = first.x * second.x + first.y * second.y + first.z * second.z + first.w * second.w;
  if (dot < 0.0) {
    second.x = -second.x;
    second.y = -second.y;
    second.z = -second.z;
    second.w = -second.w;
    dot = -dot;
  }
  if (dot > 0.9995) {
    geometry_msgs::msg::Quaternion result;
    result.x = first.x + ratio * (second.x - first.x);
    result.y = first.y + ratio * (second.y - first.y);
    result.z = first.z + ratio * (second.z - first.z);
    result.w = first.w + ratio * (second.w - first.w);
    return normalized(result);
  }
  const double angle = std::acos(std::clamp(dot, -1.0, 1.0));
  const double sin_angle = std::sin(angle);
  const double first_weight = std::sin((1.0 - ratio) * angle) / sin_angle;
  const double second_weight = std::sin(ratio * angle) / sin_angle;
  geometry_msgs::msg::Quaternion result;
  result.x = first_weight * first.x + second_weight * second.x;
  result.y = first_weight * first.y + second_weight * second.y;
  result.z = first_weight * first.z + second_weight * second.z;
  result.w = first_weight * first.w + second_weight * second.w;
  return normalized(result);
}

inline geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & before, const geometry_msgs::msg::Pose & after, double ratio)
{
  ratio = std::clamp(ratio, 0.0, 1.0);
  geometry_msgs::msg::Pose result;
  result.position.x = before.position.x + ratio * (after.position.x - before.position.x);
  result.position.y = before.position.y + ratio * (after.position.y - before.position.y);
  result.position.z = before.position.z + ratio * (after.position.z - before.position.z);
  result.orientation = slerp(before.orientation, after.orientation, ratio);
  return result;
}

inline std::string invalid_reason(
  bool camera_valid, bool lidar_valid, bool imu_valid, bool pose_valid)
{
  std::vector<std::string> missing;
  if (!camera_valid) {missing.emplace_back("camera_missing_or_outside_tolerance");}
  if (!lidar_valid) {missing.emplace_back("lidar_missing_or_outside_tolerance");}
  if (!imu_valid) {missing.emplace_back("imu_window_incomplete");}
  if (!pose_valid) {missing.emplace_back("pose_bracket_missing_or_gap_exceeded");}
  std::string result;
  for (size_t i = 0; i < missing.size(); ++i) {
    if (i != 0) {result += ";";}
    result += missing[i];
  }
  return result;
}

inline uint32_t quality_flags(
  bool camera_valid, bool lidar_valid, bool imu_valid, bool pose_valid)
{
  return (camera_valid ? 0U : 1U) |
         (lidar_valid ? 0U : 2U) |
         (imu_valid ? 0U : 4U) |
         (pose_valid ? 0U : 8U);
}

}  // namespace inspection_sync

#endif  // INSPECTION_SYNC__SYNC_MATH_HPP_
