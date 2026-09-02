#ifndef INSPECTION_LIVOX_ADAPTER__IMU_CONVERSION_HPP_
#define INSPECTION_LIVOX_ADAPTER__IMU_CONVERSION_HPP_

#include <cmath>

#include "sensor_msgs/msg/imu.hpp"

namespace inspection_livox_adapter
{

constexpr double kStandardGravityMps2 = 9.80665;

inline bool convert_imu_to_si(
  const sensor_msgs::msg::Imu & input,
  sensor_msgs::msg::Imu & output,
  const double acceleration_scale = kStandardGravityMps2)
{
  const auto stamp_ns =
    static_cast<int64_t>(input.header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(input.header.stamp.nanosec);
  if (stamp_ns <= 0 || !std::isfinite(acceleration_scale) || acceleration_scale <= 0.0 ||
    !std::isfinite(input.angular_velocity.x) ||
    !std::isfinite(input.angular_velocity.y) ||
    !std::isfinite(input.angular_velocity.z) ||
    !std::isfinite(input.linear_acceleration.x) ||
    !std::isfinite(input.linear_acceleration.y) ||
    !std::isfinite(input.linear_acceleration.z))
  {
    return false;
  }

  output = input;
  output.linear_acceleration.x *= acceleration_scale;
  output.linear_acceleration.y *= acceleration_scale;
  output.linear_acceleration.z *= acceleration_scale;

  // MID-360 supplies angular velocity and acceleration, but no orientation estimate.
  output.orientation.x = 0.0;
  output.orientation.y = 0.0;
  output.orientation.z = 0.0;
  output.orientation.w = 1.0;
  output.orientation_covariance.fill(0.0);
  output.orientation_covariance[0] = -1.0;
  return true;
}

}  // namespace inspection_livox_adapter

#endif  // INSPECTION_LIVOX_ADAPTER__IMU_CONVERSION_HPP_
