#include <cmath>

#include <gtest/gtest.h>

#include "inspection_livox_adapter/imu_conversion.hpp"

using inspection_livox_adapter::convert_imu_to_si;
using inspection_livox_adapter::kStandardGravityMps2;

TEST(ImuConversion, ConvertsAccelerationAndMarksOrientationUnavailable)
{
  sensor_msgs::msg::Imu input;
  input.header.stamp.sec = 123;
  input.header.frame_id = "livox_frame";
  input.angular_velocity.z = 0.2;
  input.linear_acceleration.x = 0.25;
  input.linear_acceleration.y = -0.5;
  input.linear_acceleration.z = 1.0;

  sensor_msgs::msg::Imu output;
  ASSERT_TRUE(convert_imu_to_si(input, output));
  EXPECT_EQ(output.header, input.header);
  EXPECT_DOUBLE_EQ(output.angular_velocity.z, 0.2);
  EXPECT_DOUBLE_EQ(output.linear_acceleration.x, 0.25 * kStandardGravityMps2);
  EXPECT_DOUBLE_EQ(output.linear_acceleration.y, -0.5 * kStandardGravityMps2);
  EXPECT_DOUBLE_EQ(output.linear_acceleration.z, kStandardGravityMps2);
  EXPECT_DOUBLE_EQ(output.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(output.orientation_covariance[0], -1.0);
}

TEST(ImuConversion, RejectsZeroTimestampAndNonFiniteInput)
{
  sensor_msgs::msg::Imu input;
  sensor_msgs::msg::Imu output;
  EXPECT_FALSE(convert_imu_to_si(input, output));

  input.header.stamp.sec = 1;
  input.linear_acceleration.x = std::nan("");
  EXPECT_FALSE(convert_imu_to_si(input, output));
}
