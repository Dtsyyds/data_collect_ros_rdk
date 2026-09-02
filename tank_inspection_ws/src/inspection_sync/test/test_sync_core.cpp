#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "inspection_sync/sync_math.hpp"
#include "inspection_sync/timestamped_ring_buffer.hpp"

namespace
{

struct DummyMessage
{
  int value;
};

using Buffer = inspection_sync::TimestampedRingBuffer<DummyMessage>;

std::shared_ptr<const DummyMessage> message(int value)
{
  return std::make_shared<const DummyMessage>(DummyMessage{value});
}

TEST(TimestampedRingBuffer, EnforcesCapacityLimit)
{
  Buffer buffer(3, 1000, 100);
  for (int i = 1; i <= 5; ++i) {
    ASSERT_TRUE(buffer.insert(i * 10, message(i)).accepted);
  }
  const auto stats = buffer.stats();
  EXPECT_EQ(stats.size, 3U);
  EXPECT_EQ(stats.oldest_stamp_ns, 30);
}

TEST(TimestampedRingBuffer, EvictsOldDataByAge)
{
  Buffer buffer(10, 100, 25);
  buffer.insert(100, message(1));
  buffer.insert(150, message(2));
  buffer.insert(250, message(3));
  const auto stats = buffer.stats();
  EXPECT_EQ(stats.size, 2U);
  EXPECT_EQ(stats.oldest_stamp_ns, 150);
}

TEST(TimestampedRingBuffer, InsertsOutOfOrderAndCountsLateMessages)
{
  Buffer buffer(10, 1000, 50);
  buffer.insert(200, message(2));
  const auto result = buffer.insert(100, message(1));
  EXPECT_TRUE(result.out_of_order);
  EXPECT_TRUE(result.late);
  const auto values = buffer.range(0, 300);
  ASSERT_EQ(values.size(), 2U);
  EXPECT_EQ(values[0].message->value, 1);
  EXPECT_EQ(buffer.stats().out_of_order, 1U);
  EXPECT_EQ(buffer.stats().late, 1U);
}

TEST(TimestampedRingBuffer, SelectsNearestCameraFrame)
{
  Buffer buffer(10, 1000, 100);
  buffer.insert(900, message(1));
  buffer.insert(1030, message(2));
  buffer.insert(1200, message(3));
  const auto match = buffer.nearest(1000, 50);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match->stamp_ns, 1030);
  EXPECT_FALSE(buffer.nearest(1100, 50).has_value());
}

TEST(SyncMath, InterpolatesTranslation)
{
  geometry_msgs::msg::Pose first;
  geometry_msgs::msg::Pose second;
  first.orientation.w = 1.0;
  second.orientation.w = 1.0;
  second.position.x = 10.0;
  second.position.y = 4.0;
  const auto pose = inspection_sync::interpolate_pose(first, second, 0.25);
  EXPECT_DOUBLE_EQ(pose.position.x, 2.5);
  EXPECT_DOUBLE_EQ(pose.position.y, 1.0);
}

TEST(SyncMath, UsesQuaternionSlerp)
{
  geometry_msgs::msg::Quaternion first;
  geometry_msgs::msg::Quaternion second;
  first.w = 1.0;
  second.z = 1.0;
  second.w = 0.0;
  const auto midpoint = inspection_sync::slerp(first, second, 0.5);
  EXPECT_NEAR(midpoint.z, std::sqrt(0.5), 1.0e-6);
  EXPECT_NEAR(midpoint.w, std::sqrt(0.5), 1.0e-6);
}

TEST(SyncMath, ReportsTimeoutAndMissingModalities)
{
  const auto reason = inspection_sync::invalid_reason(false, true, false, false);
  EXPECT_NE(reason.find("camera_missing"), std::string::npos);
  EXPECT_NE(reason.find("imu_window_incomplete"), std::string::npos);
  EXPECT_NE(reason.find("pose_bracket_missing"), std::string::npos);
  EXPECT_EQ(inspection_sync::quality_flags(false, true, false, false), 13U);
  EXPECT_TRUE(inspection_sync::invalid_reason(true, true, true, true).empty());
}

TEST(SyncMath, SoftwareClockOffsetCancelsSignedBiasAndIsBounded)
{
  double offset_ns = 0.0;
  constexpr double alpha = 0.01;
  constexpr double max_offset_ns = 50000000.0;
  constexpr int64_t raw_bias_ns = -2000000;
  for (int i = 0; i < 1000; ++i) {
    const auto corrected_error_ns = static_cast<int64_t>(raw_bias_ns + offset_ns);
    offset_ns = inspection_sync::updated_clock_offset_ns(
      offset_ns, corrected_error_ns, alpha, max_offset_ns);
  }
  EXPECT_NEAR(offset_ns, 2000000.0, 100.0);

  offset_ns = inspection_sync::updated_clock_offset_ns(
    0.0, -1000000000, 1.0, max_offset_ns);
  EXPECT_DOUBLE_EQ(offset_ns, max_offset_ns);
}

TEST(SyncMath, ConvertsTankXyzToUnwrappedAssetCoordinates)
{
  geometry_msgs::msg::Point position;
  position.x = 0.0;
  position.y = 15.3;
  position.z = 2.5;
  const auto coordinates = inspection_sync::tank_xyz_to_asset_coordinates(
    position, 15.0, 0.0, 0.3);
  EXPECT_NEAR(coordinates.s_m, 15.0 * std::acos(-1.0) / 2.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(coordinates.v_m, 2.5);
  EXPECT_NEAR(coordinates.n_m, 0.0, 1.0e-9);

  position.x = 15.3;
  position.y = -1.0e-9;
  const auto wrapped = inspection_sync::tank_xyz_to_asset_coordinates(
    position, 15.0, 0.0, 0.3);
  EXPECT_GT(wrapped.s_m, 2.0 * std::acos(-1.0) * 15.0 - 1.0e-6);
}

}  // namespace
