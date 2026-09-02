#include <filesystem>
#include <fstream>

#include "gtest/gtest.h"
#include "inspection_mock_sensors/mission_trajectory.hpp"

namespace
{

TEST(MissionTrajectory, LoadsInterpolatesAndHonorsBreaks)
{
  const auto path = std::filesystem::temp_directory_path() / "mission_trajectory_test.json";
  std::ofstream output(path);
  output << R"({
    "coordinate_contract": "cylindrical_unwrapped_sz",
    "mission_id": "test-mission",
    "radius_m": 15.0,
    "height_m": 18.0,
    "circumference_m": 94.24777960769379,
    "start_angle_rad": 0.0,
    "points": [
      {"s_m": 1.0, "z_m": 2.0, "break_before": true, "action": "start"},
      {"s_m": 3.0, "z_m": 2.0, "break_before": false, "action": "scan"},
      {"s_m": 20.0, "z_m": 4.0, "break_before": true, "action": "manual_reposition"}
    ]
  })";
  output.close();

  const auto trajectory = inspection_mock_sensors::MissionTrajectory::load(
    path.string(), 15.0, 18.0, 0.0, 2.0, 0.5);
  EXPECT_EQ(trajectory.size(), 3U);
  EXPECT_EQ(trajectory.break_count(), 2U);
  EXPECT_EQ(trajectory.mission_id(), "test-mission");
  const auto midpoint = trajectory.pose(1.0, false);
  EXPECT_NEAR(midpoint.s_m, 2.0, 1.0e-9);
  EXPECT_NEAR(midpoint.z_m, 2.0, 1.0e-9);
  const auto repositioned = trajectory.pose(1.6, false);
  EXPECT_NEAR(repositioned.s_m, 20.0, 1.0e-9);
  EXPECT_NEAR(repositioned.z_m, 4.0, 1.0e-9);
  std::filesystem::remove(path);
}

}  // namespace
