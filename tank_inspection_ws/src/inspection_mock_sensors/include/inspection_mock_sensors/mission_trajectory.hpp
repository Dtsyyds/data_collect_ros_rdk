#ifndef INSPECTION_MOCK_SENSORS__MISSION_TRAJECTORY_HPP_
#define INSPECTION_MOCK_SENSORS__MISSION_TRAJECTORY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace inspection_mock_sensors
{

struct MissionTrajectoryPoint
{
  double s_m;
  double z_m;
  bool break_before;
  std::string action;
};

struct MissionTrajectoryPose
{
  double s_m;
  double z_m;
  std::string action;
};

class MissionTrajectory
{
public:
  static MissionTrajectory load(
    const std::string & path, double expected_radius_m, double expected_height_m,
    double expected_start_angle_rad, double speed_m_s, double dwell_s)
  {
    std::ifstream input(path);
    if (!input) {
      throw std::runtime_error("mission trajectory not found: " + path);
    }
    nlohmann::json data;
    input >> data;
    if (data.value("coordinate_contract", "") != "cylindrical_unwrapped_sz") {
      throw std::invalid_argument("mission trajectory has an unsupported coordinate contract");
    }
    const double radius_m = data.at("radius_m").get<double>();
    const double height_m = data.at("height_m").get<double>();
    const double circumference_m = data.at("circumference_m").get<double>();
    const double start_angle_rad = data.at("start_angle_rad").get<double>();
    if (!close(radius_m, expected_radius_m) || !close(height_m, expected_height_m) ||
      !close(start_angle_rad, expected_start_angle_rad))
    {
      throw std::invalid_argument("mission trajectory geometry does not match mock tank geometry");
    }
    if (!(speed_m_s > 0.0) || !(dwell_s > 0.0) ||
      !std::isfinite(speed_m_s) || !std::isfinite(dwell_s))
    {
      throw std::invalid_argument("mission trajectory speed and dwell must be positive and finite");
    }

    std::vector<MissionTrajectoryPoint> points;
    for (const auto & item : data.at("points")) {
      MissionTrajectoryPoint point{
        item.at("s_m").get<double>(), item.at("z_m").get<double>(),
        item.value("break_before", false), item.value("action", "move")};
      if (!std::isfinite(point.s_m) || !std::isfinite(point.z_m) ||
        point.z_m < -1.0e-9 || point.z_m > height_m + 1.0e-9)
      {
        throw std::invalid_argument("mission trajectory contains an invalid point");
      }
      point.s_m = normalize_s(point.s_m, circumference_m);
      points.push_back(std::move(point));
    }
    if (points.empty()) {
      throw std::invalid_argument("mission trajectory contains no points");
    }
    points.front().break_before = true;
    return MissionTrajectory(
      std::move(points), circumference_m, speed_m_s, dwell_s,
      data.value("mission_id", ""));
  }

  MissionTrajectoryPose pose(double time_s, bool loop) const
  {
    if (!std::isfinite(time_s)) {
      throw std::invalid_argument("mission trajectory time must be finite");
    }
    double phase = std::max(0.0, time_s);
    if (loop) {
      phase = std::fmod(phase, duration_s_);
    } else if (phase >= duration_s_) {
      const auto & point = points_.back();
      return MissionTrajectoryPose{point.s_m, point.z_m, point.action};
    }
    const auto iterator = std::upper_bound(segment_end_s_.begin(), segment_end_s_.end(), phase);
    const size_t index = std::min(
      static_cast<size_t>(std::distance(segment_end_s_.begin(), iterator)),
      points_.size() - 1);
    const auto & destination = points_[index];
    if (index == 0 || destination.break_before) {
      return MissionTrajectoryPose{destination.s_m, destination.z_m, destination.action};
    }
    const double start_s = segment_end_s_[index - 1];
    const double segment_duration = segment_end_s_[index] - start_s;
    const double fraction = segment_duration > 0.0 ?
      std::clamp((phase - start_s) / segment_duration, 0.0, 1.0) : 1.0;
    const auto & source = points_[index - 1];
    const double delta_s = shortest_delta_s(source.s_m, destination.s_m, circumference_m_);
    return MissionTrajectoryPose{
      normalize_s(source.s_m + delta_s * fraction, circumference_m_),
      source.z_m + (destination.z_m - source.z_m) * fraction,
      destination.action};
  }

  size_t size() const {return points_.size();}
  size_t break_count() const
  {
    return static_cast<size_t>(std::count_if(
      points_.begin(), points_.end(),
      [](const auto & point) {return point.break_before;}));
  }
  double duration_s() const {return duration_s_;}
  const std::string & mission_id() const {return mission_id_;}

private:
  MissionTrajectory(
    std::vector<MissionTrajectoryPoint> points, double circumference_m,
    double speed_m_s, double dwell_s, std::string mission_id)
  : points_(std::move(points)), circumference_m_(circumference_m),
    mission_id_(std::move(mission_id))
  {
    segment_end_s_.reserve(points_.size());
    double elapsed = dwell_s;
    segment_end_s_.push_back(elapsed);
    for (size_t index = 1; index < points_.size(); ++index) {
      const auto & source = points_[index - 1];
      const auto & destination = points_[index];
      const double delta_s = shortest_delta_s(source.s_m, destination.s_m, circumference_m_);
      const double distance = std::hypot(delta_s, destination.z_m - source.z_m);
      const double segment_duration = destination.break_before || distance < 1.0e-9 ?
        dwell_s : distance / speed_m_s;
      elapsed += segment_duration;
      segment_end_s_.push_back(elapsed);
    }
    duration_s_ = elapsed;
  }

  static bool close(double first, double second)
  {
    return std::isfinite(first) && std::isfinite(second) &&
           std::abs(first - second) <= 1.0e-6;
  }

  static double normalize_s(double value, double circumference_m)
  {
    double result = std::fmod(value, circumference_m);
    if (result < 0.0) {
      result += circumference_m;
    }
    return result;
  }

  static double shortest_delta_s(double first, double second, double circumference_m)
  {
    double delta = second - first;
    if (delta > circumference_m / 2.0) {
      delta -= circumference_m;
    } else if (delta < -circumference_m / 2.0) {
      delta += circumference_m;
    }
    return delta;
  }

  std::vector<MissionTrajectoryPoint> points_;
  std::vector<double> segment_end_s_;
  double circumference_m_{0.0};
  double duration_s_{0.0};
  std::string mission_id_;
};

}  // namespace inspection_mock_sensors

#endif  // INSPECTION_MOCK_SENSORS__MISSION_TRAJECTORY_HPP_
