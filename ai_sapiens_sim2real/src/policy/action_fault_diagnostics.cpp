// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "ai_sapiens_sim2real/policy/action_fault_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace ai_sapiens_sim2real
{
namespace
{
constexpr double kDegreesPerRadian = 180.0 / 3.141592653589793;
// Heuristics for highlighting only: equivalent to 20 deg and 500 deg/s
// changes over 20 ms. Scale by the actual interval between observations.
constexpr double kAngleChangeRate = 1000.0;  // deg/s
constexpr double kVelocityChangeRate = 25000.0;  // deg/s^2

Eigen::Vector3d roll_pitch_yaw(const Eigen::Quaternionf & orientation)
{
  if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-6f) {
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix().cast<double>();
  return Eigen::Vector3d(
    std::atan2(rotation(2, 1), rotation(2, 2)),
    std::atan2(-rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0))),
    std::atan2(rotation(1, 0), rotation(0, 0))) * kDegreesPerRadian;
}

constexpr int kValueWidth = 10;

void append_columns(std::ostream & out, const char * current_label)
{
  out << " " << std::right << std::setw(kValueWidth) << current_label
      << " " << std::setw(kValueWidth) << "prev" << " " << std::setw(kValueWidth) << "delta"
      << " flag";
}

bool append_value(
  std::ostream & out, double current, std::optional<double> previous,
  double jump_limit, bool wrap_angle = false)
{
  std::string flag;
  if (!std::isfinite(current)) {flag = "[NONFINITE]";}
  out << " " << std::right << std::setw(kValueWidth) << current;
  if (!previous) {
    out << " " << std::setw(kValueWidth) << "n/a" << " " << std::setw(kValueWidth) << "n/a";
  } else {
    const double delta = wrap_angle ? std::remainder(current - *previous, 360.0) :
      current - *previous;
    out << " " << std::setw(kValueWidth) << *previous
        << " " << std::showpos << std::setw(kValueWidth) << delta << std::noshowpos;
    if (flag.empty() && !std::isfinite(*previous)) {flag = "[PREV_NONFINITE]";}
    if (std::isfinite(delta) && std::isfinite(jump_limit) && std::abs(delta) > jump_limit) {
      flag = "[JUMP!]";
    }
  }
  if (!flag.empty()) {out << " " << flag;}
  return flag == "[JUMP!]";
}

std::optional<double> previous_joint_value(
  const std::optional<SensorData> & previous, size_t index, bool velocity)
{
  if (!previous) {return std::nullopt;}
  const auto & values = velocity ? previous->joint_vel : previous->joint_pos;
  if (index >= static_cast<size_t>(values.size())) {return std::nullopt;}
  return values[index] * kDegreesPerRadian;
}
}  // namespace

std::string compact_joint_name(const std::string & name)
{
  std::string result = name;
  if (result.compare(0, 5, "left_") == 0) {result.replace(0, 5, "L.");}
  else if (result.compare(0, 6, "right_") == 0) {result.replace(0, 6, "R.");}
  if (result.size() >= 6 && result.compare(result.size() - 6, 6, "_joint") == 0) {
    result.resize(result.size() - 6);
  }
  return result;
}

std::string format_action_fault_sensors(
  const SensorData & current, const std::optional<SensorData> & previous,
  const std::vector<std::string> & joint_names, double elapsed_seconds, bool jumps_only)
{
  const bool has_interval = previous && std::isfinite(elapsed_seconds) && elapsed_seconds > 0.0;
  const double interval = has_interval ? elapsed_seconds :
    std::numeric_limits<double>::quiet_NaN();
  size_t name_width = 5;
  for (const auto & name : joint_names) {
    name_width = std::max(name_width, compact_joint_name(name).size());
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision(3);
  if (!jumps_only) {
    out << "Sensors: dt=" << interval
        << " s; IMU usable=" << (current.is_imu_usable ? "true" : "false")
        << "\nJump thresholds (diagnostic):\n  angle=" << kAngleChangeRate * interval
        << " deg, velocity=" << kVelocityChangeRate * interval << " deg/s\n";
  }
  // Emit a table only when it has a selected row. Console selection and full
  // reports share the same numeric jump decision, without parsing log text.
  const auto table = [&](const std::string & title, const std::vector<std::string> & names,
      const std::vector<double> & values, const std::vector<std::optional<double>> & old,
      double limit, bool wrap, size_t width, const char * label) {
      bool has_header = false;
      for (size_t i = 0; i < names.size(); ++i) {
        std::ostringstream row;
        row << std::fixed << std::setprecision(3)
            << "  " << std::left << std::setw(width) << names[i];
        bool is_jump = false;
        if (i < values.size()) {is_jump = append_value(row, values[i], old[i], limit, wrap);}
        else {row << " [MISSING]";}
        if (jumps_only && !is_jump) {continue;}
        if (!has_header) {
          out << "\n" << title << "\n  " << std::left << std::setw(width) << label;
          append_columns(out, "current");
          out << "\n";
          has_header = true;
        }
        out << row.str() << "\n";
      }
    };
  const auto rpy = roll_pitch_yaw(current.orientation);
  const auto old_rpy = previous ? roll_pitch_yaw(previous->orientation) : Eigen::Vector3d::Zero().eval();
  std::vector<double> angles, gyro;
  std::vector<std::optional<double>> old_angles, old_gyro;
  for (size_t i = 0; i < 3; ++i) {
    angles.push_back(rpy[i]);
    gyro.push_back(current.angular_velocity[i] * kDegreesPerRadian);
    old_angles.push_back(previous ? std::optional<double>(old_rpy[i]) : std::nullopt);
    old_gyro.push_back(previous ?
      std::optional<double>(previous->angular_velocity[i] * kDegreesPerRadian) : std::nullopt);
  }
  table("IMU RPY [deg, ZYX; wrapped delta]", {"roll", "pitch", "yaw"}, angles,
    old_angles, kAngleChangeRate * interval, true, 5, "axis");
  table("IMU gyro [deg/s, body axes]", {"wx", "wy", "wz"}, gyro,
    old_gyro, kVelocityChangeRate * interval, false, 5, "axis");
  for (const bool is_velocity : {false, true}) {
    const auto & source = is_velocity ? current.joint_vel : current.joint_pos;
    std::vector<std::string> names;
    std::vector<double> values;
    std::vector<std::optional<double>> old;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      names.push_back(compact_joint_name(joint_names[i]));
      if (i < static_cast<size_t>(source.size())) {values.push_back(source[i] * kDegreesPerRadian);}
      old.push_back(previous_joint_value(previous, i, is_velocity));
    }
    table(is_velocity ? "Joint velocity [deg/s]" : "Joint position [deg]", names,
      values, old, (is_velocity ? kVelocityChangeRate : kAngleChangeRate) * interval,
      false, name_width, "joint");
  }
  return out.str();
}

std::string format_action_fault_actions(
  const std::vector<float> & processed, const std::vector<std::string> & joints,
  float limit, bool invalid_only)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3);
  for (size_t i = 0; i < processed.size(); ++i) {
    const auto value = processed[i];
    const bool is_invalid = !std::isfinite(value) || std::abs(value) > limit;
    if (invalid_only && !is_invalid) {continue;}
    out << "  " << std::left << std::setw(16) << compact_joint_name(joints.at(i))
        << " " << std::right << std::setw(10) << value;
    if (!std::isfinite(value)) {out << " [NONFINITE]";}
    else if (is_invalid) {out << " [LIMIT]";}
    out << "\n";
  }
  return out.str();
}

}  // namespace ai_sapiens_sim2real
