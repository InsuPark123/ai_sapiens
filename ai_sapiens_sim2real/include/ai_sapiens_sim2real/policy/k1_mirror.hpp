// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#ifndef AI_SAPIENS_SIM2REAL__POLICY__K1_MIRROR_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__K1_MIRROR_HPP_

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Geometry>

namespace ai_sapiens_sim2real
{
// Reflection across the sagittal plane (Y -> -Y). Joint axes follow k1.xml.
class K1Mirror
{
public:
  explicit K1Mirror(const std::vector<std::string> & names)
  {
    for (const auto & name : names) {
      std::string counterpart = name;
      std::string joint = name;
      if (name.rfind("left_", 0) == 0) {
        counterpart = "right_" + name.substr(5);
        joint = name.substr(5);
      } else if (name.rfind("right_", 0) == 0) {
        counterpart = "left_" + name.substr(6);
        joint = name.substr(6);
      } else if (name != "waist_yaw_joint") {
        throw std::runtime_error("K1 mirror does not support joint: " + name);
      }
      const std::vector<std::string> positive = {
        "hip_pitch_joint", "knee_joint", "ankle_pitch_joint", "shoulder_pitch_joint",
        "elbow_joint"};
      const std::vector<std::string> negative = {
        "hip_roll_joint", "hip_yaw_joint", "ankle_roll_joint", "waist_yaw_joint",
        "shoulder_roll_joint", "shoulder_yaw_joint", "wrist_roll_joint"};
      const bool is_positive = std::find(positive.begin(), positive.end(), joint) != positive.end();
      if (!is_positive && std::find(negative.begin(), negative.end(), joint) == negative.end()) {
        throw std::runtime_error("K1 mirror does not support joint: " + name);
      }
      const auto it = std::find(names.begin(), names.end(), counterpart);
      if (it == names.end()) {
        throw std::runtime_error("K1 mirror requires counterpart: " + counterpart);
      }
      indices_.push_back(static_cast<size_t>(it - names.begin()));
      signs_.push_back(is_positive ? 1.0f : -1.0f);
    }
  }

  size_t source(size_t index) const {return indices_.at(index);}
  float sign(size_t index) const {return signs_.at(index);}

  template<typename Vector>
  Vector reflect(const Vector & values, bool is_signed = true) const
  {
    if (static_cast<size_t>(values.size()) != indices_.size()) {
      throw std::runtime_error("K1 mirror joint vector size mismatch");
    }
    Vector result = values;
    for (size_t i = 0; i < indices_.size(); ++i) {
      result[i] = values[indices_[i]] * (is_signed ? signs_[i] : 1.0f);
    }
    return result;
  }

  static Eigen::Quaternionf orientation(const Eigen::Quaternionf & q)
  {
    return {q.w(), -q.x(), q.y(), -q.z()};
  }

private:
  std::vector<size_t> indices_;
  std::vector<float> signs_;
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__K1_MIRROR_HPP_
