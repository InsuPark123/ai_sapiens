// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <vector>
#include <string>
#include <unordered_map>
namespace ai_sapiens_sim2real
{
struct PolicyObservationOverrides
{
  std::vector<float> joint_position, joint_velocity, last_action;
  std::unordered_map<std::string, std::vector<float>> terms;
  // Aliases share the same raw values; correction precedes history, scale and clip.
  void apply(const std::string & name, std::vector<float> & values) const
  {
    if (name == "joint_pos" || name == "joint_pos_rel") {
      apply(values, joint_position);
    } else if (name == "joint_vel" || name == "joint_vel_rel") {
      apply(values, joint_velocity);
    } else if (name == "actions" || name == "last_action") {apply(values, last_action);}
  }

private:
  static void apply(std::vector<float> & values, const std::vector<float> & overrides)
  {
    for (size_t i = 0; i < values.size() && i < overrides.size(); ++i) {
      if (!std::isnan(overrides[i])) {values[i] = overrides[i];}
    }
  }
};
}  // namespace ai_sapiens_sim2real
