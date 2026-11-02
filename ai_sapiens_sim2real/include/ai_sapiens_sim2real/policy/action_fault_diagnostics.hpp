// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#ifndef AI_SAPIENS_SIM2REAL__POLICY__ACTION_FAULT_DIAGNOSTICS_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__ACTION_FAULT_DIAGNOSTICS_HPP_

#include <optional>
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

// Display only: left_/right_ -> L./R., omit the _joint suffix.
std::string compact_joint_name(const std::string & name);

// Physical sensor coordinates, including for mirrored policies. Diagnostic
// jump thresholds never affect action acceptance or failsafe decisions.
std::string format_action_fault_sensors(
  const SensorData & current, const std::optional<SensorData> & previous,
  const std::vector<std::string> & joint_names, double elapsed_seconds, bool jumps_only = false);

std::string format_action_fault_actions(
  const std::vector<float> & processed, const std::vector<std::string> & joints,
  float limit, bool invalid_only = false);

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__ACTION_FAULT_DIAGNOSTICS_HPP_
