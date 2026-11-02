// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#ifndef AI_SAPIENS_SIM2REAL__POLICY__POLICY_ACTION_RESULT_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__POLICY_ACTION_RESULT_HPP_

#include <string>
#include <vector>

namespace ai_sapiens_sim2real
{
enum class PolicyActionStatus {Accepted, ActionRejected, InferenceFailed};

// Evaluation is separate from committing targets, feedback and retry state.
struct PolicyActionResult
{
  PolicyActionStatus status{PolicyActionStatus::InferenceFailed};
  std::vector<float> raw, processed;
  std::string error;
  bool is_accepted() const {return status == PolicyActionStatus::Accepted;}
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__POLICY_ACTION_RESULT_HPP_
