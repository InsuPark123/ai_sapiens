// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

namespace ai_sapiens_sim2real
{
struct ActionSafetyLimits
{
  float absolute_position_rad{2.f * 3.14159265f};
  float delta_rad{3.14159265f / 2.f};
  // Detection only, including for Mimic exports without an actual Clip operation.
  float raw_action_boundary{10.f};
};

enum class ActionRejection {None, Nonfinite, AbsoluteLimit, InvalidReference, BoundaryAndDelta};
enum class BoundaryCheckState {Disabled, NotNeeded, Required, Completed, SkippedInvalidAction};

struct JointActionCheck
{
  std::optional<float> reference;
  float delta{0.f};
  bool has_large_delta{false};
  bool is_at_raw_action_boundary{false};
  ActionRejection rejection{ActionRejection::None};
};

struct ActionValidation
{
  BoundaryCheckState boundary_check{BoundaryCheckState::Disabled};
  std::vector<JointActionCheck> joints;

  bool is_delta_enabled() const {return boundary_check != BoundaryCheckState::Disabled;}
  bool needs_boundary_evaluation() const {return boundary_check == BoundaryCheckState::Required;}

  bool has_rejections() const
  {
    return std::any_of(joints.begin(), joints.end(), [](const auto & joint) {
      return joint.rejection != ActionRejection::None;
    });
  }

  bool is_valid() const {return !needs_boundary_evaluation() && !has_rejections();}
};

// A null reference explicitly disables the delta rule for this inference.
inline ActionValidation validate_action_basics(
  const std::vector<float> & processed, const std::vector<float> * reference,
  const ActionSafetyLimits & limits)
{
  if (reference && reference->size() != processed.size()) {
    throw std::runtime_error("Action reference size does not match processed action");
  }

  ActionValidation result;
  result.joints.resize(processed.size());
  bool has_large_delta = false;

  for (size_t j = 0; j < processed.size(); ++j) {
    auto & check = result.joints[j];

    if (reference) {
      check.reference = (*reference)[j];
      check.delta = processed[j] - *check.reference;
      check.has_large_delta = std::abs(check.delta) >= limits.delta_rad;
      has_large_delta |= check.has_large_delta;
    }

    if (!std::isfinite(processed[j])) {
      check.rejection = ActionRejection::Nonfinite;
    } else if (std::abs(processed[j]) > limits.absolute_position_rad) {
      check.rejection = ActionRejection::AbsoluteLimit;
    } else if (check.reference && !std::isfinite(*check.reference)) {
      check.rejection = ActionRejection::InvalidReference;
    }
  }

  if (reference) {
    if (result.has_rejections()) {
      result.boundary_check = BoundaryCheckState::SkippedInvalidAction;
    } else if (has_large_delta) {
      result.boundary_check = BoundaryCheckState::Required;
    } else {
      result.boundary_check = BoundaryCheckState::NotNeeded;
    }
  }

  return result;
}

inline void complete_action_boundary_check(
  ActionValidation & validation, const std::vector<bool> & is_at_boundary)
{
  if (!validation.needs_boundary_evaluation() || is_at_boundary.size() != validation.joints.size()) {
    throw std::runtime_error("Invalid action boundary evaluation state or size");
  }

  for (size_t j = 0; j < validation.joints.size(); ++j) {
    auto & check = validation.joints[j];
    check.is_at_raw_action_boundary = is_at_boundary[j];

    if (check.is_at_raw_action_boundary && check.has_large_delta) {
      check.rejection = ActionRejection::BoundaryAndDelta;
    }
  }

  validation.boundary_check = BoundaryCheckState::Completed;
}
}  // namespace ai_sapiens_sim2real
