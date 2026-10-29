// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <functional>
#include <memory>
#include "ai_sapiens_sim2real/modules/locomotion_mimic/config.hpp"
#include "ai_sapiens_sim2real/config/sim2real_config.hpp"
#include "ai_sapiens_sim2real/observation/policy_observation_overrides.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"
namespace ai_sapiens_sim2real
{
// Policy-local motion selection and walk/upper-body composition. No shared
// history/masks or actuator delivery; PolicyRuntime retains those lifecycles.
class LocomotionMimic
{
public:
  static std::unique_ptr<LocomotionMimic> create_if_configured(
    const Sim2RealConfig & config, bool has_reference_motion, bool is_mirrored);
  explicit LocomotionMimic(LocomotionMimicConfig config);
  void reset();
  void set_observation_layout(size_t size, const std::vector<std::string> & terms);
  Eigen::Vector3f update(
    const SharedControlData & shared, const PolicyJointContext & joints,
    const ActionProperties & actions);
  const PolicyObservationOverrides & observations() const {return observations_;}
  using Infer = std::function<std::vector<float>(const std::vector<float> &)>;
  std::vector<float> infer(const std::vector<float> & observation, const Infer & run);

private:
  locomotion_mimic::Inputs read_inputs(
    const SharedControlData & shared,
    const PolicyJointContext & joints) const;
  void update_walking_observations(
    const ActionProperties & actions,
    const locomotion_mimic::Vec & defaults);
  LocomotionMimicConfig config_;
  locomotion_mimic::Gate gate_;
  PolicyObservationOverrides observations_;
  std::vector<float> walk_last_;
  size_t weight_index_{0};
  bool is_walking_clip_active_{false};
};
}  // namespace ai_sapiens_sim2real
