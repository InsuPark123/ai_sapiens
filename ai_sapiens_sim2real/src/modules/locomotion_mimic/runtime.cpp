// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "ai_sapiens_sim2real/modules/locomotion_mimic/runtime.hpp"
#include "ai_sapiens_sim2real/policy/torso_orientation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include "ai_sapiens_sim2real/observation/observations.hpp"
namespace ai_sapiens_sim2real
{
namespace
{
void register_observations()
{
  static const bool is_registered = ObservationRegistry::register_observation(
    "mimic_gate", [](const ObservationContext & context, const YAML::Node & params) {
      if (context.overrides) {
        const auto it = context.overrides->terms.find("mimic_gate");
        if (it != context.overrides->terms.end()) {return it->second;}
      }
      const size_t clips = params && params["clips"] ? params["clips"].size() : 0U;
      return std::vector<float>(locomotion_mimic::kTeacherDim + 1U + clips, 0.0f);
    });
  (void)is_registered;
}
}  // namespace

std::unique_ptr<LocomotionMimic> LocomotionMimic::create_if_configured(
  const Sim2RealConfig & config, bool has_reference_motion, bool is_mirrored)
{
  const auto observations = config.observations();
  const auto term = observations["mimic_gate"];
  if (!term || term.IsNull()) {return nullptr;}
  if (config.any2track() || has_reference_motion || is_mirrored || observations["gait_phase"]) {
    throw std::runtime_error("mimic_gate requires a standalone locomotion policy");
  }
  auto module = std::make_unique<LocomotionMimic>(LocomotionMimicConfig::read(
      term, config.path(), config.step_dt(), config.policy_joints()));
  register_observations();
  return module;
}

LocomotionMimic::LocomotionMimic(LocomotionMimicConfig config)
: config_(std::move(config)), gate_(config_.parameters, std::move(config_.clips)),
  walk_last_(locomotion_mimic::kJoints, 0.0f)
{
  reset();
}
void LocomotionMimic::reset()
{
  gate_.reset();
  observations_ = {};
  observations_.terms["mimic_gate"].assign(gate_.observation_size(), 0.0f);
  std::fill(walk_last_.begin(), walk_last_.end(), 0.0f);
  pending_walk_.clear();
  is_walking_clip_active_ = false;
}
void LocomotionMimic::set_observation_layout(size_t size, const std::vector<std::string> & terms)
{
  if (terms.empty() || terms.back() != "mimic_gate" ||
    size < observations_.terms.at("mimic_gate").size())
  {
    throw std::runtime_error("mimic_gate must be the last observation term");
  }
  weight_index_ = size - observations_.terms.at("mimic_gate").size() +
    locomotion_mimic::kTeacherDim;
}
locomotion_mimic::Inputs LocomotionMimic::read_inputs(
  const SharedControlData & shared, const PolicyJointContext & joints) const
{
  const auto & teleop = shared.teleop;
  locomotion_mimic::Inputs in;
  const bool rc = !teleop.unavailable &&
    teleop.rc_valid[static_cast<size_t>(config_.trigger_channel)];
  const uint16_t sd_us = teleop.rc_us[static_cast<size_t>(config_.trigger_channel)];
  in.sd = rc && sd_us >= config_.trigger_min && sd_us <= config_.trigger_max;
  in.dial = teleop.unavailable ? -1 : static_cast<int>(teleop.selector_code) - config_.dial_base;
  // Dial slots immediately after the clip table select a loop.
  if (!config_.dial_clips.empty() && in.dial >= static_cast<int>(config_.dial_clips.size())) {
    const int k = in.dial - static_cast<int>(config_.dial_clips.size());
    if (k < static_cast<int>(gate_.parameters().loops.size())) {in.loop = k;}
  }
  if (!config_.dial_clips.empty() && in.dial >= 0) {
    in.dial = in.dial <
      static_cast<int>(config_.dial_clips.size()) ?
      config_.dial_clips[static_cast<size_t>(in.dial)] :
      -1;
  }
  in.stick = shared.mode.velocity_commands;
  for (int j = 0; j < locomotion_mimic::kJoints; ++j) {
    const auto c =
      static_cast<Eigen::Index>(joints.policy_to_controller[static_cast<size_t>(j)]);
    in.q[j] = shared.sensors.joint_pos[c];
    in.dq[j] = shared.sensors.joint_vel[c];
  }
  in.gyro = shared.sensors.angular_velocity;
  in.torso = torso_orientation_in_world(shared.sensors, joints);
  if (config_.select_channel > 0 && !teleop.unavailable) {
    const auto ch = static_cast<size_t>(config_.select_channel);
    const auto cf = static_cast<size_t>(config_.confirm_channel);
    if (teleop.rc_valid[ch]) {
      const int us = teleop.rc_us[ch];
      in.select = us <= config_.select_low_max ? 1 : (us >= config_.select_high_min ? 2 : 0);
    }
    in.confirm = teleop.rc_valid[cf] ? teleop.rc_us[cf] >
      1500 : gate_.confirmation_state();
  } else {
    in.confirm = gate_.confirmation_state();
  }
  return in;
}
Eigen::Vector3f LocomotionMimic::update(
  const SharedControlData & shared,
  const PolicyJointContext & joints, const ActionProperties & actions)
{
  const auto input = read_inputs(shared, joints);
  locomotion_mimic::Vec defaults;
  for (size_t i = 0; i < joints.policy_to_controller.size(); ++i) {
    defaults[i] = shared.output.default_joint_pos[joints.policy_to_controller[i]];
  }
  gate_.step(input);
  gate_.observation(input, defaults, shared.policy.last_action.data(),
      observations_.terms.at("mimic_gate").data());
  update_walking_observations(actions, defaults);
  return gate_.command();
}
void LocomotionMimic::update_walking_observations(
  const ActionProperties & actions, const locomotion_mimic::Vec & defaults)
{
  // Walking inference sees its own previous arm command while an upper-body clip plays.
  is_walking_clip_active_ = gate_.walking_clip_active();
  if (is_walking_clip_active_) {
    constexpr size_t n = locomotion_mimic::kJoints;
    const auto & ap = actions;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    observations_.joint_position.assign(n, nan);
    observations_.joint_velocity.assign(n, nan);
    observations_.last_action.assign(n, nan);
    for (size_t j = 13; j < n; ++j) {
      observations_.joint_position[j] = ap.offset[j] + ap.scale[j] * walk_last_[j] -
        defaults[static_cast<Eigen::Index>(j)];
      observations_.joint_velocity[j] = 0.0f;
      observations_.last_action[j] = walk_last_[j];
    }
  } else if (!observations_.joint_position.empty()) {
    observations_.joint_position.clear();
    observations_.joint_velocity.clear();
    observations_.last_action.clear();
  }
}
std::vector<float> LocomotionMimic::infer(const std::vector<float> & observation, const Infer & run)
{
  pending_walk_.clear();
  auto action = run(observation);
  if (action.size() != walk_last_.size()) {
    throw std::runtime_error("mimic_gate ONNX action size changed during inference");
  }
  if (is_walking_clip_active_) {
    auto walk_observation = observation;
    walk_observation.at(weight_index_) = 0.0f;
    const auto walk_action = run(walk_observation);
    if (walk_action.size() != action.size()) {
      throw std::runtime_error("mimic_gate ONNX action size changed during inference");
    }
    for (size_t j = 0; j < 13; ++j) {
      action[j] = walk_action[j] + (1.0f - gate_.leg_handoff()) * (action[j] - walk_action[j]);
    }
    pending_walk_ = walk_action;
  } else {
    pending_walk_ = action;
  }
  // The walking-only arms may not reach the blended target, but become future
  // observations. Do not let a nonfinite hidden output poison that feedback.
  if (!std::all_of(pending_walk_.begin(), pending_walk_.end(),
    [](float value) {return std::isfinite(value);}))
  {
    throw std::runtime_error("mimic_gate nonfinite walking feedback");
  }
  return action;
}

std::vector<bool> LocomotionMimic::evaluate_raw_action_boundaries(
  const std::vector<float> & observation, const std::vector<float> & action,
  float limit, const Infer & run) const
{
  std::vector<bool> reached(action.size(), false);

  if (!has_motion_weight()) {
    return reached;
  }

  auto mimic_action = action;

  if (observation.at(weight_index_) < 1.f || is_walking_clip_active_) {
    // The exported weight only mixes the two branches. Inspect the same mimic
    // branch at weight=1 without committing targets or private walking feedback.
    auto mimic_observation = observation;
    mimic_observation.at(weight_index_) = 1.f;
    mimic_action = run(mimic_observation);
  }

  if (mimic_action.size() != action.size() ||
    !std::all_of(mimic_action.begin(), mimic_action.end(),
    [](float value) {return std::isfinite(value);}))
  {
    throw std::runtime_error("Invalid mimic branch output during action clip check");
  }

  for (size_t j = 0; j < action.size(); ++j) {
    const bool contributes = !is_walking_clip_active_ || j >= 13 || gate_.leg_handoff() < 1.f;
    reached[j] = contributes && std::abs(mimic_action[j]) >= limit;
  }

  return reached;
}

void LocomotionMimic::accept() noexcept
{
  if (!pending_walk_.empty()) {walk_last_.swap(pending_walk_);}
  pending_walk_.clear();
}
void LocomotionMimic::reject() noexcept
{
  pending_walk_.clear();
}
}  // namespace ai_sapiens_sim2real
