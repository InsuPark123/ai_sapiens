// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include "ai_sapiens_sim2real/modules/locomotion_mimic/gate.hpp"
namespace ai_sapiens_sim2real
{
struct LocomotionMimicConfig
{
  locomotion_mimic::Parameters parameters;
  std::vector<locomotion_mimic::Clip> clips;
  std::vector<std::string> clip_names;
  std::vector<int> dial_clips;
  int trigger_channel{0}, trigger_min{0}, trigger_max{0}, dial_base{0};
  int select_channel{0}, select_low_max{0}, select_high_min{0}, confirm_channel{0};
  static LocomotionMimicConfig read(
    const YAML::Node & term,
    const std::filesystem::path & policy_config, double step_dt,
    const std::vector<std::string> & joints);
};
}  // namespace ai_sapiens_sim2real
