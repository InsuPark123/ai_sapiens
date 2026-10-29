// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include "data/locomotion_mimic/original_gate.hpp"
#include "ai_sapiens_sim2real/policy/policy_runtime.hpp"
#include "ai_sapiens_sim2real/modules/locomotion_mimic/runtime.hpp"
#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"

using namespace ai_sapiens_sim2real;

TEST(LocomotionMimicAsset, RealExportLoadsAndHandsOffWithoutLeakingOverrides)
{
  const char * directory = std::getenv("LOCOMIMIC_ASSET_DIR");
  if (!directory) {GTEST_SKIP() << "Set LOCOMIMIC_ASSET_DIR to validate an exported policy";}
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  const auto node = std::make_shared<rclcpp::Node>("locomimic_asset_test");
  const auto root = std::filesystem::path(directory);
  const Sim2RealConfig config(root / "params/sim2real.yaml");
  EXPECT_THROW(LocomotionMimic::create_if_configured(config, true, false), std::runtime_error);
  EXPECT_THROW(LocomotionMimic::create_if_configured(config, false, true), std::runtime_error);
  const auto names = config.policy_joints();
  auto resolved = LocomotionMimicConfig::read(config.observations()["mimic_gate"],
      config.path(), config.step_dt(), names);
  ASSERT_EQ(resolved.clips.size(), 24U);
  ASSERT_EQ(resolved.trigger_channel, 8);
  const auto yaml = YAML::LoadFile(config.path().string());
  const auto paths = yaml["observations"]["mimic_gate"]["params"]["clips"];
  for (size_t i = 0; i < resolved.clips.size(); ++i) {
    const auto legacy = k1_mimic_gate::load_clip(
      (root / "params" / paths[i].as<std::string>()).string(), 12, .02f);
    const auto & current = resolved.clips[i];
    ASSERT_EQ(current.frames(), legacy.frames());
    for (int f = 0; f < current.frames(); ++f) {
      ASSERT_TRUE(current.q[f] == legacy.q[f]);
      ASSERT_TRUE(current.dq[f] == legacy.dq[f]);
      ASSERT_TRUE(current.root[f].coeffs() == legacy.root[f].coeffs());
      ASSERT_EQ(current.torso_tilt[f], legacy.torso_tilt[f]);
    }
  }
  SharedControlData shared;
  shared.resize(23, 23);
  shared.sensors.is_imu_usable = true;
  for (size_t i = 0; i < names.size(); ++i) {
    shared.sensors.joint_pos[i] = config.joint_properties().default_position[i];
  }
  shared.teleop.unavailable = false;
  shared.teleop.rc_valid[8] = true;
  shared.teleop.rc_us[8] = 1000;
  shared.teleop.selector_code = 202;  // wave_right_hand_only
  shared.mode.velocity_source = VelocitySource::Teleop;
  shared.teleop.velocity_command_normalized.x() = .3f;
  PolicyRuntime runtime(node, "loco_mimic", (root / "exported/policy.onnx").string(),
    config, names, &shared);
  ASSERT_EQ(runtime.observation_size(), 539U);
  runtime.enter();
  const auto dt = rclcpp::Duration::from_seconds(.02);
  ASSERT_EQ(runtime.update(dt), PolicyUpdateResult::TargetReady);
  EXPECT_FLOAT_EQ(shared.policy.input[514], 0.f);
  shared.teleop.rc_us[8] = 2000;
  for (int tick = 0; tick < 200; ++tick) {
    ASSERT_EQ(runtime.update(dt), PolicyUpdateResult::TargetReady) << tick;
    for (float q : runtime.target_command().position) {
      ASSERT_TRUE(std::isfinite(q));
    }
  }
  EXPECT_GT(shared.policy.input[514], 0.f);
  shared.teleop.rc_us[8] = 1000;
  for (int tick = 0; tick < 300; ++tick) {
    ASSERT_EQ(runtime.update(dt), PolicyUpdateResult::TargetReady);
  }
  EXPECT_FLOAT_EQ(shared.policy.input[514], 0.f);
  // A fresh runtime has its own view even when the preceding policy was mid-clip.
  shared.teleop.rc_us[8] = 2000;
  ASSERT_EQ(runtime.update(dt), PolicyUpdateResult::TargetReady);
  // Existing tiny normal-policy fixture has a different joint count and ABI.
  const std::filesystem::path fixture(COMMON_POLICY_TEST_DATA_DIR);
  Sim2RealConfig normal_config(fixture / "sim2real.yaml");
  EXPECT_EQ(LocomotionMimic::create_if_configured(normal_config, false, false), nullptr);
  PolicyRuntime normal(node, "velocity", (fixture / "policy.onnx").string(),
    normal_config, names, &shared);
  normal.enter();
  ASSERT_EQ(normal.update(dt), PolicyUpdateResult::TargetReady);
  EXPECT_FLOAT_EQ(shared.policy.input[0], shared.sensors.joint_pos[1] - .1f);
  EXPECT_FLOAT_EQ(shared.policy.input[1], shared.sensors.joint_pos[7] + .2f);
  runtime.enter();
  ASSERT_EQ(runtime.update(dt), PolicyUpdateResult::TargetReady);
  EXPECT_FLOAT_EQ(shared.policy.input[514], 0.f);  // held trigger cannot restart on entry
}
