// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "ai_sapiens_sim2real/observation/observation_manager.hpp"
#include "ai_sapiens_sim2real/observation/policy_observation_overrides.hpp"

using namespace ai_sapiens_sim2real;

TEST(PolicyObservationOverrides, PreservesAliasesHistoryAndProcessingOrder)
{
  for (const bool is_scale_first : {false, true}) {
    for (const bool is_frame_major : {false, true}) {
      SharedControlData shared;
      shared.resize(2, 2);
      shared.sensors.joint_pos.setConstant(.25f);
      shared.sensors.joint_vel.setConstant(.25f);
      shared.policy.last_action.assign(2, .25f);
      PolicyJointContext joints;
      joints.policy_joint_names = {"a", "b"};
      joints.policy_to_controller = {0, 1};
      auto config =
        YAML::Load(
        R"(
joint_pos: {history_length: 2, scale: [3], clip: [-1, 1]}
joint_pos_rel: {history_length: 2, scale: [3], clip: [-1, 1]}
joint_vel: {history_length: 2, scale: [3], clip: [-1, 1]}
joint_vel_rel: {history_length: 2, scale: [3], clip: [-1, 1]}
actions: {history_length: 2, scale: [3], clip: [-1, 1]}
last_action: {history_length: 2, scale: [3], clip: [-1, 1]}
)");
      config["scale_first"] = is_scale_first;
      config["use_gym_history"] = is_frame_major;
      PolicyObservationOverrides overrides;
      const float nan = std::numeric_limits<float>::quiet_NaN();
      overrides.joint_position = overrides.joint_velocity = overrides.last_action = {2.f, nan};
      ObservationManager manager(config, &shared, joints, nullptr, nullptr, nullptr, &overrides);
      // The old frame was seeded at construction. Only the new frame changes.
      overrides.joint_position[0] = overrides.joint_velocity[0] = overrides.last_action[0] = -.5f;
      const auto actual = manager.compute();
      ASSERT_EQ(actual.size(), 24U);
      const float initial = is_scale_first ? 1.f : 3.f;
      const float current = is_scale_first ? -1.f : -1.5f;
      for (size_t term = 0; term < 6; ++term) {
        for (size_t frame = 0; frame < 2; ++frame) {
          const size_t index = is_frame_major ? (frame * 6 + term) * 2 : term * 4 + frame * 2;
          EXPECT_FLOAT_EQ(actual[index], frame == 0 ? initial : current);
          EXPECT_FLOAT_EQ(actual[index + 1], .75f);  // NaN preserves the raw value.
        }
      }
      manager.reset();
      const auto reset = manager.compute();
      for (size_t i = 0; i < reset.size(); i += 2) {
        EXPECT_FLOAT_EQ(reset[i], current);
        EXPECT_FLOAT_EQ(reset[i + 1], .75f);
      }
      // Releasing the overrides restores the ordinary path, including reset history.
      overrides = {};
      manager.reset();
      ObservationManager ordinary(config, &shared, joints, nullptr);
      EXPECT_EQ(manager.compute(), ordinary.compute());
      EXPECT_FLOAT_EQ(shared.sensors.joint_pos[0], .25f);
      EXPECT_FLOAT_EQ(shared.policy.last_action[0], .25f);
    }
  }
}

TEST(PolicyObservationHistory, RejectsOnlyLatestSampleWithoutRescalingOrResettingHistory)
{
  for (bool is_frame_major : {false, true}) {
    for (int history_length : {1, 4}) {
      for (int failure_count : {1, 2}) {
        SharedControlData shared;
        shared.resize(1, 1);
        PolicyJointContext joints;
        joints.policy_joint_names = {"joint"};
        joints.policy_to_controller = {0};
        auto config = YAML::Load(
          "joint_pos: {scale: [2]}\njoint_vel: {scale: [3]}\n");
        config["joint_pos"]["history_length"] = history_length;
        config["joint_vel"]["history_length"] = history_length;
        config["use_gym_history"] = is_frame_major;
        ObservationManager manager(config, &shared, joints, nullptr);
        for (int value = 1; value <= 3; ++value) {
          shared.sensors.joint_pos[0] = value;
          shared.sensors.joint_vel[0] = value * 10;
          manager.compute();
        }
        for (int failure = 0; failure < failure_count; ++failure) {
          shared.sensors.joint_pos[0] = 999;
          shared.sensors.joint_vel[0] = 999;
          manager.compute();
          manager.reject_latest();
        }
        shared.sensors.joint_pos[0] = 4;
        shared.sensors.joint_vel[0] = 40;
        const auto observation = manager.compute();
        ASSERT_EQ(observation.size(), static_cast<size_t>(history_length * 2));
        for (int frame = 0; frame < history_length; ++frame) {
          const float expected = frame == history_length - 1 ? 4 :
            (failure_count == 1 && frame == 0 ? 2 : 3);
          const int position = is_frame_major ? frame * 2 : frame;
          const int velocity = is_frame_major ? frame * 2 + 1 : history_length + frame;
          EXPECT_FLOAT_EQ(observation[position], expected * 2);
          EXPECT_FLOAT_EQ(observation[velocity], expected * 30);
        }
      }
    }
  }
}

TEST(PolicyObservationHistory, FirstRejectedFrameUsesEntrySeedAndResetClearsFallback)
{
  ObservationTermCfg term;
  term.history_length = 1;
  term.scale = {2};
  term.reset({3});
  term.add({999});
  term.reject_latest();
  EXPECT_EQ(term.get(), std::vector<float>({6}));
  term.add({999});
  term.reject_latest();
  EXPECT_EQ(term.get(), std::vector<float>({6}));
  term.reset({4});
  term.add({999});
  term.reject_latest();
  EXPECT_EQ(term.get(), std::vector<float>({8}));
}

TEST(PolicyObservationHistory, DiagnosticLayoutReadsActualTensorIncludingReplacedCommands)
{
  for (const bool frame_major : {false, true}) {
    SharedControlData shared;
    shared.resize(1, 1);
    PolicyJointContext joints;
    joints.policy_joint_names = {"joint"};
    joints.policy_to_controller = {0};
    auto config = YAML::Load(
      "joint_pos: {scale: [2], history_length: 3}\n"
      "velocity_commands: {history_length: 3}\n");
    config["use_gym_history"] = frame_major;
    config["replace_velocity_command_history"] = frame_major;
    ObservationManager manager(config, &shared, joints, nullptr);
    std::vector<float> input;
    for (int frame = 1; frame <= 3; ++frame) {
      shared.sensors.joint_pos[0] = frame;
      shared.mode.velocity_commands.setConstant(frame);
      input = manager.compute();
    }
    const auto layout = manager.layout();
    ASSERT_EQ(layout.size(), 2U);
    for (size_t frame = 0; frame < 3; ++frame) {
      EXPECT_FLOAT_EQ(input.at(layout[0].offset + frame * layout[0].frame_stride), 2 * (frame + 1));
      for (size_t channel = 0; channel < 3; ++channel) {
        EXPECT_FLOAT_EQ(input.at(layout[1].offset + frame * layout[1].frame_stride + channel),
          frame_major ? 3 : frame + 1);
      }
    }
  }
}
