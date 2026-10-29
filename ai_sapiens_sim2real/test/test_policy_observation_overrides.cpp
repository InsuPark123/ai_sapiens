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
