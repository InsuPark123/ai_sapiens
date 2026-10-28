// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/observation/observation_manager.hpp"
#include "ai_sapiens_sim2real/policy/gait_clock.hpp"
#include "ai_sapiens_sim2real/observation/observations.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

using ai_sapiens_sim2real::make_gait_clock;
using ai_sapiens_sim2real::ObservationContext;
using ai_sapiens_sim2real::ObservationManager;
using ai_sapiens_sim2real::ObservationRegistry;
using ai_sapiens_sim2real::PolicyJointContext;
using ai_sapiens_sim2real::SharedControlData;

TEST(GaitObservation, PreservesLegacyAndDwaqOrderingAndStandBehavior)
{
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  auto observe = ObservationRegistry::get_registry().at("gait_phase");
  auto params = YAML::Load("{period: 0.8}");
  EXPECT_EQ(observe(context, params), (std::vector<float>{0, 0}));
  shared.mode.velocity_commands.x() = .2;
  shared.policy.phase_state.phase = .25;
  auto obs = observe(context, params);
  ASSERT_EQ(obs.size(), 2u);
  EXPECT_NEAR(obs[0], 1, 1e-6);
  EXPECT_NEAR(obs[1], 0, 1e-6);
  params["offset"] = .5;
  params["command_threshold"] = .1;
  obs = observe(context, params);
  ASSERT_EQ(obs.size(), 4u);
  EXPECT_NEAR(obs[0], 1, 1e-6);
  EXPECT_NEAR(obs[1], -1, 1e-6);
  EXPECT_NEAR(obs[2], 0, 1e-6);
  EXPECT_NEAR(obs[3], 0, 1e-6);
  shared.mode.velocity_commands.setZero();
  EXPECT_EQ(observe(context, params), (std::vector<float>{0, 0, 1, 1}));
}

TEST(GaitObservation, BilateralPhaseWithSwingAppliesCommandOnlyStandGate)
{
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  shared.mode.velocity_commands = Eigen::Vector3f(.06, 0, .06);
  shared.policy.phase_state.phase = .25f;
  shared.policy.phase_state.swing_ratio = .36f;
  auto observe = ObservationRegistry::get_registry().at("gait_phase");
  auto params = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) +
    "/phase_contract.yaml")["gait_phase"]["params"];
  const auto obs = observe(context, params);
  ASSERT_EQ(obs.size(), 6u);
  EXPECT_NEAR(obs[0], 1, 1e-6);
  EXPECT_NEAR(obs[1], 0, 1e-6);
  EXPECT_NEAR(obs[2], -1, 1e-6);
  EXPECT_NEAR(obs[3], 0, 1e-6);
  EXPECT_FLOAT_EQ(obs[4], .36f);
  EXPECT_FLOAT_EQ(obs[5], .36f);
  for (float x : {0.0f, .1f}) {
    shared.mode.velocity_commands = Eigen::Vector3f(x, 0, 0);
    EXPECT_EQ(observe(context, params), std::vector<float>(6, 0));
  }
}

// Frozen equations from the former combined AMP clock/observation rule.
// Verify entry-history seeding and every output across gait changes and resets.
// The intentionally changed exact lower boundary is covered by the clock tests.
TEST(GaitObservation, AdaptiveMatchesPreviousRuleAwayFromChangedBoundary)
{
  auto config = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) + "/phase_contract.yaml");
  auto clock = make_gait_clock(config, .02);
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  auto observe = ObservationRegistry::get_registry().at("gait_phase");
  float old_phase = 0;
  float old_period = 1.16f;
  float old_swing = .36f;
  bool first = true;
  const auto reference_observation = [&]() {
      std::vector<float> output(6, 0);
      const auto & c = shared.mode.velocity_commands;
      if (!(std::hypot(c.x(), c.y()) + std::abs(c.z()) > .1f)) {
        return output;
      }
      constexpr float two_pi = 6.283185307179586f;
      const float right = std::fmod(old_phase + .5f, 1.0f);
      output = {std::sin(two_pi * old_phase), std::cos(two_pi * old_phase),
        std::sin(two_pi * right), std::cos(two_pi * right), old_swing, old_swing};
      return output;
    };
  const Eigen::Vector3f commands[] = {
    {.2f, 0, 0}, {1.5999f, 0, 0}, {1.6001f, 0, 0}, {2.3999f, 0, 0},
    {2.4f, 0, 0}, {2.8f, 0, 0}, {0, 0, 0}, {.1f, 0, 0},
    {.06f, 0, -.06f}, {-.2f, 0, 0}, {0, .2f, 0}, {0, 0, -.4f}};
  for (int i = 0; i < 2400; ++i) {
    SCOPED_TRACE(i);
    shared.mode.velocity_commands = commands[(i / 50) % 12];
    if (i % 401 == 0) {
      clock->reset(shared.policy.phase_state);
      old_phase = 0;
      old_period = 1.16f;
      old_swing = .36f;
      first = true;
      EXPECT_EQ(observe(context, config["gait_phase"]["params"]), reference_observation());
    }
    if (first) {
      first = false;
    } else {
      const float x = shared.mode.velocity_commands.x();
      const int category = (x > 1.6f) + (x >= 2.4f);
      const float periods[] = {1.16f, .8f, .65f};
      const float swings[] = {.36f, .6f, .6f};
      const float alpha = 1.0f - std::exp(-.02f / .25f);
      old_period += alpha * (periods[category] - old_period);
      old_swing += alpha * (swings[category] - old_swing);
      old_phase = std::fmod(old_phase + .02f / old_period, 1.0f);
    }
    clock->advance(shared.policy.phase_state, shared);
    EXPECT_EQ(observe(context, config["gait_phase"]["params"]), reference_observation());
  }
}

TEST(GaitHistory, PreservesTermMajorFrameMajorAndCurrentCommandReplacement)
{
  ObservationRegistry::register_observation("test_history_signal",
    [](const ObservationContext & context, const YAML::Node &) {
      return std::vector<float>{context.shared.policy.phase_state.phase};
    });
  for (int layout = 0; layout < 3; ++layout) {
    SCOPED_TRACE(layout);
    SharedControlData shared;
    PolicyJointContext joints;
    const auto config = YAML::Load(
      std::string("use_gym_history: ") + (layout ? "true" : "false") +
      "\nreplace_velocity_command_history: " + (layout == 2 ? "true" : "false") + R"(
test_history_signal:
  history_length: 2
velocity_commands:
  history_length: 2
)");
    ObservationManager manager(config, &shared, joints, nullptr);
    EXPECT_EQ(manager.get_observation_size(), 8u);
    shared.policy.phase_state.phase = 1;
    shared.mode.velocity_commands.x() = .2f;
    const std::vector<std::vector<float>> first = {
      {0, 1, 0, 0, 0, .2f, 0, 0},
      {0, 0, 0, 0, 1, .2f, 0, 0},
      {0, .2f, 0, 0, 1, .2f, 0, 0}};
    EXPECT_EQ(manager.compute(), first[layout]);
    shared.policy.phase_state.phase = 2;
    shared.mode.velocity_commands.x() = .4f;
    const std::vector<std::vector<float>> second = {
      {1, 2, .2f, 0, 0, .4f, 0, 0},
      {1, .2f, 0, 0, 2, .4f, 0, 0},
      {1, .4f, 0, 0, 2, .4f, 0, 0}};
    EXPECT_EQ(manager.compute(), second[layout]);
    manager.reset();
    const std::vector<float> reset = layout == 0 ?
      std::vector<float>{2, 2, .4f, 0, 0, .4f, 0, 0} :
      std::vector<float>{2, .4f, 0, 0, 2, .4f, 0, 0};
    EXPECT_EQ(manager.compute(), reset);
  }
}

TEST(GaitHistory, PreservesInvalidExportRejection)
{
  SharedControlData shared;
  PolicyJointContext joints;
  for (const auto * yaml : {
      "replace_velocity_command_history: true\nvelocity_commands: {history_length: 2}",
      "use_gym_history: true\nvelocity_commands: {history_length: 2}\n"
      "gait_phase: {history_length: 3, params: {period: 0.8}}",
      "use_gym_history: true\nreplace_velocity_command_history: true\n"
      "gait_phase: {history_length: 2, params: {period: 0.8}}"})
  {
    SCOPED_TRACE(yaml);
    EXPECT_THROW(ObservationManager(YAML::Load(yaml), &shared, joints, nullptr), std::runtime_error);
  }
}

TEST(GaitObservation, AdaptiveUsesYamlOffsetsAndStandThreshold)
{
  auto params = YAML::LoadFile(std::string(PHASE_TEST_DATA_DIR) +
    "/phase_contract.yaml")["gait_phase"]["params"];
  params.remove("schema");
  params["offsets"] = YAML::Load("[0.25, 0.75]");
  params["stand_command_norm_plus_yaw_threshold"] = .2;
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  auto observe = ObservationRegistry::get_registry().at("gait_phase");
  shared.mode.velocity_commands.x() = .15;
  EXPECT_EQ(observe(context, params), std::vector<float>(6, 0));
  shared.mode.velocity_commands.x() = .3;
  shared.policy.phase_state.swing_ratio = .4;
  const auto result = observe(context, params);
  ASSERT_EQ(result.size(), 6u);
  EXPECT_NEAR(result[0], 1, 1e-6);
  EXPECT_NEAR(result[1], 0, 1e-6);
  EXPECT_NEAR(result[2], -1, 1e-6);
  EXPECT_NEAR(result[3], 0, 1e-6);
  EXPECT_FLOAT_EQ(result[4], .4f);
}

TEST(GaitObservation, AdaptiveMatchesTrainingFixtureAcrossLowAndForwardRanges)
{
  const std::string directory = std::string(PHASE_TEST_DATA_DIR) + "/../adaptive_phase/";
  auto config = YAML::LoadFile(directory + "phase_contract.yaml");
  auto clock = make_gait_clock(config, .02);
  SharedControlData shared;
  PolicyJointContext joints;
  ObservationContext context{shared, joints};
  const auto observe = ObservationRegistry::get_registry().at("gait_phase");
  clock->reset(shared.policy.phase_state);
  std::ifstream fixture(directory + "clock_fixture.txt");
  ASSERT_TRUE(fixture.is_open());
  float x, y, yaw;
  int rows = 0;
  while (fixture >> x >> y >> yaw) {
    std::vector<float> expected(6);
    for (float & value : expected) {ASSERT_TRUE(static_cast<bool>(fixture >> value));}
    shared.mode.velocity_commands = Eigen::Vector3f(x, y, yaw);
    clock->advance(shared.policy.phase_state, shared);
    const auto actual = observe(context, config["gait_phase"]["params"]);
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < actual.size(); ++i) {
      EXPECT_NEAR(actual[i], expected[i], 2e-4f) << "row=" << rows << " channel=" << i;
    }
    ++rows;
  }
  EXPECT_EQ(rows, 4480);
}
