// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include "ai_sapiens_sim2real/config/any2track_config.hpp"
#include "ai_sapiens_sim2real/policy/any2track_policy.hpp"

using namespace ai_sapiens_sim2real;

namespace
{
class Any2TrackHistory : public ::testing::TestWithParam<bool>
{
protected:
  void SetUp() override
  {
    char temporary[] = "/tmp/any2track-history-XXXXXX";
    const auto directory = mkdtemp(temporary);
    ASSERT_NE(directory, nullptr);
    root = directory;
    const std::vector<std::string> joints{
      "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
      "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",
      "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
      "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint",
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
      "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
      "right_wrist_roll_joint"};
    YAML::Node yaml = YAML::Load(R"(
step_dt: 0.02
history_length: 79
joint_vel_scale: 0.05
commands: {}
orientation_tracking:
  version: 1
  anchor: pelvis
  alignment: initial_yaw
  representation: relative_rotation_first_two_columns_row_major
  quaternion_order: wxyz
  requires_pelvis_orientation_estimate: true
  reference_time_offset_steps: 1
velocity_estimation: {version: 1, history_length: 20}
actions:
  reference_residual: {offset: [], clip: null}
)");
    yaml["runtime_type"] = GetParam() ? "any2track_gt_xy_adapter" : "opentrack_anyadapter";
    if (GetParam()) {
      yaml["path_tracking"] = YAML::Load(R"({version: 1, source: ground_truth, anchor: pelvis,
frame: robot_heading, reference: fixed_motion_world, simulation_only: true,
reference_time_offset_steps: 1, motion_end: truncate})");
    }
    yaml["policy_joints"] = joints;
    yaml["torque_limits"] = std::vector<float>(23, 10.f);
    yaml["actions"]["reference_residual"]["scale"] = std::vector<float>(23, .25f);
    for (const auto & joint : joints) {
      yaml["joint_properties"][joint] = YAML::Load(
        "{default_position: 0.1, stiffness: 10, damping: 1}");
    }
    const auto names = any2track_observation_names(GetParam());
    std::vector<int> dimensions{23, 23, 3, 3, 23, 23, 23, 6, 2, 4, 1};
    if (!GetParam()) {dimensions.erase(dimensions.begin() + 8);}
    for (size_t i = 0; i < names.size(); ++i) {
      auto term = yaml["observations"][names[i]];
      term["dimension"] = dimensions[i];
      term["history_length"] = 1;
      term["clip"] = YAML::Node(YAML::NodeType::Null);
      term["params"] = YAML::Node(YAML::NodeType::Map);
      term["scale"] = std::vector<float>(dimensions[i], 1.f);
    }
    std::ofstream(root / "sim2real.yaml") << yaml;
    {
      std::ofstream motion(root / "motion.csv");
      for (int frame = 0; frame < 16; ++frame) {
        motion << "0,0,0.7,0,0,0,1";
        for (int joint = 0; joint < 23; ++joint) {motion << ',' << .001f * frame;}
        motion << '\n';
      }
    }
    config = std::make_unique<Sim2RealConfig>(root / "sim2real.yaml");
    mimic.motion_file = root / "motion.csv";
    policy = std::make_unique<Any2TrackPolicy>(*config, mimic);
    entry.position.setConstant(.2f);
    entry.velocity.setConstant(.3f);
    entry.gyro << .4f, .5f, .6f;
    policy->reset(entry, true);
  }
  void TearDown() override {if (!root.empty()) {std::filesystem::remove_all(root);}}
  std::filesystem::path root;
  std::unique_ptr<Sim2RealConfig> config;
  MimicBehavior mimic;
  std::unique_ptr<Any2TrackPolicy> policy;
  Any2TrackSample entry;
};
}  // namespace

TEST_P(Any2TrackHistory, FirstRejectedObservationReplacesEveryReplicatedSlot)
{
  Any2TrackPolicy oracle(*config, mimic);
  oracle.reset(entry, true);
  ASSERT_TRUE(oracle.observe(entry));
  const auto expected = oracle.inputs().at("velocity_history");
  const auto adapter = policy->inputs().at("history");
  auto bad = entry;
  bad.position.setConstant(100.f);
  bad.velocity.setConstant(1000.f);
  bad.gyro.setConstant(100.f);
  for (size_t attempt = 0; attempt < 5; ++attempt) {
    ASSERT_TRUE(policy->observe(bad));
    EXPECT_NE(policy->inputs().at("velocity_history"), expected);
    const auto rejected_observation = policy->observation();
    policy->reject_latest();
    EXPECT_EQ(policy->inputs().at("velocity_history"), expected);
    EXPECT_EQ(policy->inputs().at("history"), adapter);
    EXPECT_EQ(policy->observation(), rejected_observation);  // diagnostics retain evidence
    EXPECT_EQ(policy->cursor(), attempt + 1);
    policy->advance();
  }
  ASSERT_TRUE(policy->observe(entry));
  EXPECT_EQ(policy->inputs().at("velocity_history"), expected);
  const auto target = policy->target(std::vector<float>(23, .1f));
  policy->accept(target);
  ASSERT_TRUE(policy->observe(entry));
  for (int j = 0; j < 23; ++j) {EXPECT_EQ(policy->observation()[98 + j], target[j]);}
}

TEST_P(Any2TrackHistory, RejectionPreservesHistoryCadenceAndAcceptedFeedback)
{
  ASSERT_TRUE(policy->observe(entry));
  policy->accept(policy->target(std::vector<float>(23, .1f)));
  policy->advance();
  auto next = entry;
  next.position.setConstant(.4f);
  next.velocity.setConstant(.5f);
  ASSERT_TRUE(policy->observe(next));
  policy->accept(policy->target(std::vector<float>(23, .2f)));
  const auto before = policy->inputs().at("velocity_history");
  const auto adapter = policy->inputs().at("history");
  policy->advance();
  next.position.setConstant(100.f);
  ASSERT_TRUE(policy->observe(next));
  policy->reject_latest();
  const auto & after = policy->inputs().at("velocity_history");
  EXPECT_TRUE(std::equal(before.begin() + 75, before.end(), after.begin()));
  EXPECT_TRUE(std::equal(before.end() - 75, before.end(), after.end() - 75));
  EXPECT_EQ(policy->inputs().at("history"), adapter);
  policy->reject_latest();  // idempotent cleanup must not shift history again
  EXPECT_TRUE(std::equal(before.end() - 75, before.end(), after.end() - 75));
  policy->reset(entry, true);
  ASSERT_TRUE(policy->observe(entry));
  EXPECT_TRUE(std::equal(after.begin(), after.begin() + 75, after.end() - 75));
}

INSTANTIATE_TEST_SUITE_P(AdapterSchemas, Any2TrackHistory, ::testing::Bool());
