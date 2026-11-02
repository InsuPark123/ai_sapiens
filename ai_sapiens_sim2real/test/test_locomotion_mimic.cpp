// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <set>
#include "data/locomotion_mimic/original_gate.hpp"
#include "ai_sapiens_sim2real/modules/locomotion_mimic/runtime.hpp"
#include "ai_sapiens_sim2real/observation/observation_manager.hpp"

namespace current = ai_sapiens_sim2real::locomotion_mimic;
namespace original = k1_mimic_gate;
using namespace ai_sapiens_sim2real;

namespace
{
template<class Clip>
Clip make_clip(int offset)
{
  Clip result;
  for (int f = 0; f < 90; ++f) {
    current::Vec q;
    for (int j = 0; j < 23; ++j) {
      q[j] = .1f * std::sin(.1f * f + .2f * j + offset);
    }
    result.q.push_back(q);
    result.dq.push_back(current::Vec::Constant(.03f));
    result.root.push_back(Eigen::Quaternionf(Eigen::AngleAxisf(.1f * std::sin(.05f * f),
      Eigen::Vector3f::UnitX())));
    result.torso_tilt.push_back(f > 30 && f < 45 ? .8f : .1f);
  }
  return result;
}
template<class Parameters>
Parameters parameters(bool abort_on_release)
{
  Parameters p;
  p.abort_on_release = abort_on_release;
  p.quiet_seconds = .1f;
  p.pending_timeout_seconds = .3f;
  p.ramp_seconds = .1f;
  p.bridge_min_seconds = .06f;
  p.bridge_max_seconds = .2f;
  p.fast_ramp_seconds = .04f;
  p.walk_allowed = {false, true, true, false};
  p.arms_only_ref = {false, true, false, false};
  p.fast_start = {false, false, true, false};
  p.hold_frames = {-1, -1, -1, 40};
  p.hold_max_seconds = .5f;
  p.switch_walk_seconds = .4f;
  p.inter_clip = 1;
  p.inter_hold_frame = 30;
  p.inter_low_clip = 2;
  p.inter_low_frame = 35;
  p.loops = {{0, 1, 2}, {3}};
  return p;
}
}

TEST(LocomotionMimic, StateAndObservationParityWithRobotStash)
{
  std::set<int> visited;
  bool saw_hold = false, saw_walk_clip = false, saw_handoff = false;
  for (bool release : {false, true}) {
    for (bool shared_switch : {false, true}) {
      auto p = parameters<current::Parameters>(release);
      auto old_p = parameters<original::Parameters>(release);
      p.confirm_is_trigger = old_p.confirm_is_trigger = shared_switch;
      current::Gate gate(p, {make_clip<current::Clip>(0), make_clip<current::Clip>(1),
          make_clip<current::Clip>(2), make_clip<current::Clip>(3)});
      original::Gate oracle(old_p, {make_clip<original::Clip>(0), make_clip<original::Clip>(1),
          make_clip<original::Clip>(2), make_clip<original::Clip>(3)});
      std::mt19937 random(7231);
      current::Inputs in;
      original::Inputs legacy;
      std::vector<float> last(23, .02f), actual(gate.observation_size()),
      expected(oracle.observation_size());
      for (int tick = 0; tick < 20000; ++tick) {
        if (tick % 1000 == 0) {gate.reset(); oracle.reset();}
        if (tick % 200 == 0) {
          const int slot = random() % 6;
          in.dial = slot < 4 ? slot : -1;
          in.loop = slot >= 4 ? slot - 4 : -1;
        }
        in.sd = tick % 200 >= 3 && tick % 200 < 160;
        in.confirm = shared_switch ? in.sd : tick % 77 > 38;
        in.select = (tick / 90) % 3;
        in.stick = tick % 600 > 250 ? Eigen::Vector3f(.3f, -.1f, .2f) : Eigen::Vector3f::Zero();
        in.q.setConstant(.05f * std::sin(tick * .01f));
        in.dq.setConstant(tick % 80 < 20 ? .5f : 0.0f);
        in.gyro = Eigen::Vector3f(.01f, .02f, .03f);
        in.torso = Eigen::Quaternionf(Eigen::AngleAxisf(.1f, Eigen::Vector3f::UnitZ()));
        legacy.sd = in.sd; legacy.dial = in.dial; legacy.loop = in.loop;
        legacy.confirm = in.confirm; legacy.select = in.select; legacy.stick = in.stick;
        legacy.q = in.q; legacy.dq = in.dq; legacy.gyro = in.gyro; legacy.torso = in.torso;
        gate.step(in); oracle.step(legacy);
        ASSERT_EQ(gate.status().state, oracle.state) << tick;
        ASSERT_EQ(gate.status().clip, oracle.clip) << tick;
        ASSERT_EQ(gate.status().frame, oracle.frame) << tick;
        ASSERT_EQ(gate.status().is_holding, oracle.holding) << tick;
        ASSERT_EQ(gate.status().handoff, oracle.handoff) << tick;
        ASSERT_EQ(gate.status().weight, oracle.w) << tick;
        ASSERT_TRUE(gate.command() == oracle.cmd) << tick;
        gate.observation(in, current::Vec::Zero(), last.data(), actual.data());
        oracle.observation(legacy, original::Vec::Zero(), last.data(), expected.data());
        ASSERT_EQ(actual, expected) << tick;
        visited.insert(gate.status().state);
        saw_hold |= gate.status().is_holding;
        saw_walk_clip |= gate.walking_clip_active();
        saw_handoff |= gate.status().handoff > 0.f && gate.status().handoff < 1.f;
      }
    }
  }
  EXPECT_EQ(visited.size(), 7U);
  EXPECT_TRUE(saw_hold);
  EXPECT_TRUE(saw_walk_clip);
  EXPECT_TRUE(saw_handoff);
}

TEST(LocomotionMimic, MasksAndTwoPassCompositionStayPolicyLocal)
{
  LocomotionMimicConfig config;
  config.parameters = parameters<current::Parameters>(true);
  config.parameters.inter_clip = -1;
  config.clips = {make_clip<current::Clip>(0), make_clip<current::Clip>(1),
    make_clip<current::Clip>(2), make_clip<current::Clip>(3)};
  config.trigger_channel = 8; config.trigger_min = 1950; config.trigger_max = 2050;
  config.dial_base = 200;
  LocomotionMimic module(std::move(config));
  module.set_observation_layout(519, {"joint_pos", "mimic_gate"});
  SharedControlData shared;
  shared.resize(23, 23);
  shared.teleop.unavailable = false;
  shared.teleop.rc_valid[8] = true;
  shared.teleop.selector_code = 201;
  shared.mode.velocity_commands.x() = .5f;
  PolicyJointContext joints;
  for (int i = 0; i < 23; ++i) {
    joints.policy_joint_names.push_back("joint_" + std::to_string(i));
    joints.policy_to_controller.push_back(i);
  }
  ActionProperties actions;
  actions.scale.assign(23, .25f); actions.offset.assign(23, .1f);
  module.update(shared, joints, actions);  // establish switch state
  // Single-pass walking also stages feedback until the target is accepted.
  const std::vector<float> walking_observation(519, 1.f);
  module.infer(walking_observation, [](const auto &) {return std::vector<float>(23, .1f);});
  module.accept();
  module.infer(walking_observation, [](const auto &) {return std::vector<float>(23, 100.f);});
  module.reject();
  shared.teleop.rc_us[8] = 2000;
  module.update(shared, joints, actions);
  EXPECT_FLOAT_EQ(module.observations().last_action[13], .1f);
  std::vector<float> observation(519, 1.f);
  int calls = 0;
  const auto result = module.infer(observation, [&](const auto & input) {
        ++calls;
        return std::vector<float>(23, input.at(514) == 0.f ? .2f : .8f);
    });
  EXPECT_EQ(calls, 2);
  for (int i = 0; i < 23; ++i) {
    EXPECT_FLOAT_EQ(result[i], i < 13 ? .2f : .8f);
  }
  module.accept();
  module.update(shared, joints, actions);
  const auto & view = module.observations();
  ASSERT_EQ(view.joint_position.size(), 23U);
  EXPECT_TRUE(std::isnan(view.joint_position[0]));
  EXPECT_FLOAT_EQ(view.joint_position[13], .15f);
  EXPECT_FLOAT_EQ(view.last_action[13], .2f);
  EXPECT_FLOAT_EQ(shared.sensors.joint_pos[13], 0.f);
  EXPECT_FLOAT_EQ(shared.policy.last_action[13], 0.f);
  // Common target rejection must discard the separate walking inference too.
  for (int attempt = 0; attempt < 5; ++attempt) {
    module.infer(observation, [](const auto &) {return std::vector<float>(23, 100.f);});
    module.reject();
    module.update(shared, joints, actions);
    EXPECT_FLOAT_EQ(view.last_action[13], .2f);
    EXPECT_FLOAT_EQ(view.joint_position[13], .15f);
  }
  calls = 0;
  EXPECT_THROW(module.infer(observation, [&](const auto &) {
    if (++calls == 2) {throw std::runtime_error("second inference failed");}
    return std::vector<float>(23, .9f);
  }), std::runtime_error);
  module.reject();
  module.update(shared, joints, actions);
  EXPECT_FLOAT_EQ(view.last_action[13], .2f);

  // Hidden walking arms can be invalid even when the composed target is finite.
  calls = 0;
  EXPECT_THROW(module.infer(observation, [&](const auto &) {
    std::vector<float> value(23, .3f);
    if (++calls == 2) {value[13] = std::numeric_limits<float>::infinity();}
    return value;
  }), std::runtime_error);
  module.reject();
  module.update(shared, joints, actions);
  EXPECT_FLOAT_EQ(view.last_action[13], .2f);

  module.infer(observation, [](const auto &) {return std::vector<float>(23, .4f);});
  module.accept();
  module.update(shared, joints, actions);
  EXPECT_FLOAT_EQ(view.last_action[13], .4f);
  module.reset();
  EXPECT_TRUE(module.observations().joint_position.empty());
  EXPECT_TRUE(module.observations().last_action.empty());
  EXPECT_EQ(module.observations().terms.at("mimic_gate"), std::vector<float>(129, 0.f));
}
