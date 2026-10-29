// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "ai_sapiens_sim2real/modules/locomotion_mimic/gate.hpp"

namespace
{

ai_sapiens_sim2real::locomotion_mimic::Gate make_gate(bool singleton_loop, bool interactive = false)
{
  ai_sapiens_sim2real::locomotion_mimic::Parameters params;
  params.quiet_seconds = 0.f;
  params.pending_timeout_seconds = 0.f;
  params.ramp_seconds = 0.02f;
  params.bridge_min_seconds = 0.02f;
  params.bridge_max_seconds = 0.02f;
  params.abort_on_release = true;
  params.hold_frames = {2};
  if (singleton_loop) {params.loops = {{0}};}

  ai_sapiens_sim2real::locomotion_mimic::Clip clip;
  for (int i = 0; i < 5; ++i) {
    clip.q.push_back(ai_sapiens_sim2real::locomotion_mimic::Vec::Zero());
    clip.dq.push_back(ai_sapiens_sim2real::locomotion_mimic::Vec::Zero());
    clip.root.push_back(Eigen::Quaternionf::Identity());
    clip.torso_tilt.push_back(0.f);
  }
  if (interactive) {
    params.hold_frames = {-1, -1};
    params.inter_clip = 0;
    params.inter_hold_frame = 2;
    params.inter_low_clip = 1;
    params.inter_low_frame = 3;
    params.confirm_is_trigger = true;
    return {params, {clip, clip}};
  }
  return {params, {clip}};
}

void reach_hold(
  ai_sapiens_sim2real::locomotion_mimic::Gate & gate,
  ai_sapiens_sim2real::locomotion_mimic::Inputs & input)
{
  gate.step(input);  // establish the initial switch positions
  input.sd = true;
  for (int i = 0; i < 100 && !gate.status().is_holding; ++i) {
    gate.step(input);
  }
  ASSERT_TRUE(gate.status().is_holding);
  ASSERT_EQ(gate.status().state, ai_sapiens_sim2real::locomotion_mimic::CLIP);
  ASSERT_EQ(gate.status().frame, 2);
}

TEST(MimicGateHold, SdReleaseResumesTheRemainingFrames)
{
  auto gate = make_gate(false);
  ai_sapiens_sim2real::locomotion_mimic::Inputs input;
  input.dial = 0;
  reach_hold(gate, input);
  for (int i = 0; i < 5; ++i) {
    gate.step(input);
    EXPECT_EQ(gate.status().frame, 2);
  }
  input.sd = false;
  gate.step(input);
  EXPECT_EQ(gate.status().state, ai_sapiens_sim2real::locomotion_mimic::CLIP);
  EXPECT_EQ(gate.status().frame, 3);
  for (int i = 0; i < 100 && gate.status().state != ai_sapiens_sim2real::locomotion_mimic::WALK;
    ++i)
  {
    gate.step(input);
  }
  EXPECT_EQ(gate.status().state, ai_sapiens_sim2real::locomotion_mimic::WALK);
}

TEST(MimicGateHold, ConfirmResumesSingletonSlotWithoutRepeating)
{
  auto gate = make_gate(true);
  ai_sapiens_sim2real::locomotion_mimic::Inputs input;
  input.loop = 0;
  reach_hold(gate, input);
  EXPECT_TRUE(gate.status().is_loop_stopping);
  input.confirm = true;
  gate.step(input);
  EXPECT_EQ(gate.status().frame, 3);
  for (int i = 0; i < 100 && gate.status().state != ai_sapiens_sim2real::locomotion_mimic::WALK;
    ++i)
  {
    gate.step(input);
  }
  EXPECT_EQ(gate.status().state, ai_sapiens_sim2real::locomotion_mimic::WALK);
  for (int i = 0; i < 5; ++i) {
    gate.step(input);
    EXPECT_EQ(gate.status().state, ai_sapiens_sim2real::locomotion_mimic::WALK);
  }
}

TEST(MimicGateHold, TriggerAndConfirmCanShareTheSdSwitch)
{
  auto gate = make_gate(false, true);

  ai_sapiens_sim2real::locomotion_mimic::Inputs input;
  input.dial = 0;
  input.select = 1;
  gate.step(input);
  input.sd = true;
  input.confirm = true;
  gate.step(input);
  EXPECT_EQ(gate.status().choice, 0);  // Starting the clip must not confirm the handshake.
  for (int i = 0; i < 100 && !gate.status().is_holding; ++i) {
    gate.step(input);
  }
  ASSERT_TRUE(gate.status().is_holding);
  ASSERT_EQ(gate.status().clip, 0);
  input.sd = false;
  input.confirm = false;
  gate.step(input);
  EXPECT_EQ(gate.status().state, ai_sapiens_sim2real::locomotion_mimic::BRIDGE_IN);
  EXPECT_EQ(gate.status().clip, 1);
  EXPECT_EQ(gate.status().frame, 3);
  EXPECT_FALSE(gate.status().is_abort_requested);
}

}  // namespace
