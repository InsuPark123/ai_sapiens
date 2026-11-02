// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <filesystem>
#include <cstdlib>
#include <limits>
#include <memory>
#include <sstream>

#include "ai_sapiens_sim2real/command_transition.hpp"
#include "ai_sapiens_sim2real/policy/action_fault_diagnostics.hpp"
#include "ai_sapiens_sim2real/policy/action_fault_recorder.hpp"
#include "ai_sapiens_sim2real/policy/policy_runtime.hpp"

using namespace ai_sapiens_sim2real;

namespace
{
std::string normalized_whitespace(const std::string & value)
{
  std::istringstream input(value);
  std::string word, result;
  while (input >> word) {
    if (!result.empty()) {result += " ";}
    result += word;
  }
  return result;
}

class PolicyActionFault : public ::testing::Test
{
protected:
  virtual std::filesystem::path policy_directory() const {return COMMON_POLICY_TEST_DATA_DIR;}

  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    const auto node = std::make_shared<rclcpp::Node>("policy_action_fault_test");
    const auto directory = policy_directory();
    const Sim2RealConfig config(directory / "sim2real.yaml");
    data.resize(2, 2);
    data.sensors.joint_pos << .4f, -.7f;
    data.sensors.is_imu_usable = true;
    runtime = std::make_unique<PolicyRuntime>(node, "fault_test",
        (directory / "policy.onnx").string(), config, config.policy_joints(), &data);
    runtime->enter();
  }

  PolicyUpdateResult tick(double seconds = .02)
  {
    return runtime->update(rclcpp::Duration::from_seconds(seconds));
  }

  SharedControlData data;
  std::unique_ptr<PolicyRuntime> runtime;
};
}  // namespace

TEST_F(PolicyActionFault, HoldsOutputAndRetriesOnFiveSeparatePolicyTicks)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  const auto good_action = data.policy.last_action;
  const auto good_target = runtime->target_command();
  auto output = good_target;
  CommandTransition transition(2);
  transition.begin(output, good_target, {0, 1}, 0, output);
  transition.update(.02, PolicyUpdateResult::TargetReady, good_target, output);

  data.sensors.joint_pos[0] = 20.0f;
  for (int attempt = 1; attempt <= 6; ++attempt) {
    if (attempt > 1) {
      ASSERT_EQ(tick(.005), PolicyUpdateResult::Skipped);
      EXPECT_FALSE(data.requests.action_limit_exceeded);
    }
    const auto result = tick(attempt == 1 ? .02 : .015);
    ASSERT_EQ(result, PolicyUpdateResult::TargetUnavailable);
    transition.update(.02, result, runtime->target_command(), output);
    EXPECT_EQ(runtime->target_command().position, good_target.position);
    EXPECT_EQ(output.position, good_target.position);
    EXPECT_EQ(data.policy.last_action, good_action);
    EXPECT_EQ(data.requests.action_limit_exceeded, attempt == 6);
    EXPECT_NEAR(data.policy.episode_time, (attempt + 1) * .02, 1e-6);
  }
}

TEST_F(PolicyActionFault, FreshSensorsRecoverAndResetTheFailureBudget)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  for (int failures : {1, 5}) {
    data.sensors.joint_pos[0] = 20.0f;
    for (int i = 0; i < failures; ++i) {
      ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
      EXPECT_FALSE(data.requests.action_limit_exceeded);
    }
    data.sensors.joint_pos[0] = .6f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    EXPECT_NEAR(runtime->target_command().position[0], .35f, 1e-6);
  }
  data.sensors.joint_pos[0] = 20.0f;
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
  EXPECT_FALSE(data.requests.action_limit_exceeded);
}

TEST_F(PolicyActionFault, ReentryResetsRetries)
{
  data.sensors.joint_pos[0] = 20.0f;
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetUnavailable);
  ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
  runtime->enter();
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetUnavailable);
  EXPECT_FALSE(data.requests.action_limit_exceeded);
}

TEST_F(PolicyActionFault, NonfiniteActionsShareTheSixFailureBudget)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  const auto target = runtime->target_command().position;
  const auto action = data.policy.last_action;
  for (int attempt = 1; attempt <= 6; ++attempt) {
    data.sensors.joint_pos[0] = attempt % 2 == 0 ? 20.0f :
      std::numeric_limits<float>::quiet_NaN();
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_EQ(data.requests.action_limit_exceeded, attempt == 6);
    EXPECT_EQ(runtime->target_command().position, target);
    EXPECT_EQ(data.policy.last_action, action);
  }
}

TEST_F(PolicyActionFault, RecoversAfterNonfiniteAction)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity()})
  {
    data.sensors.joint_pos[0] = invalid;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    data.sensors.joint_pos[0] = .4f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
  }
}

TEST(ActionFaultDiagnostics, HighlightsJumpsWithPreviousValuesAndUnits)
{
  SensorData previous, current;
  previous.joint_pos = Eigen::VectorXf::Zero(1);
  previous.joint_vel = Eigen::VectorXf::Zero(1);
  current = previous;
  current.joint_pos[0] = 1;
  current.joint_vel[0] = 10;
  current.angular_velocity.x() = 10;
  const auto log = normalized_whitespace(
    format_action_fault_sensors(current, previous, {"knee"}, .02));
  EXPECT_NE(log.find("dt=0.020"), std::string::npos);
  EXPECT_NE(log.find("knee 57.296 0.000 +57.296 [JUMP!]"), std::string::npos);
  EXPECT_NE(log.find("knee 572.958 0.000 +572.958 [JUMP!]"), std::string::npos);
  EXPECT_NE(log.find("wx 572.958 0.000 +572.958 [JUMP!]"), std::string::npos);
  const auto slow = format_action_fault_sensors(current, previous, {"knee"}, .1);
  EXPECT_EQ(slow.find("[JUMP!]"), std::string::npos);
}

TEST(ActionFaultDiagnostics, WrapsYawAndHandlesMissingBaselineAndInvalidValues)
{
  constexpr float rad = 3.141592653589793f / 180;
  SensorData previous, current;
  previous.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(179 * rad, Eigen::Vector3f::UnitZ()));
  current.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(-179 * rad, Eigen::Vector3f::UnitZ()));
  const auto wrapped = normalized_whitespace(
    format_action_fault_sensors(current, previous, {}, .02));
  EXPECT_NE(wrapped.find("yaw -179.000 179.000 +2.000"), std::string::npos);
  EXPECT_EQ(wrapped.find("[JUMP!]"), std::string::npos);
  current.angular_velocity.y() = std::numeric_limits<float>::quiet_NaN();
  const auto initial = normalized_whitespace(
    format_action_fault_sensors(current, std::nullopt, {"absent"}, 0));
  EXPECT_NE(initial.find("n/a n/a"), std::string::npos);
  EXPECT_NE(initial.find("[NONFINITE]"), std::string::npos);
  EXPECT_NE(initial.find("absent [MISSING]"), std::string::npos);
}

namespace
{
class PolicyHistoryFault : public PolicyActionFault
{
protected:
  std::filesystem::path policy_directory() const override {return HISTORY_POLICY_TEST_DATA_DIR;}
};
}  // namespace

TEST_F(PolicyHistoryFault, FourRejectedSpikesRecoverOnTheFifthPolicyTick)
{
  ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
  const auto normal_target = runtime->target_command().position;
  for (int i = 0; i < 4; ++i) {
    data.sensors.joint_pos[0] = 200.4f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    EXPECT_GT(data.policy.input[6], 200.0f);  // Keep failed input visible in diagnostics.
  }
  data.sensors.joint_pos[0] = .4f;
  // The model sums all four frames. Without rejection cleanup, the same
  // spike causes three more failures even though physical feedback is normal.
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
    EXPECT_FALSE(data.requests.action_limit_exceeded);
    EXPECT_EQ(runtime->target_command().position, normal_target);
    for (size_t frame = 0; frame < 4; ++frame) {
      EXPECT_FLOAT_EQ(data.policy.input[frame * 2], .4f);
      EXPECT_FLOAT_EQ(data.policy.input[frame * 2 + 1], -.7f);
    }
  }
}

TEST_F(PolicyActionFault, CapturesRejectedInputBeforeHistoryRepairAndRecoverySeparately)
{
  bool has_recorded_pair = false;
  // The production queue intentionally drops records on lock contention. Retry
  // the isolated scenario a bounded number of times; persistent loss still fails.
  for (int trial = 0; trial < 5; ++trial) {
    runtime->enter();
    std::string pattern = "/tmp/policy-fault-capture-XXXXXX";
    ASSERT_NE(::mkdtemp(pattern.data()), nullptr);
    ActionFaultRecorder::Options options;
    options.directory = pattern;
    auto recorder = std::make_shared<ActionFaultRecorder>(options, rclcpp::get_logger("capture_test"));
    const auto session = recorder->session_directory();
    runtime->set_fault_recorder(recorder);
    ASSERT_EQ(tick(0), PolicyUpdateResult::TargetReady);
    const auto accepted = data.policy.input;
    data.sensors.joint_pos[0] = 200.f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetUnavailable);
    const auto rejected = data.policy.input;
    data.sensors.joint_pos[0] = .4f;
    ASSERT_EQ(tick(), PolicyUpdateResult::TargetReady);
    const auto recovered = data.policy.input;
    runtime->set_fault_recorder(nullptr);
    recorder.reset();
    if (!std::filesystem::exists(session / "incident-1-attempt-1.yaml") ||
      !std::filesystem::exists(session / "incident-1-attempt-2-recovered.yaml"))
    {
      std::filesystem::remove_all(pattern);
      continue;
    }
    const auto failure = YAML::LoadFile((session / "incident-1-attempt-1.yaml").string());
    const auto recovery = YAML::LoadFile((session / "incident-1-attempt-2-recovered.yaml").string());
    const auto tensor = [](const YAML::Node & node) {
        return node["onnx_calls"][0]["inputs"][0]["raw_tensor"].as<std::vector<float>>();
      };
    EXPECT_EQ(tensor(failure["previous_accepted_inference"]), accepted);
    EXPECT_EQ(tensor(failure["current_inference"]), rejected);
    EXPECT_EQ(tensor(recovery["current_inference"]), recovered);
    EXPECT_EQ(recovery["current_inference"]["outcome"].as<std::string>(), "ACCEPTED");
    EXPECT_FALSE(recovery["previous_accepted_inference"]);
    EXPECT_NE(failure["diagnostics"].as<std::string>().find("[JUMP!]"), std::string::npos);
    std::filesystem::remove_all(pattern);
    has_recorded_pair = true;
    break;
  }
  EXPECT_TRUE(has_recorded_pair);
}

TEST(ActionFaultDiagnostics, ConsoleIncludesOnlyInvalidActionsAndJumpRows)
{
  const auto actions = format_action_fault_actions({7.f, 0.f,
    std::numeric_limits<float>::quiet_NaN()}, {"limit_joint", "normal_joint", "nan_joint"}, 6.283f, true);
  EXPECT_NE(actions.find("limit"), std::string::npos);
  EXPECT_NE(actions.find("nan"), std::string::npos);
  EXPECT_EQ(actions.find("normal"), std::string::npos);
  SensorData previous, current;
  previous.joint_pos = Eigen::VectorXf::Zero(2);
  previous.joint_vel = Eigen::VectorXf::Zero(2);
  current = previous;
  current.joint_pos[0] = 1.f;
  const auto sensors = format_action_fault_sensors(current, previous, {"jump_joint", "normal_joint"}, .02, true);
  EXPECT_NE(sensors.find("jump"), std::string::npos);
  EXPECT_EQ(sensors.find("normal"), std::string::npos);
  EXPECT_EQ(sensors.find("IMU"), std::string::npos);
  EXPECT_EQ(sensors.find("Joint velocity"), std::string::npos);
}
