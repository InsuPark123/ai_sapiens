// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <vector>

#include "ai_sapiens_sim2real/command_transition.hpp"
#include "ai_sapiens_sim2real/policy/k1_mirror.hpp"
#include "ai_sapiens_sim2real/policy/motion_source.hpp"
#include "ai_sapiens_sim2real/policy/mimic_policy_runtime.hpp"

using namespace ai_sapiens_sim2real;

TEST(K1Mirror, ReflectsAxesAndIsAnInvolution)
{
  K1Mirror mirror({"right_hip_roll_joint", "left_hip_roll_joint", "waist_yaw_joint",
    "left_knee_joint", "right_knee_joint"});
  const std::vector<float> values{1, 2, 3, 4, 5};
  EXPECT_EQ(mirror.reflect(values), (std::vector<float>{-2, -1, -3, 5, 4}));
  EXPECT_EQ(mirror.reflect(mirror.reflect(values)), values);
  EXPECT_EQ(mirror.reflect(values, false), (std::vector<float>{2, 1, 3, 5, 4}));
  const Eigen::Quaternionf q = Eigen::Quaternionf(Eigen::AngleAxisf(.7f,
      Eigen::Vector3f(1, 2, 3).normalized()));
  Eigen::Matrix3f reflection = Eigen::Vector3f(1, -1, 1).asDiagonal();
  EXPECT_TRUE(K1Mirror::orientation(q).toRotationMatrix().isApprox(
      reflection * q.toRotationMatrix() * reflection));
  EXPECT_THROW(K1Mirror({"left_knee_joint"}), std::runtime_error);
  EXPECT_THROW(K1Mirror({"unknown_joint"}), std::runtime_error);
}

TEST(MimicMirror, ModelCoordinatesStayLocalAndCompletionHandsOffPhysicalCommand)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  const auto node = std::make_shared<rclcpp::Node>("mimic_mirror_test");
  const std::filesystem::path directory(COMMON_POLICY_TEST_DATA_DIR);
  Sim2RealConfig config(directory / "sim2real.yaml");
  const auto names = config.policy_joints();
  const K1Mirror mirror(names);
  SharedControlData real, virtual_robot;
  real.resize(2, 2);
  virtual_robot.resize(2, 2);
  real.sensors.joint_pos << .4f, -.7f;
  real.sensors.joint_vel << .2f, -.3f;
  real.sensors.angular_velocity << .1f, .2f, .3f;
  real.sensors.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(-.4f, Eigen::Vector3f::UnitX()));
  real.sensors.compute_projected_gravity();
  real.sensors.is_imu_usable = true;
  const auto original_sensors = real.sensors;
  virtual_robot.sensors = real.sensors;
  virtual_robot.sensors.joint_pos = mirror.reflect(real.sensors.joint_pos);
  virtual_robot.sensors.joint_vel = mirror.reflect(real.sensors.joint_vel);
  virtual_robot.sensors.angular_velocity << -.1f, .2f, -.3f;
  virtual_robot.sensors.orientation = K1Mirror::orientation(real.sensors.orientation);
  virtual_robot.sensors.compute_projected_gravity();
  MimicBehavior mimic;
  mimic.motion_file = std::filesystem::path(MIRROR_TEST_DATA_DIR) / "motion.csv";
  mimic.on_complete = "Velocity";
  mimic.source = std::make_shared<MotionSource>(mimic.motion_file.string(), 50.0f,
    std::vector<std::string>{"left_hip_roll_joint", "right_hip_roll_joint", "waist_yaw_joint"});
  auto ordinary = MimicPolicyRuntime::create(node, "ordinary", (directory / "policy.onnx").string(),
      config, mimic, names, &virtual_robot);
  mimic.is_mirrored = true;
  auto mirrored = MimicPolicyRuntime::create(node, "mirrored", (directory / "policy.onnx").string(),
      config, mimic, names, &real);
  // Entry comparison must map the physical published command back into policy coordinates.
  real.output.has_published_command = virtual_robot.output.has_published_command = true;
  real.output.last_published.position = {-2.75f, -3.35f};
  virtual_robot.output.last_published.position = mirror.reflect(real.output.last_published.position);
  ordinary->enter();
  mirrored->enter();
  const auto dt = rclcpp::Duration::from_seconds(.02);
  for (int i = 0; i < 2; ++i) {
    ASSERT_EQ(ordinary->update(dt), PolicyUpdateResult::TargetReady);
    ASSERT_EQ(mirrored->update(dt), PolicyUpdateResult::TargetReady);
    EXPECT_EQ(real.policy.input, virtual_robot.policy.input);
    EXPECT_EQ(real.policy.last_action, virtual_robot.policy.last_action);
    EXPECT_EQ(mirrored->target_command().position,
      mirror.reflect(ordinary->target_command().position));
  }
  EXPECT_EQ(mirrored->target_command().stiffness, (std::vector<float>{20, 10}));
  EXPECT_EQ(mirrored->target_command().damping, (std::vector<float>{2, 1}));
  EXPECT_EQ(mirrored->update(dt), PolicyUpdateResult::TargetUnavailable);
  EXPECT_EQ(real.requests.state_name, "Velocity");
  EXPECT_TRUE(real.sensors.joint_pos.isApprox(original_sensors.joint_pos));
  EXPECT_TRUE(real.sensors.orientation.isApprox(original_sensors.orientation));
  EXPECT_TRUE(real.sensors.angular_velocity.isApprox(original_sensors.angular_velocity));

  // Next runtime observes real sensors and seeds fresh history. Blend starts at
  // the physical mirrored output, regardless of the next policy's joint ABI.
  PolicyRuntime velocity(node, "velocity", (directory / "policy.onnx").string(), config, names, &real);
  auto output = mirrored->target_command();
  const auto source = output;
  CommandTransition transition(2);
  transition.begin(source, velocity.target_command(), {0, 1}, .1, output);
  velocity.enter();
  EXPECT_EQ(real.policy.last_action, (std::vector<float>{0, 0}));
  ASSERT_EQ(velocity.update(dt), PolicyUpdateResult::TargetReady);
  EXPECT_NEAR(real.policy.input[0], .3f, 1e-6);
  EXPECT_NEAR(real.policy.input[1], -.5f, 1e-6);
  EXPECT_NEAR(velocity.target_command().position[0], .25f, 1e-6);
  EXPECT_NEAR(velocity.target_command().position[1], -.325f, 1e-6);
  transition.update(.02, PolicyUpdateResult::TargetReady, velocity.target_command(), output);
  EXPECT_EQ(output.position, source.position);
  transition.update(.1, PolicyUpdateResult::TargetReady, velocity.target_command(), output);
  EXPECT_EQ(output.position, velocity.target_command().position);
  mirrored->enter();
  EXPECT_EQ(real.policy.last_action, (std::vector<float>{0, 0}));
  EXPECT_EQ(mirrored->update(dt), PolicyUpdateResult::TargetReady);
}

TEST(MimicMirror, RejectsCommandObservationsBeforeLoadingModel)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  const auto node = std::make_shared<rclcpp::Node>("unsupported_mirror_test");
  const std::filesystem::path directory(COMMON_POLICY_TEST_DATA_DIR);
  for (const auto & term : {"velocity_commands", "gait_phase"}) {
    auto document = YAML::LoadFile((directory / "sim2real.yaml").string());
    document["observations"][term] = YAML::Load("{}");
    const auto path = std::filesystem::temp_directory_path() /
      (std::string("unsupported_mirror_") + term + ".yaml");
    std::ofstream(path) << document;
    Sim2RealConfig config(path);
    SharedControlData shared;
    shared.resize(2, 2);
    MimicBehavior mimic;
    mimic.is_mirrored = true;
    mimic.motion_file = std::filesystem::path(MIRROR_TEST_DATA_DIR) / "motion.csv";
    mimic.source = std::make_shared<MotionSource>(mimic.motion_file.string(), 50.0f,
      std::vector<std::string>{"left_hip_roll_joint", "right_hip_roll_joint", "waist_yaw_joint"});
    try {
      MimicPolicyRuntime::create(node, "unsupported", "missing.onnx", config, mimic,
        config.policy_joints(), &shared);
      FAIL() << "Expected unsupported mirrored policy to be rejected";
    } catch (const std::runtime_error & error) {
      EXPECT_NE(std::string(error.what()).find("mirror supports ordinary Mimic"), std::string::npos);
    }
    std::filesystem::remove(path);
  }
}
