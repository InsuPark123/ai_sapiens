#include <gtest/gtest.h>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include "ai_sapiens_sim2real/policy/mimic_policy_runtime.hpp"
#include "ai_sapiens_sim2real/config/sim2real_config.hpp"
using namespace ai_sapiens_sim2real;

TEST(SuperMimicRuntime, LoadsAndRejectsInvalidImuBeforeInference)
{
 const char * path = std::getenv("SUPERMIMIC_ASSET");if(!path) {
    GTEST_SKIP() << "Set SUPERMIMIC_ASSET";
                                                                                                       }
 const std::string base(path);if(!rclcpp::ok()) {rclcpp::init(0, nullptr);}
 auto node = std::make_shared<rclcpp::Node>("supermimic_runtime_test");
 Sim2RealConfig cfg(base + "/params/sim2real.yaml");MimicBehavior behavior;
 behavior.motion_file = base + "/params/taeguek_6.csv";behavior.fps = 50;
    behavior.on_complete = "Velocity";
 SharedControlData shared;shared.resize(23, 23);
    MotionReference reference(behavior.motion_file.string(), 50, cfg.policy_joints());
 auto first = reference.frame(0);shared.sensors.orientation = first.root_quaternion;
 shared.sensors.joint_pos = first.joint_pos;shared.sensors.joint_vel = first.joint_vel;
    shared.sensors.compute_projected_gravity();
 auto runtime = MimicPolicyRuntime::create(node, "SuperMimicTaeguek6",
    base + "/exported/policy.onnx", cfg, behavior, cfg.policy_joints(), &shared);
 runtime->enter();shared.sensors.is_imu_usable = false;
 EXPECT_FALSE(runtime->check_inputs());
 shared.sensors.is_imu_usable = true;
 EXPECT_TRUE(runtime->check_inputs());
 EXPECT_EQ(runtime->update(rclcpp::Duration::from_seconds(.02)), PolicyUpdateResult::TargetReady);
 EXPECT_EQ(shared.policy.input.size(), 1316U);
}
