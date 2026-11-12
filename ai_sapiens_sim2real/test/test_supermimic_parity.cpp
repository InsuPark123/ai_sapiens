#include <gtest/gtest.h>
#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include "ai_sapiens_sim2real/observation/observation_manager.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
#include "ai_sapiens_sim2real/policy/mimic_heading_alignment.hpp"
#include "ai_sapiens_sim2real/policy/torso_orientation.hpp"
using namespace ai_sapiens_sim2real;

TEST(SuperMimicParity, MujocoObservationsAndActorOutputs)
{
  const char * asset = std::getenv("SUPERMIMIC_ASSET");
  const char * captures = std::getenv("SUPERMIMIC_CAPTURES");
  if(!asset || !captures) {GTEST_SKIP() << "Set SUPERMIMIC_ASSET and SUPERMIMIC_CAPTURES";}
  const std::string dir(asset), input(captures);
  auto cfg = YAML::LoadFile(dir + "/params/sim2real.yaml");
  auto names = cfg["policy_joints"].as<std::vector<std::string>>();
  ASSERT_EQ(names.size(), 23U);
  Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "parity");Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
  Ort::Session session(env, (dir + "/exported/policy.onnx").c_str(), options);
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  const std::array<int64_t, 2> shape{1, 1316}; const char * in_names[] = {"observations"};
    const char * out_names[] = {"actions"};
  float max_obs = 0, max_action = 0; size_t row_count = 0, output_count = 0;
  for(int clip = 1; clip <= 7; ++clip) {
    const auto name = "taeguek_" + std::to_string(clip);
    MotionReference ref(dir + "/params/" + name + ".csv", 50, names);
    std::ifstream file(input + "/" + name + ".csv.capture"); ASSERT_TRUE(file.good());
    SharedControlData shared;shared.resize(23, 23);PolicyJointContext joints;
    joints.policy_joint_names = names;
    for(int j = 0; j < 23; ++j) {
      joints.policy_to_controller.push_back(j);
      shared.output.default_joint_pos[j] = cfg["joint_properties"][names[j]]["default_position"].as<float>();
    }
    std::string line;std::unique_ptr<ObservationManager> manager;size_t frame = 0;
    while(std::getline(file, line)) {
      std::vector<float> row;row.reserve(1393);const char * p = line.c_str();char * end = nullptr;
      while(*p) {
        float value = std::strtof(p, &end);ASSERT_NE(end, p);row.push_back(value);
        p = *end == ',' ? end + 1 : end;
      }
      ASSERT_EQ(row.size(), 1393U);size_t k = 0;double t = row[k++];
      shared.sensors.orientation = Eigen::Quaternionf(row[k], row[k + 1], row[k + 2], row[k + 3]);
      k += 4;
      shared.sensors.angular_velocity = Eigen::Vector3f(row[k], row[k + 1], row[k + 2]);k += 3;
      for(int j = 0; j < 23; ++j) {
        shared.sensors.joint_pos[j] = row[k++];
      }
      for(int j = 0; j < 23; ++j) {
        shared.sensors.joint_vel[j] = row[k++];
      }
      for(int j = 0; j < 23; ++j) {
        shared.policy.last_action[j] = row[k++];
      }
      shared.sensors.compute_projected_gravity();ref.seek(t);
      if(!manager) {
        const Eigen::Quaternionf reference = ref.root_quaternion() *
          Eigen::AngleAxisf(ref.joint_pos_for_joint(kWaistYawJointName), Eigen::Vector3f::UnitZ());
        shared.policy.motion_init_quat = supermimic_heading_alignment(reference,
          torso_orientation_in_world(shared.sensors, joints));
        manager = std::make_unique<ObservationManager>(cfg["observations"], &shared, joints, &ref);
        manager->reset();
      }
      auto actual = manager->compute();ASSERT_EQ(actual.size(), 1316U);
      std::vector<float> expected(row.begin() + k, row.end());
      for(size_t j = 0; j < actual.size(); ++j) {
        float e = std::abs(actual[j] - expected[j]);max_obs = std::max(max_obs, e);
        ASSERT_LT(e, 2e-4f) << name << " frame=" << frame << " obs=" << j;
      }
      if(frame % 100 == 0 || frame + 1 == static_cast<size_t>(ref.frame_count())) {
        auto a = Ort::Value::CreateTensor<float>(memory, actual.data(), actual.size(), shape.data(),
          shape.size());
        auto b = Ort::Value::CreateTensor<float>(memory, expected.data(), expected.size(),
          shape.data(), shape.size());
        auto oa = session.Run(Ort::RunOptions{nullptr}, in_names, &a, 1, out_names, 1);
        auto ob = session.Run(Ort::RunOptions{nullptr}, in_names, &b, 1, out_names, 1);
        auto va = oa[0].GetTensorData<float>();auto vb = ob[0].GetTensorData<float>();
        for(int j = 0; j < 23; ++j) {
          float e = std::abs(va[j] - vb[j]);max_action = std::max(max_action, e);
          ASSERT_LT(e, 1e-3f) << name << " frame=" << frame << " action=" << j;
        }
        ++output_count;
      }
      ++frame;++row_count;
    }
    EXPECT_EQ(frame, static_cast<size_t>(ref.frame_count()));
  }
  std::cout << "Parity samples=" << row_count << " output comparisons=" << output_count <<
    " max obs error=" << max_obs << " max action error=" << max_action << std::endl;
}

TEST(SuperMimicParity, RejectsReferenceWithDifferentPolicyDefaults)
{
  const char * asset = std::getenv("SUPERMIMIC_ASSET");
  if(!asset) {GTEST_SKIP() << "Set SUPERMIMIC_ASSET";}
  const std::string dir(asset);auto cfg = YAML::LoadFile(dir + "/params/sim2real.yaml");
  auto names = cfg["policy_joints"].as<std::vector<std::string>>();
  MotionReference ref(dir + "/params/taeguek_6.csv", 50, names);
  SharedControlData shared;shared.resize(23, 23);PolicyJointContext joints;
    joints.policy_joint_names = names;
  std::vector<float> defaults;
  for(int j = 0; j < 23; ++j) {
    joints.policy_to_controller.push_back(j);
    defaults.push_back(cfg["joint_properties"][names[j]]["default_position"].as<float>());
                                                                                                                                                       }
  defaults[0] += 1.0f;
  EXPECT_THROW(ObservationManager(cfg["observations"], &shared, joints, &ref, nullptr, &defaults),
    std::runtime_error);
}
