// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Kiwoong Park

#ifndef AI_SAPIENS_SIM2REAL__POLICY__POLICY_RUNTIME_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__POLICY_RUNTIME_HPP_

#include <cmath>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include "ai_sapiens_sim2real/policy/action_pipeline.hpp"
#include "ai_sapiens_sim2real/policy/policy_action_result.hpp"
#include "ai_sapiens_sim2real/policy/k1_mirror.hpp"
#include "ai_sapiens_sim2real/policy/any2track_policy.hpp"
#include "ai_sapiens_sim2real/policy/gait_clock.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
#include "ai_sapiens_sim2real/observation/observation_manager.hpp"
#include "ai_sapiens_sim2real/config/sim2real_config.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

class OnnxInference;
class LocomotionMimic;
class ActionFaultRecorder;
class PolicyDiagnostics;

/**
 * @brief A policy state: observe, run inference, and produce a validated target
 *        command in policy joint order. The controller owns command delivery.
 *
 * This base is the whole behavior for a plain `kind: policy` state. The
 * per-tick flow lives here once (enter/update are template methods);
 * MimicPolicyRuntime extends it through the on_enter()/prepare_observation() hooks to play
 * a reference motion. Any2Track's separate input/action ABI is composed through
 * Any2TrackPolicy, shared with the standalone simulator. Both paths keep target
 * validation, failure escalation and controller command delivery in common.
 */
class PolicyRuntime
{
public:
  PolicyRuntime(
    rclcpp::Node::SharedPtr node,
    std::string state_name,
    std::string model_path,
    const Sim2RealConfig & sim2real_config,
    const std::vector<std::string> & controller_joint_names,
    SharedControlData * shared_data,
    const MotionReference * reference_motion = nullptr,
    const MimicBehavior * any2track_mimic = nullptr,
    bool is_mirrored = false);
  virtual ~PolicyRuntime();

  void set_fault_recorder(std::shared_ptr<ActionFaultRecorder> recorder);
  void reset();
  bool check_inputs();
  bool requires_localization() const {return requires_localization_;}
  void enter();
  PolicyUpdateResult update(const rclcpp::Duration & period);
  const JointCommand & target_command() const {return target_command_;}
  const std::vector<size_t> & controlled_joints() const
  {
    return joint_context_.policy_to_controller;
  }
  const std::string & state_name() const;
  size_t observation_size() const
  {
    if (any2track_) {return 134U;}
    if (!obs_manager_) {
      return 0U;
    }

    return obs_manager_->get_observation_size();
  }

protected:
  // Per-tick hooks split around inference. prepare_observation() runs before it:
  // the base advances the gait phase, mimic seeks the motion (returning false to
  // skip inference once the window ends). advance_clocks() runs after, advancing
  // episode_time so the next tick infers one step later. on_enter() handles
  // per-kind entry.
  virtual void on_enter();
  virtual bool prepare_observation();
  // Runs with the active policy's resolved command, before computing observations.
  virtual void prepare_command_observation() {}
  virtual void advance_clocks();

  void set_velocity_command_ranges(const AxisRanges & ranges)
  {
    velocity_command_ranges_ = ranges;
  }

  // State the hooks read; writes still go only through the owned output block,
  // which stays private so derived kinds cannot bypass target validation.
  const SensorData * sensors_;
  PolicyState * policy_;
  ModeRequests * requests_;
  // Whole shared block, passed to per-step rules that read robot state (gait clock).
  const SharedControlData * shared_data_;
  const PolicyJointContext & joint_context() const
  {
    return joint_context_;
  }

private:
  // Startup loading is split by asset type so constructor order stays visible.
  void load_sim2real_config(
    const Sim2RealConfig & sim2real_config,
    const std::vector<std::string> & controller_joint_names);
  void log_joint_coverage(const std::vector<std::string> & controller_joint_names) const;
  void log_loading() const;
  void load_onnx_model();
  size_t create_observation_manager(
    const Sim2RealConfig & sim2real_config,
    const SharedControlData * shared_data,
    const MotionReference * reference_motion);
  void validate_observation_size(size_t observation_size) const;
  void log_ready(size_t observation_size) const;
  void initialize_diagnostics(const std::vector<std::string> & controller_joints);

  // Entry steps, each at the same level so enter() reads as one sequence.
  void install_joint_properties() const;
  void install_velocity_command_ranges() const;
  void reset_episode_state();

  // RT update helpers keep PolicyRuntime::update at one abstraction level.
  bool advance_policy_tick(const rclcpp::Duration & period);
  // Applies the active policy's velocity-command range before observation.
  void resolve_active_velocity_command();
  void compute_observation();
  void refresh_sensor_view();
  bool prepare_policy_observation();
  std::vector<float> run_policy_inference();
  PolicyActionResult evaluate_policy_action();
  bool is_action_valid(const PolicyActionResult & action) const;
  void commit_action(const PolicyActionResult & action);
  void reject_action(PolicyActionStatus status);
  std::vector<float> infer_once(std::unordered_map<std::string, std::vector<float>> & inputs);

  rclcpp::Node::SharedPtr node_;
  BehaviorOutput * output_;
  AxisRanges * active_velocity_command_ranges_;
  ModeDecision * mode_;
  std::string state_name_;
  std::string model_path_;
  // Kept only for log messages; the config is parsed in the constructor.
  std::filesystem::path sim2real_config_path_;

  PolicyJointContext joint_context_;
  std::optional<K1Mirror> mirror_;
  SensorData sensor_view_;
  JointProperties joint_properties_;
  ActionPipeline action_pipeline_;
  JointCommand target_command_;

  Any2TrackSample any2track_sample() const;
  std::unique_ptr<Any2TrackPolicy> any2track_;
  std::unique_ptr<OnnxInference> inference_;
  std::unique_ptr<PolicyDiagnostics> diagnostics_;
  std::unique_ptr<LocomotionMimic> locomotion_mimic_;
  std::unique_ptr<ObservationManager> obs_manager_;
  // Set only when this policy declares a gait_phase observation.
  std::optional<GaitClock> gait_clock_;
  std::string onnx_input_name_;
  std::unordered_map<std::string, std::vector<float>> obs_buffer_;

  // From this policy's sim2real.yaml; stays [-1, 1] when the policy declares none.
  AxisRanges velocity_command_ranges_;
  double step_dt_{0.02};
  double accumulated_period_{0.02};
  bool requires_localization_{false};
  bool requires_imu_orientation_{false};
  static constexpr float kAbsActionLimitRad = 2.0f * 3.14159265f;
  // Share the retry budget across rejected actions and inference exceptions:
  // only a validated action resets it, so alternating failures cannot hold forever.
  static constexpr size_t kMaxConsecutiveActionFailures = 6;
  size_t consecutive_action_failures_{0};
};

}  // namespace ai_sapiens_sim2real

#endif  // AI_SAPIENS_SIM2REAL__POLICY__POLICY_RUNTIME_HPP_
