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

#include "ai_sapiens_sim2real/policy/policy_runtime.hpp"

#include <cmath>

#include "ai_sapiens_sim2real/modules/locomotion_mimic/runtime.hpp"
#include "ai_sapiens_sim2real/policy/policy_diagnostics.hpp"
#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

bool contains_joint_name(
  const std::vector<std::string> & joint_names,
  const std::string & joint_name)
{
  return std::find(joint_names.begin(), joint_names.end(), joint_name) != joint_names.end();
}

std::string action_size_mismatch_message(
  const std::string & label,
  size_t actual_size,
  size_t expected_size)
{
  return label + " action size " + std::to_string(actual_size) +
         " does not match policy_joints size " + std::to_string(expected_size);
}

}  // namespace

PolicyRuntime::PolicyRuntime(
  rclcpp::Node::SharedPtr node,
  std::string state_name,
  std::string model_path,
  const Sim2RealConfig & sim2real_config,
  const std::vector<std::string> & controller_joint_names,
  SharedControlData * shared_data,
  const MotionReference * reference_motion,
  const MimicBehavior * any2track_mimic,
  bool is_mirrored)
: sensors_(&shared_data->sensors)
  , policy_(&shared_data->policy)
  , requests_(&shared_data->requests)
  , shared_data_(shared_data)
  , node_(std::move(node))
  , output_(&shared_data->output)
  , active_velocity_command_ranges_(&shared_data->active_velocity_command_ranges)
  , mode_(&shared_data->mode)
  , state_name_(std::move(state_name))
  , model_path_(std::move(model_path))
  , sim2real_config_path_(sim2real_config.path())
{
  configure_input_requirements(sim2real_config, reference_motion);
  load_sim2real_config(sim2real_config, controller_joint_names);
  configure_mirroring(sim2real_config, reference_motion, is_mirrored);
  log_joint_coverage(controller_joint_names);
  log_loading();

  initialize_policy_modules(sim2real_config, reference_motion, any2track_mimic);
  load_onnx_model();
  const auto observation_size = initialize_observations(sim2real_config, reference_motion);

  initialize_target_command();
  initialize_diagnostics(controller_joint_names);

  log_ready(observation_size);
}

void PolicyRuntime::configure_input_requirements(
  const Sim2RealConfig & sim2real_config, const MotionReference * reference_motion)
{
  const auto observations = sim2real_config.observations();
  const bool has_robot_xy = static_cast<bool>(observations["robot_root_position_xy_w"]);
  const bool has_reference_xy = static_cast<bool>(observations["reference_root_position_xy_w"]);
  const bool has_robot_velocity = static_cast<bool>(observations["robot_root_velocity_xy_h"]);
  const bool has_reference_velocity =
    static_cast<bool>(observations["reference_root_velocity_xy_h"]);

  if (has_robot_velocity || has_reference_velocity) {
    throw std::runtime_error(
        "Global-position Mimic requires robot/reference_root_position_xy_w; "
        "root velocity observations belong to an incompatible training schema");
  }

  const bool supermimic = static_cast<bool>(observations["supermimic_future"]);
  requires_localization_ = has_robot_xy || sim2real_config.any2track_path_tracking();
  requires_imu_orientation_ = sim2real_config.any2track() || sim2real_config.use_imu_orientation() || supermimic;

  if (requires_imu_orientation_ && !requires_localization_ && !sim2real_config.any2track() && !supermimic) {
    throw std::runtime_error("reference_trajectory.orientation_source=imu requires global-position Mimic");
  }

  const bool has_incomplete_xy_pair = has_robot_xy != has_reference_xy;
  const bool is_motion_reference_missing =
    requires_localization_ && !sim2real_config.any2track() && reference_motion == nullptr;

  if (has_incomplete_xy_pair || is_motion_reference_missing) {
    throw std::runtime_error(
        "Global-position Mimic requires both robot/reference_root_position_xy_w and a motion reference");
  }
}

void PolicyRuntime::configure_mirroring(
  const Sim2RealConfig & sim2real_config, const MotionReference * reference_motion, bool is_mirrored)
{
  if (!is_mirrored) {
    return;
  }

  const auto observations = sim2real_config.observations();

  if (!reference_motion || sim2real_config.any2track() || requires_localization_ ||
    sim2real_config.steering() || sim2real_config.dance_reference() ||
    observations["velocity_commands"] || observations["gait_phase"])
  {
    throw std::runtime_error(
      "mirror supports ordinary Mimic without localization, steering, dance reference or gait commands");
  }

  mirror_.emplace(joint_context_.policy_joint_names);
  refresh_sensor_view();
  sensors_ = &sensor_view_;
}

void PolicyRuntime::initialize_policy_modules(
  const Sim2RealConfig & sim2real_config, const MotionReference * reference_motion,
  const MimicBehavior * any2track_mimic)
{
  locomotion_mimic_ = LocomotionMimic::create_if_configured(
    sim2real_config, reference_motion != nullptr, mirror_.has_value());
  is_reference_mimic_ = reference_motion != nullptr && !sim2real_config.any2track();

  if (sim2real_config.any2track()) {
    if (!any2track_mimic) {
      throw std::runtime_error("Any2Track requires a mimic behavior");
    }

    any2track_ = std::make_unique<Any2TrackPolicy>(sim2real_config, *any2track_mimic);
    RCLCPP_INFO(node_->get_logger(),
      "[Any2Track] IMU attitude/gyro; XY input=%s; "
      "20-frame velocity history and 79-frame adapter history",
      requires_localization_ ? "pelvis odometry" : "none (orientation-only model)");
  }
}

size_t PolicyRuntime::initialize_observations(
  const Sim2RealConfig & sim2real_config, const MotionReference * reference_motion)
{
  const size_t observation_size = any2track_ ? any2track_->observation().size() :
    create_observation_manager(sim2real_config, shared_data_, reference_motion);

  if (!any2track_) {
    inference_->validate_policy_metadata(
      joint_context_.policy_joint_names, obs_manager_->get_term_names(), requires_localization_);
    gait_clock_ = make_gait_clock(sim2real_config.observations(), step_dt_);
  }

  if (locomotion_mimic_) {
    locomotion_mimic_->set_observation_layout(observation_size, obs_manager_->get_term_names());
  }

  validate_observation_size(observation_size);
  obs_buffer_[onnx_input_name_].resize(observation_size);

  return observation_size;
}

void PolicyRuntime::initialize_target_command()
{
  target_command_.resize(joint_context_.policy_joint_names.size());
  target_command_.stiffness = joint_properties_.stiffness;
  target_command_.damping = joint_properties_.damping;

  if (mirror_) {
    target_command_.stiffness = mirror_->reflect(target_command_.stiffness, false);
    target_command_.damping = mirror_->reflect(target_command_.damping, false);
  }
}

PolicyRuntime::~PolicyRuntime() = default;

void PolicyRuntime::initialize_diagnostics(const std::vector<std::string> & controller_joints)
{
  FaultPolicyMetadata metadata;
  metadata.state = state_name_;
  metadata.model = model_path_;
  metadata.config_path = sim2real_config_path_.string();

  metadata.policy_joints = joint_context_.policy_joint_names;
  metadata.controller_joints = controller_joints;
  metadata.input_names = inference_->get_input_names();
  metadata.input_shapes = inference_->get_input_shapes();

  metadata.step_dt = step_dt_;
  metadata.action_limit = action_safety_limits_.absolute_position_rad;
  metadata.action_change_limit = action_safety_limits_.delta_rad;
  metadata.raw_action_boundary = action_safety_limits_.raw_action_boundary;
  metadata.is_mirrored = mirror_.has_value();

  if (obs_manager_) {
    metadata.layouts[onnx_input_name_] = obs_manager_->layout();
  } else if (any2track_) {
    metadata.layouts = any2track_fault_layout(any2track_->observation().size());
  }

  diagnostics_ = std::make_unique<PolicyDiagnostics>(
    std::move(metadata), kMaxConsecutiveActionFailures, node_->get_logger());
}

void PolicyRuntime::set_fault_recorder(std::shared_ptr<ActionFaultRecorder> recorder)
{
  diagnostics_->set_recorder(std::move(recorder));
}

bool PolicyRuntime::check_inputs()
{
  return (!requires_localization_ || shared_data_->localization.valid) &&
         (!requires_imu_orientation_ || shared_data_->sensors.is_imu_usable);
}

void PolicyRuntime::reset()
{
  diagnostics_->reset();
  accumulated_period_ = step_dt_;
  if (obs_manager_) {
    obs_manager_->reset();
  }
}

void PolicyRuntime::enter()
{
  diagnostics_->reset();
  refresh_sensor_view();
  policy_->uses_global_position = requires_localization_;
  policy_->motion_frame.reset();
  policy_->uses_motion_steering = false;
  policy_->motion_steering.reset();
  install_joint_properties();
  install_velocity_command_ranges();
  reset_episode_state();
  seed_action_reference();
  on_enter();
  if (obs_manager_) {obs_manager_->reset();}  // seed after on_enter()
  if (requires_localization_) {
    RCLCPP_INFO(node_->get_logger(),
      "[PolicyEntry] %s: reset time/action/steering/frame; orientation_source=%s",
      state_name_.c_str(), requires_imu_orientation_ ? "imu" : "localization");
    if (sensors_->is_imu_usable) {
      // Gravity in the body frame compares tilt without confusing independent
      // IMU/odom world yaw origins. This is a diagnostic, not a pose correction.
      const auto imu_up = sensors_->orientation.normalized().conjugate() * Eigen::Vector3f::UnitZ();
      const auto odom_up = shared_data_->localization.orientation.normalized().conjugate() *
        Eigen::Vector3f::UnitZ();
      const float tilt_degrees = std::acos(std::clamp(imu_up.dot(odom_up), -1.0f, 1.0f)) *
        180.0f / 3.14159265359f;
      RCLCPP_INFO(node_->get_logger(), "[PolicyEntry] %s: IMU/odom tilt difference %.2f deg",
        state_name_.c_str(), tilt_degrees);
    }
  }
}

void PolicyRuntime::install_joint_properties() const
{
  const size_t policy_joint_count = joint_context_.policy_joint_names.size();
  if (joint_properties_.default_position.size() != policy_joint_count ||
    joint_properties_.stiffness.size() != policy_joint_count ||
    joint_properties_.damping.size() != policy_joint_count ||
    joint_properties_.position_limits.size() != policy_joint_count)
  {
    throw std::runtime_error("Policy state '" + state_name_ + "' joint property size mismatch");
  }

  // Scatter each policy joint's properties into its robot-joint slot. Slots for
  // joints this policy does not control are never touched, so they keep the
  // previous behavior's gains and defaults (see BehaviorOutput).
  const auto & action_properties = action_pipeline_.properties();
  for (size_t policy_index = 0; policy_index < policy_joint_count; ++policy_index) {
    const size_t controller_index = joint_context_.policy_to_controller[policy_index];
    const size_t source = mirror_ ? mirror_->source(policy_index) : policy_index;
    const float sign = mirror_ ? mirror_->sign(policy_index) : 1.0f;
    output_->default_joint_pos[static_cast<Eigen::Index>(controller_index)] =
      sign * joint_properties_.default_position[source];
    output_->feedforward[controller_index] = 0.0f;
    output_->action_scale[controller_index] = sign * action_properties.scale[source];
    output_->action_offset[controller_index] = sign * action_properties.offset[source];
    auto limit = joint_properties_.position_limits[source];
    if (limit && sign < 0.0f) {
      limit = std::make_pair(-limit->second, -limit->first);
    }
    output_->position_limits[controller_index] = limit;
  }
}

void PolicyRuntime::install_velocity_command_ranges() const
{
  *active_velocity_command_ranges_ = velocity_command_ranges_;
}

void PolicyRuntime::reset_episode_state()
{
  policy_->episode_time = 0.0f;
  if (locomotion_mimic_) {locomotion_mimic_->reset();}
  if (policy_->last_action.size() < joint_context_.policy_joint_names.size()) {
    throw std::runtime_error("Policy state '" + state_name_ + "' last_action buffer too small");
  }

  std::fill(policy_->last_action.begin(), policy_->last_action.end(), 0.0f);
  accumulated_period_ = step_dt_;
  consecutive_action_failures_ = 0;
}

void PolicyRuntime::seed_action_reference()
{
  const auto & source = output_->has_published_command ? output_->last_published : output_->command;
  action_reference_.resize(joint_context_.policy_to_controller.size());
  for (size_t j = 0; j < action_reference_.size(); ++j) {
    action_reference_[j] = source.position.at(joint_context_.policy_to_controller[j]);
  }
  // Reflection is its own inverse. Compare both values in policy coordinates.
  if (mirror_) {action_reference_ = mirror_->reflect(action_reference_);}
}

void PolicyRuntime::on_enter()
{
  if (any2track_) {
    any2track_->reset(any2track_sample(),
      !requires_localization_ || shared_data_->localization.align_on_entry);
  }
  if (gait_clock_) {
    gait_clock_->reset(policy_->phase_state);
    if (gait_clock_->resolve_command_before_update()) {resolve_active_velocity_command();}
  }
}

bool PolicyRuntime::prepare_observation()
{
  if (any2track_) {
    if (!any2track_->observe(any2track_sample())) {
      requests_->state_name = any2track_->completion_state();
      return false;
    }
    return true;
  }
  if (gait_clock_) {
    gait_clock_->advance(policy_->phase_state, *shared_data_);
  }

  return true;
}

void PolicyRuntime::advance_clocks()
{
  policy_->episode_time += step_dt_;
  if (any2track_) {any2track_->advance();}
}

PolicyUpdateResult PolicyRuntime::update(const rclcpp::Duration & period)
{
  diagnostics_->advance(period.seconds());
  if (!advance_policy_tick(period)) {return PolicyUpdateResult::Skipped;}
  if (!prepare_policy_observation()) {return PolicyUpdateResult::TargetUnavailable;}

  diagnostics_->begin(policy_->episode_time, shared_data_->sensors);
  const auto action = evaluate_policy_action();
  const size_t previous_failures = consecutive_action_failures_;
  if (action.is_accepted()) {commit_action(action);}
  else {reject_action(action.status);}
  diagnostics_->finish(action, shared_data_->sensors,
    previous_failures, consecutive_action_failures_);
  advance_clocks();
  return action.is_accepted() ? PolicyUpdateResult::TargetReady : PolicyUpdateResult::TargetUnavailable;
}

bool PolicyRuntime::prepare_policy_observation()
{
  refresh_sensor_view();
  const bool command_first = gait_clock_ && gait_clock_->resolve_command_before_update();
  if (command_first) {resolve_active_velocity_command();}
  if (!prepare_observation()) {
    return false;
  }

  if (!command_first) {resolve_active_velocity_command();}
  prepare_command_observation();
  if (locomotion_mimic_) {
    mode_->velocity_commands = locomotion_mimic_->update(
      *shared_data_, joint_context_, action_pipeline_.properties());
  }
  compute_observation();
  return true;
}

bool PolicyRuntime::advance_policy_tick(const rclcpp::Duration & period)
{
  // Accumulate measured control periods and run the policy at configured step_dt.
  accumulated_period_ += period.seconds();
  if (accumulated_period_ < step_dt_) {
    return false;
  }

  // Drop whole elapsed steps without looping on large time jumps.
  accumulated_period_ = std::fmod(accumulated_period_, step_dt_);
  return true;
}

void PolicyRuntime::resolve_active_velocity_command()
{
  const auto & active = *active_velocity_command_ranges_;
  switch (shared_data_->mode.velocity_source) {
    case VelocitySource::Zero:
      mode_->velocity_commands.setZero();
      break;
    case VelocitySource::Api: {
        const auto & raw = shared_data_->api.velocity_command_raw;
        mode_->velocity_commands.x() = clamp_to_axis(raw.x(), active.linear_x);
        mode_->velocity_commands.y() = clamp_to_axis(raw.y(), active.linear_y);
        mode_->velocity_commands.z() = clamp_to_axis(raw.z(), active.angular_z);
        break;
      }
    case VelocitySource::Teleop: {
        const auto & normalized = shared_data_->teleop.velocity_command_normalized;
        mode_->velocity_commands.x() = scale_unit_to_axis(normalized.x(), active.linear_x);
        mode_->velocity_commands.y() = scale_unit_to_axis(normalized.y(), active.linear_y);
        mode_->velocity_commands.z() = scale_unit_to_axis(normalized.z(), active.angular_z);
        break;
      }
  }
}

void PolicyRuntime::refresh_sensor_view()
{
  if (!mirror_) {return;}
  sensor_view_ = shared_data_->sensors;
  sensor_view_.orientation = K1Mirror::orientation(sensor_view_.orientation);
  sensor_view_.angular_velocity.x() *= -1.0f;
  sensor_view_.angular_velocity.z() *= -1.0f;
  sensor_view_.projected_gravity.y() *= -1.0f;
  const auto & mapping = joint_context_.policy_to_controller;
  for (size_t i = 0; i < mapping.size(); ++i) {
    const auto source = mapping[mirror_->source(i)];
    sensor_view_.joint_pos[mapping[i]] = mirror_->sign(i) * shared_data_->sensors.joint_pos[source];
    sensor_view_.joint_vel[mapping[i]] = mirror_->sign(i) * shared_data_->sensors.joint_vel[source];
  }
}

void PolicyRuntime::compute_observation()
{
  if (any2track_) {
    policy_->input = any2track_->observation();
    return;
  }
  // ObservationManager returns the full scaled/clipped/history observation.
  obs_buffer_[onnx_input_name_] = obs_manager_->compute();
  policy_->input = obs_buffer_[onnx_input_name_];
}

std::vector<float> PolicyRuntime::infer_once(
  std::unordered_map<std::string, std::vector<float>> & inputs)
{
  diagnostics_->input(inputs);
  auto output = inference_->run(inputs);
  diagnostics_->output(output);
  return output;
}

std::vector<float> PolicyRuntime::infer_observation(const std::vector<float> & observation)
{
  std::unordered_map<std::string, std::vector<float>> inputs{{onnx_input_name_, observation}};
  return infer_once(inputs);
}

std::vector<float> PolicyRuntime::run_policy_inference()
{
  if (locomotion_mimic_) {
    return locomotion_mimic_->infer(
      obs_buffer_.at(onnx_input_name_),
      [this](const std::vector<float> & input) {return infer_observation(input);});
  }

  return infer_once(any2track_ ? any2track_->inputs() : obs_buffer_);
}

PolicyActionResult PolicyRuntime::evaluate_policy_action()
{
  PolicyActionResult result;

  try {
    result.raw = run_policy_inference();
    result.processed = any2track_ ? any2track_->target(result.raw) :
      action_pipeline_.process(result.raw);
    validate_action_dimensions(result);
    result.validation = validate_action_basics(result.processed,
      is_action_change_check_enabled() ? &action_reference_ : nullptr, action_safety_limits_);

    if (result.validation.needs_boundary_evaluation()) {
      complete_action_boundary_check(result.validation, evaluate_raw_action_boundaries(result.raw));
    }

    result.status = result.validation.is_valid() ?
      PolicyActionStatus::Accepted : PolicyActionStatus::ActionRejected;
  } catch (const std::exception & e) {
    result.error = e.what();
    result.status = PolicyActionStatus::InferenceFailed;
  }

  return result;
}

bool PolicyRuntime::is_action_change_check_enabled() const
{
  return is_action_boundary_delta_check_enabled_ || is_reference_mimic_ ||
         (locomotion_mimic_ && locomotion_mimic_->has_motion_weight());
}

std::vector<bool> PolicyRuntime::evaluate_raw_action_boundaries(const std::vector<float> & raw)
{
  if (locomotion_mimic_) {
    return locomotion_mimic_->evaluate_raw_action_boundaries(
      obs_buffer_.at(onnx_input_name_), raw, action_safety_limits_.raw_action_boundary,
      [this](const std::vector<float> & input) {return infer_observation(input);});
  }

  std::vector<bool> is_at_boundary(raw.size());

  for (size_t j = 0; j < raw.size(); ++j) {
    is_at_boundary[j] = std::abs(raw[j]) >= action_safety_limits_.raw_action_boundary;
  }

  return is_at_boundary;
}

void PolicyRuntime::validate_action_dimensions(const PolicyActionResult & action) const
{
  const size_t count = joint_context_.policy_joint_names.size();

  if (action.raw.size() != count) {
    throw std::runtime_error(action_size_mismatch_message("Raw", action.raw.size(), count));
  }

  if (action.processed.size() != count) {
    throw std::runtime_error(action_size_mismatch_message("Processed", action.processed.size(), count));
  }
}

void PolicyRuntime::commit_action(const PolicyActionResult & action)
{
  // Only validated targets may update module feedback, last_action or output.
  if (any2track_) {
    any2track_->accept(action.processed);
  }

  if (locomotion_mimic_) {
    locomotion_mimic_->accept();
  }

  if (mirror_) {
    target_command_.position = mirror_->reflect(action.processed);
  } else {
    std::copy(action.processed.begin(), action.processed.end(), target_command_.position.begin());
  }

  // The buffer is controller-sized; this policy uses its first policy-sized slots.
  const auto & applied_raw = any2track_ ? action.raw : action_pipeline_.applied_raw_action();
  std::copy(applied_raw.begin(), applied_raw.end(), policy_->last_action.begin());
  action_reference_ = action.processed;
  consecutive_action_failures_ = 0;
}

void PolicyRuntime::reject_action(PolicyActionStatus status)
{
  // Keep the failed time slot, replacing only its sample with retained feedback.
  // Each module owns its history layout; clocks and motion progression continue.
  if (obs_manager_) {
    obs_manager_->reject_latest();
  }

  if (any2track_) {
    any2track_->reject_latest();
  }

  if (locomotion_mimic_) {
    locomotion_mimic_->reject();
  }

  // Exceptions and invalid targets share one budget, reset only by acceptance.
  ++consecutive_action_failures_;

  if (consecutive_action_failures_ < kMaxConsecutiveActionFailures) {
    return;
  }

  if (status == PolicyActionStatus::ActionRejected) {
    requests_->action_limit_exceeded = true;
  } else {
    requests_->damping = true;
  }
}

const std::string & PolicyRuntime::state_name() const
{
  return state_name_;
}

void PolicyRuntime::load_sim2real_config(
  const Sim2RealConfig & sim2real_config,
  const std::vector<std::string> & controller_joint_names)
{
  joint_context_.policy_joint_names = sim2real_config.policy_joints();
  // Policy joints may be a subset of the robot joints; every policy joint
  // must exist on the robot, but not the other way around.
  joint_context_.policy_to_controller = make_source_indices_for_target(
    controller_joint_names, joint_context_.policy_joint_names, "robot_joint_order",
    "policy_joints");

  step_dt_ = sim2real_config.step_dt();
  accumulated_period_ = step_dt_;

  joint_properties_ = sim2real_config.joint_properties();
  action_pipeline_ = ActionPipeline(sim2real_config.action_properties());

  if (action_pipeline_.size() != joint_context_.policy_joint_names.size()) {
    throw std::runtime_error("Policy state '" + state_name_ +
      "' action size does not match policy_joints");
  }

  if (const auto & ranges = sim2real_config.velocity_command_ranges()) {
    velocity_command_ranges_ = *ranges;
  }
}

void PolicyRuntime::log_joint_coverage(
  const std::vector<std::string> & controller_joint_names) const
{
  const bool covers_all_controller_joints =
    joint_context_.policy_joint_names.size() == controller_joint_names.size();
  if (covers_all_controller_joints) {
    return;
  }

  std::string uncontrolled;
  for (const auto & name : controller_joint_names) {
    if (contains_joint_name(joint_context_.policy_joint_names, name)) {
      continue;
    }

    if (!uncontrolled.empty()) {
      uncontrolled += ", ";
    }

    uncontrolled += name;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[PolicyRuntime] '%s' controls %zu/%zu joints; uncontrolled joints keep their previous "
    "commands and gains (uncontrolled: %s)",
    state_name_.c_str(),
    joint_context_.policy_joint_names.size(),
    controller_joint_names.size(),
    uncontrolled.c_str());
}

void PolicyRuntime::log_loading() const
{
  std::cout << "\n"
            << "[PolicyRuntime] ==================================================\n"
            << "[PolicyRuntime] Loading behavior: " << state_name_ << "\n"
            << "[PolicyRuntime] Model: " << model_path_ << "\n"
            << "[PolicyRuntime] sim2real.yaml: " << sim2real_config_path_.string() << "\n"
            << "[PolicyRuntime] --------------------------------------------------"
            << std::endl;
  RCLCPP_INFO(
    node_->get_logger(),
    "[PolicyRuntime] loading %s from %s",
    state_name_.c_str(),
    model_path_.c_str());
}

void PolicyRuntime::load_onnx_model()
{
  inference_ = std::make_unique<OnnxInference>(model_path_, static_cast<bool>(any2track_));
  const auto & input_names = inference_->get_input_names();
  if (!any2track_ && input_names.size() != 1) {
    throw std::runtime_error(
      "Policy state '" + state_name_ + "' must have exactly one ONNX input");
  }

  onnx_input_name_ = input_names[0];

  // Validate ONNX output size before RT updates.
  const auto output_size = static_cast<size_t>(inference_->get_output_size());
  const auto action_size = action_pipeline_.size();
  if (output_size != action_size) {
    throw std::runtime_error("Policy state '" + state_name_ + "' ONNX output size " +
      std::to_string(output_size) + " does not match action size " +
      std::to_string(action_size));
  }
}

size_t PolicyRuntime::create_observation_manager(
  const Sim2RealConfig & sim2real_config,
  const SharedControlData * shared_data,
  const MotionReference * reference_motion)
{
  obs_manager_ = std::make_unique<ObservationManager>(
    sim2real_config.observations(),
    shared_data,
    joint_context_,
    reference_motion, mirror_ ? &sensor_view_ : nullptr,
    (mirror_ || sim2real_config.observations()["supermimic_future"]) ?
    &joint_properties_.default_position : nullptr,
    locomotion_mimic_ ? &locomotion_mimic_->observations() : nullptr);
  return obs_manager_->get_observation_size();
}

void PolicyRuntime::validate_observation_size(size_t observation_size) const
{
  const auto & input_sizes = inference_->get_input_sizes();
  const int64_t expected_size = input_sizes.empty() ? 0 : input_sizes[0];
  const int64_t actual_size = static_cast<int64_t>(observation_size);

  if (expected_size != actual_size) {
    throw std::runtime_error(
      "Policy state '" + state_name_ + "' ONNX input expects " +
      std::to_string(expected_size) +
      " values, but sim2real observations produce " + std::to_string(observation_size));
  }
}

Any2TrackSample PolicyRuntime::any2track_sample() const
{
  Any2TrackSample sample;
  if (requires_localization_) {sample.xy = shared_data_->localization.position;}
  // Only XY comes from the estimator; attitude and gyro come from the robot IMU.
  sample.orientation = sensors_->orientation;
  sample.gyro = sensors_->angular_velocity;
  for (size_t j = 0; j < joint_context_.policy_to_controller.size(); ++j) {
    const size_t robot = joint_context_.policy_to_controller[j];
    sample.position[j] = sensors_->joint_pos[robot];
    sample.velocity[j] = sensors_->joint_vel[robot];
  }
  return sample;
}

void PolicyRuntime::log_ready(size_t observation_size) const
{
  if (mirror_) {
    RCLCPP_INFO(node_->get_logger(), "[PolicyRuntime] %s: K1 left/right mirror enabled",
      state_name_.c_str());
  }
  std::cout << "[PolicyRuntime] Ready: " << state_name_
            << " (obs=" << observation_size
            << ", action=" << inference_->get_output_size() << ")\n"
            << "[PolicyRuntime] ==================================================\n"
            << std::endl;
}

}  // namespace ai_sapiens_sim2real
