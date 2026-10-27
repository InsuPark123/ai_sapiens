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

#include "ai_sapiens_sim2real/policy/gait_clock.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{

void GaitClock::advance(PhaseState & state, const SharedControlData & shared) const
{
  state = update_(state, shared);
}

namespace
{

float next_phase(float phase, float phase_increment)
{
  return std::fmod(phase + phase_increment, 1.0f);
}

GaitClock make_free_running_phase_clock(float increment)
{
  return GaitClock([increment](PhaseState state, const SharedControlData &) {
      state.phase = next_phase(state.phase, increment);
      return state;
    });
}

GaitClock make_command_reset_phase_clock(float increment, float threshold)
{
  return GaitClock([increment, threshold](PhaseState state, const SharedControlData & shared) {
      // DWAQ's first observation is at phase zero. This is a rule detail,
      // not a branch in the common clock.
      if (state.first_step) {
        state.first_step = false;
        return state;
      }
      state.phase = shared.mode.velocity_commands.norm() > threshold ?
        next_phase(state.phase, increment) : 0.0f;
      return state;
    });
}

struct AdaptivePhaseConfig
{
  std::array<float, 3> periods;
  std::array<float, 2> forward_thresholds;
  std::array<float, 3> swing_ratios;
  float transition_tau;
  float reset_phase;
};

template<typename T>
T read_phase_value(const YAML::Node & params, const char * key)
{
  if (!params[key]) {
    throw std::runtime_error("Missing gait_phase." + std::string(key));
  }
  try {
    return params[key].as<T>();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("Invalid gait_phase." + std::string(key) + ": " + error.what());
  }
}

float read_phase_scalar(const YAML::Node & params, const char * key)
{
  const float value = read_phase_value<float>(params, key);
  if (!std::isfinite(value)) {
    throw std::runtime_error("gait_phase." + std::string(key) + " must be finite");
  }
  return value;
}

template<std::size_t N>
std::array<float, N> read_phase_array(const YAML::Node & params, const char * key)
{
  const auto values = read_phase_value<std::vector<float>>(params, key);
  if (values.size() != N) {
    throw std::runtime_error("gait_phase." + std::string(key) +
            " must contain " + std::to_string(N) + " values");
  }
  std::array<float, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    if (!std::isfinite(values[i])) {
      throw std::runtime_error("gait_phase." + std::string(key) +
              "[" + std::to_string(i) + "] must be finite");
    }
    result[i] = values[i];
  }
  return result;
}

void validate_adaptive_phase_export(const YAML::Node & params)
{
  if (params["schema"]) {
    const auto schema = read_phase_value<std::string>(params, "schema");
    if (schema != "k1_settle_march_phase_v1_command_only" &&
      schema != "k1_settle_march_phase_v1")
    {
      throw std::runtime_error("Unsupported gait_phase.schema: " + schema);
    }
  }
  for (const auto key : {"min_height", "min_upright", "recovery_delay_mask",
    "deployment_walk_period_scale"})
  {
    if (params[key]) {throw std::runtime_error("Unsupported gait_phase." + std::string(key));}
  }
  if (read_phase_value<std::string>(params, "update") != "integrated_free_running") {
    throw std::runtime_error("gait_phase.update must be integrated_free_running");
  }
  if (!read_phase_value<bool>(params, "stand_mode_priority")) {
    throw std::runtime_error("gait_phase.stand_mode_priority must be true");
  }
  const std::vector<std::string> order = {
    "sin_left", "cos_left", "sin_right", "cos_right", "swing_left", "swing_right"};
  if (read_phase_value<std::vector<std::string>>(params, "observation_order") != order) {
    throw std::runtime_error("gait_phase.observation_order must be "
            "[sin_left, cos_left, sin_right, cos_right, swing_left, swing_right]");
  }
  // Validate the observation contract here; observation encoding stays outside the clock.
  for (float offset : read_phase_array<2>(params, "offsets")) {
    if (offset < 0 || offset >= 1) {
      throw std::runtime_error("gait_phase.offsets must be in [0, 1)");
    }
  }
  if (read_phase_scalar(params, "stand_command_norm_plus_yaw_threshold") < 0) {
    throw std::runtime_error("gait_phase.stand_command_norm_plus_yaw_threshold must be >= 0");
  }
}

AdaptivePhaseConfig parse_adaptive_phase_config(const YAML::Node & params)
{
  validate_adaptive_phase_export(params);
  const AdaptivePhaseConfig config{
    read_phase_array<3>(params, "periods"),
    read_phase_array<2>(params, "forward_thresholds"),
    read_phase_array<3>(params, "swing_ratios"),
    read_phase_scalar(params, "transition_tau"),
    read_phase_scalar(params, "reset_phase")};
  for (float period : config.periods) {
    if (period <= 0) {throw std::runtime_error("gait_phase.periods must be positive");}
  }
  for (float swing : config.swing_ratios) {
    if (swing <= 0 || swing >= 1) {
      throw std::runtime_error("gait_phase.swing_ratios must be in (0, 1)");
    }
  }
  if (config.forward_thresholds[0] < 0 ||
    config.forward_thresholds[0] >= config.forward_thresholds[1])
  {
    throw std::runtime_error("gait_phase.forward_thresholds must be nonnegative and strictly increasing");
  }
  if (config.transition_tau <= 0) {
    throw std::runtime_error("gait_phase.transition_tau must be positive");
  }
  if (config.reset_phase < 0 || config.reset_phase >= 1) {
    throw std::runtime_error("gait_phase.reset_phase must be in [0, 1)");
  }
  return config;
}

PhaseState advance_adaptive_phase(
  PhaseState state, float forward_command, const AdaptivePhaseConfig & config,
  float dt, float alpha)
{
  if (state.first_step) {
    state.first_step = false;
    return state;
  }
  // Each threshold is the inclusive lower bound of the next speed range.
  const int category = (forward_command >= config.forward_thresholds[0]) +
    (forward_command >= config.forward_thresholds[1]);
  state.period += alpha * (config.periods[category] - state.period);
  state.swing_ratio += alpha * (config.swing_ratios[category] - state.swing_ratio);
  state.phase = std::fmod(state.phase + dt / state.period, 1.0f);
  return state;
}

GaitClock make_adaptive_phase_clock(const YAML::Node & params, double step_dt)
{
  const auto config = parse_adaptive_phase_config(params);
  const PhaseState initial{config.reset_phase, config.periods[0], config.swing_ratios[0], true};
  const float dt = static_cast<float>(step_dt);
  const float alpha = 1.0f - std::exp(-dt / config.transition_tau);
  return GaitClock([config, dt, alpha](PhaseState state, const SharedControlData & shared) {
      return advance_adaptive_phase(state, shared.mode.velocity_commands.x(), config, dt, alpha);
    }, initial, /*resolve_command_before_update=*/true);
}

float require_period(const YAML::Node & params)
{
  if (!params || params.IsNull()) {
    throw std::runtime_error("gait_phase observation requires params");
  }
  if (!params.IsMap()) {
    throw std::runtime_error("gait_phase observation params must be a map");
  }

  const auto period_node = params["period"];
  if (!period_node || period_node.IsNull()) {
    throw std::runtime_error("gait_phase observation requires params.period (seconds per cycle)");
  }

  const float period = period_node.as<float>();
  if (!std::isfinite(period) || period <= 0.0f) {
    throw std::runtime_error("gait_phase period must be > 0");
  }

  return period;
}

std::optional<float> command_threshold(const YAML::Node & params)
{
  const auto threshold_node = params["command_threshold"];
  if (!threshold_node || threshold_node.IsNull()) {
    return std::nullopt;
  }

  const float threshold = threshold_node.as<float>();
  if (!std::isfinite(threshold) || threshold < 0.0f) {
    throw std::runtime_error("gait_phase command_threshold must be finite and >= 0");
  }
  return threshold;
}

void validate_offset(const YAML::Node & params)
{
  const auto offset_node = params["offset"];
  if (!offset_node || offset_node.IsNull()) {
    return;
  }

  const float offset = offset_node.as<float>();
  if (!std::isfinite(offset) || offset < 0.0f || offset >= 1.0f) {
    throw std::runtime_error("gait_phase offset must be finite and in [0, 1)");
  }
}

}  // namespace

std::optional<GaitClock> make_gait_clock(const YAML::Node & observations, double step_dt)
{
  // Existing exports call this observation "gait_phase"; retain that YAML ABI.
  const auto term = observations["gait_phase"];
  if (!term) {
    return std::nullopt;
  }

  const auto params = term["params"];
  if (!std::isfinite(step_dt) || step_dt <= 0) {
    throw std::runtime_error("gait_phase requires positive finite step_dt");
  }
  if (!params || !params.IsMap()) {throw std::runtime_error("gait_phase requires params map");}
  if (params["periods"] || params["schema"]) {
    return make_adaptive_phase_clock(params, step_dt);
  }
  const float increment = static_cast<float>(step_dt) / require_period(params);
  validate_offset(params);

  const auto threshold = command_threshold(params);
  if (threshold.has_value()) {
    // DWAQ evaluates its first observation at phase zero, then advances by one
    // policy step. Keep that ordering even though PolicyRuntime calls advance()
    // immediately before constructing the observation.
    return make_command_reset_phase_clock(increment, *threshold);
  }

  return make_free_running_phase_clock(increment);
}

}  // namespace ai_sapiens_sim2real
