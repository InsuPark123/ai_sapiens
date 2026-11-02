// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "ai_sapiens_sim2real/policy/policy_diagnostics.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <rclcpp/logging.hpp>
#include "ai_sapiens_sim2real/policy/action_fault_diagnostics.hpp"

namespace ai_sapiens_sim2real
{
PolicyDiagnostics::PolicyDiagnostics(
  FaultPolicyMetadata metadata, size_t max_failures, rclcpp::Logger logger)
: metadata_(std::make_shared<FaultPolicyMetadata>(std::move(metadata))),
  max_failures_(max_failures), logger_(std::move(logger)) {}

void PolicyDiagnostics::set_recorder(std::shared_ptr<ActionFaultRecorder> recorder) noexcept
{
  capture_.reset();

  if (!recorder) {return;}

  // Configuration IO happens only during startup attachment, never in update().
  try {
    auto metadata = std::make_shared<FaultPolicyMetadata>(*metadata_);
    std::ifstream config(metadata->config_path);
    metadata->config_yaml.assign(std::istreambuf_iterator<char>(config), {});
    capture_ = std::make_unique<PolicyFaultCapture>(
      std::move(recorder), std::move(metadata), logger_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Cannot prepare fault observation capture: %s", e.what());
  }
}

void PolicyDiagnostics::reset() noexcept
{
  if (capture_) {capture_->reset();}
  previous_sensors_.reset();
  sensor_interval_ = 0.0;
}

void PolicyDiagnostics::advance(double period) noexcept
{
  if (std::isfinite(period) && period > 0.0) {sensor_interval_ += period;}
}

void PolicyDiagnostics::begin(double episode_time, const SensorData & sensors) noexcept
{
  if (capture_) {capture_->begin(episode_time, sensors, previous_sensors_, sensor_interval_);}
}

void PolicyDiagnostics::input(
  const std::unordered_map<std::string, std::vector<float>> & inputs) noexcept
{
  if (capture_) {capture_->input(inputs);}
}

void PolicyDiagnostics::output(const std::vector<float> & output) noexcept
{
  if (capture_) {capture_->output(output);}
}

void PolicyDiagnostics::finish(const PolicyActionResult & result, const SensorData & sensors,
  size_t previous_failures, size_t failures) noexcept
{
  try {
    std::string path;

    if (capture_) {
      if (!result.error.empty()) {capture_->error(result.error.c_str());}
      path = capture_->finish(result.is_accepted(), failures >= max_failures_,
        failures, result.processed, result.validation);
    }

    log_result(result, sensors, previous_failures, failures, path);
    previous_sensors_ = sensors;
    sensor_interval_ = 0.0;
  } catch (...) {
    // Diagnostic allocation/formatting failures must never change control.
    previous_sensors_.reset();
    sensor_interval_ = 0.0;
    RCLCPP_ERROR(logger_, "Cannot report policy diagnostics; control continues");
  }
}

void PolicyDiagnostics::log_result(const PolicyActionResult & result, const SensorData & sensors,
  size_t previous_failures, size_t failures, const std::string & path) const
{
  if (result.status == PolicyActionStatus::ActionRejected) {
    log_action_limit(result, sensors, failures, path);
    return;
  }

  if (result.status == PolicyActionStatus::InferenceFailed) {
    RCLCPP_WARN(logger_,
      "Policy '%s' inference failed (%zu/%zu consecutive unusable actions): %s",
      metadata_->state.c_str(), failures, max_failures_, result.error.c_str());

    if (failures >= max_failures_) {
      RCLCPP_ERROR(logger_,
        "Policy '%s' failed to produce a usable action; requesting damping failsafe",
        metadata_->state.c_str());
    }
  } else if (previous_failures != 0) {
    RCLCPP_INFO(logger_,
      "Policy '%s' recovered after %zu unusable action(s); accepting target and resetting retries",
      metadata_->state.c_str(), previous_failures);
  }

  if (!path.empty()) {RCLCPP_INFO(logger_, "Inference diagnostics queued: %s", path.c_str());}
}

void PolicyDiagnostics::log_action_limit(
  const PolicyActionResult & result, const SensorData & sensors,
  size_t failures, const std::string & path) const
{
  std::ostringstream report;
  report << "ACTION LIMIT | " << failures << "/" << max_failures_
         << " | " << (failures >= max_failures_ ? "DAMPING" : "HOLD; retry next tick")
         << "\n\n--- ACTION: rejected target ---\n"
         << "Processed position [rad, policy coordinates]; change limit="
         << metadata_->action_change_limit * 180.0f / 3.14159265f << " deg"
         << " AND |raw action| >= " << metadata_->raw_action_boundary
         << " (" << boundary_check_label(result.validation.boundary_check) << ")\n"
         << "  " << std::left << std::setw(16) << "joint"
         << " " << std::right << std::setw(10) << "processed" << "   previous      delta flag\n"
         << format_action_fault_actions(result.processed, metadata_->policy_joints, result.validation, true);
  const auto jumps = format_action_fault_sensors(sensors,
    previous_sensors_, metadata_->controller_joints, sensor_interval_, true);

  if (!jumps.empty()) {
    report << "\n--- SENSOR: feedback jumps ---\n"
           << "Physical sensor coordinates; JUMP only\n" << jumps;
  }

  report << "\n[FILE] " << (path.empty() ? "not queued (recording unavailable)" : path + " (queued)");

  if (failures >= max_failures_) {
    RCLCPP_ERROR(logger_, "%s", report.str().c_str());
  } else {
    RCLCPP_WARN(logger_, "%s", report.str().c_str());
  }
}

}  // namespace ai_sapiens_sim2real
