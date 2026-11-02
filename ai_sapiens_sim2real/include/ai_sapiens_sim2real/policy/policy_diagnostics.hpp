// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#ifndef AI_SAPIENS_SIM2REAL__POLICY__POLICY_DIAGNOSTICS_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__POLICY_DIAGNOSTICS_HPP_

#include "ai_sapiens_sim2real/policy/action_fault_recorder.hpp"
#include "ai_sapiens_sim2real/policy/policy_action_result.hpp"

namespace ai_sapiens_sim2real
{
// Owns diagnostic context and presentation, never control or retry decisions.
// Terminal diagnostics remain available without a file recorder.
class PolicyDiagnostics
{
public:
  PolicyDiagnostics(FaultPolicyMetadata metadata, size_t max_failures, rclcpp::Logger logger);
  void set_recorder(std::shared_ptr<ActionFaultRecorder> recorder) noexcept;
  void reset() noexcept;
  void advance(double period) noexcept;
  void begin(double episode_time, const SensorData & sensors) noexcept;
  void input(const std::unordered_map<std::string, std::vector<float>> & inputs) noexcept;
  void output(const std::vector<float> & output) noexcept;
  void finish(const PolicyActionResult & result, const SensorData & sensors,
    size_t previous_failures, size_t failures) noexcept;

private:
  void log_result(const PolicyActionResult & result, const SensorData & sensors,
    size_t previous_failures, size_t failures, const std::string & path) const;
  void log_action_limit(const PolicyActionResult & result, const SensorData & sensors,
    size_t failures, const std::string & path) const;
  std::shared_ptr<const FaultPolicyMetadata> metadata_;
  size_t max_failures_;
  rclcpp::Logger logger_;
  std::unique_ptr<PolicyFaultCapture> capture_;
  std::optional<SensorData> previous_sensors_;
  double sensor_interval_{0.0};
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__POLICY_DIAGNOSTICS_HPP_
