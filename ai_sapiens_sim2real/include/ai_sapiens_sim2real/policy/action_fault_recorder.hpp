// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#ifndef AI_SAPIENS_SIM2REAL__POLICY__ACTION_FAULT_RECORDER_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__ACTION_FAULT_RECORDER_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include "ai_sapiens_sim2real/observation/observation_layout.hpp"
#include "ai_sapiens_sim2real/shared_control_data.hpp"

namespace ai_sapiens_sim2real
{
struct FaultPolicyMetadata
{
  std::string state, model, config_path, config_yaml;
  std::vector<std::string> policy_joints, controller_joints, input_names;
  std::vector<std::vector<int64_t>> input_shapes;
  std::unordered_map<std::string, std::vector<ObservationSlice>> layouts;
  double step_dt{0.0};
  float action_limit{0.0f};
  bool is_mirrored{false};
};

// The specialized Any2Track ABI has two different history layouts.
std::unordered_map<std::string, std::vector<ObservationSlice>> any2track_fault_layout(size_t obs_size);

struct FaultInferenceCall
{
  std::unordered_map<std::string, std::vector<float>> inputs;
  std::vector<float> output;
};

struct FaultSnapshot
{
  int64_t unix_time_ns{0};
  double episode_time{0.0}, sensor_interval{0.0};
  SensorData sensors;
  std::optional<SensorData> previous_sensors;
  std::vector<FaultInferenceCall> calls;
  std::vector<float> processed_action;
  std::string error, outcome;
  size_t consecutive_failures{0};
};

struct ActionFaultRecord
{
  std::shared_ptr<const FaultPolicyMetadata> metadata;
  FaultSnapshot current;
  std::optional<FaultSnapshot> previous_accepted;
  uint64_t incident{0};
  size_t attempt{0};
  std::filesystem::path path;
  size_t queued_bytes{0};
};

// One writer per controller, shared by all loaded policies. No filesystem IO
// or serialization in enqueue(); an unavailable queue drops diagnostics only.
class ActionFaultRecorder
{
public:
  struct Options
  {
    std::filesystem::path directory;
    size_t max_storage_bytes{256 * 1024 * 1024};
    size_t max_queue_bytes{32 * 1024 * 1024};
    size_t max_queue_records{16};
  };
  ActionFaultRecorder(Options options, rclcpp::Logger logger);
  ~ActionFaultRecorder();
  static std::filesystem::path default_directory();
  uint64_t next_incident() {return ++incident_sequence_;}
  std::string enqueue(ActionFaultRecord record) noexcept;
  const std::filesystem::path & session_directory() const {return session_directory_;}

private:
  void write_loop() noexcept;
  void write_record(const ActionFaultRecord & record);
  Options options_;
  rclcpp::Logger logger_;
  std::filesystem::path session_directory_;
  std::atomic<uint64_t> incident_sequence_{0};
  std::atomic<bool> is_enabled_{true};
  std::atomic<size_t> dropped_records_{0};
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<ActionFaultRecord> queue_;
  size_t queued_bytes_{0}, stored_bytes_{0};
  bool is_stopping_{false};
  std::thread worker_;
};

// Control-thread capture, independent of action validation and retry decisions.
// Copies each actual ONNX call (including both Loco Mimic calls) before inference.
class PolicyFaultCapture
{
public:
  PolicyFaultCapture(std::shared_ptr<ActionFaultRecorder> recorder,
    std::shared_ptr<const FaultPolicyMetadata> metadata, rclcpp::Logger logger);
  void reset() noexcept;
  void begin(double episode_time, const SensorData & sensors,
    const std::optional<SensorData> & previous, double interval) noexcept;
  void input(const std::unordered_map<std::string, std::vector<float>> & inputs) noexcept;
  void output(const std::vector<float> & output) noexcept;
  void error(const char * reason) noexcept;
  std::string finish(bool is_accepted, bool is_damping, size_t failures,
    const std::vector<float> & processed) noexcept;

private:
  void disable() noexcept;
  std::shared_ptr<ActionFaultRecorder> recorder_;
  std::shared_ptr<const FaultPolicyMetadata> metadata_;
  rclcpp::Logger logger_;
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
  FaultSnapshot current_;
  std::optional<FaultSnapshot> previous_accepted_;
  uint64_t incident_{0};
  size_t attempt_{0};
  bool is_enabled_{true};
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__POLICY__ACTION_FAULT_RECORDER_HPP_
