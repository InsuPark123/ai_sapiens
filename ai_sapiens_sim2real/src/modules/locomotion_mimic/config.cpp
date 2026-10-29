// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "ai_sapiens_sim2real/modules/locomotion_mimic/config.hpp"
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
namespace ai_sapiens_sim2real
{
namespace
{
locomotion_mimic::Clip load_clip(
  const std::string & path, int waist_index, float dt,
  const std::vector<std::string> & joints)
{
  // Use the shared CSV parser and frame storage, with this export's derivative ABI.
  MotionReference motion(path, 1.0f / dt, joints,
    MotionReference::Derivative::Forward, false);
  if (motion.frame_count() < 2) {
    throw std::runtime_error("mimic_gate requires at least two frames");
  }
  if (motion.joint_order() != joints) {
    throw std::runtime_error("mimic_gate clip joint order does not match policy");
  }
  locomotion_mimic::Clip clip;
  for (int i = 0; i < motion.frame_count(); ++i) {
    const auto frame = motion.frame(i);
    clip.q.push_back(frame.joint_pos);
    clip.dq.push_back(frame.joint_vel);
    clip.root.push_back(frame.root_quaternion);
    const Eigen::Quaternionf torso = frame.root_quaternion *
      Eigen::AngleAxisf(frame.joint_pos[waist_index], Eigen::Vector3f::UnitZ());
    clip.torso_tilt.push_back(std::acos(std::clamp(torso.toRotationMatrix()(2, 2), -1.f, 1.f)));
  }
  return clip;
}
}  // namespace
namespace
{
  class ConfigReader
  {
public:
    ConfigReader(
      const YAML::Node & term, const std::filesystem::path & path,
      double dt, const std::vector<std::string> & names)
    : cfg_(term["params"]), policy_config_(path), step_dt_(dt), names_(names) {}

    LocomotionMimicConfig read()
    {
      validate_contract();
      read_timing_and_trigger();
      load_clips();
      read_clip_options();
      read_interactive();
      read_dial_clips();
      read_loops();
      validate_parameters();
      return std::move(result_);
    }

private:
    YAML::Node cfg_;
    std::filesystem::path policy_config_;
    double step_dt_;
    const std::vector<std::string> & names_;
    LocomotionMimicConfig result_;
    void validate_parameters() const
    {
      const auto & p = result_.parameters;
      for (float value : {p.quiet_seconds, p.pending_timeout_seconds, p.quiet_command,
          p.quiet_leg_speed, p.quiet_ang_speed, p.abort_max_reference_tilt})
      {
        if (!std::isfinite(value) || value < 0.f) {
          throw std::runtime_error("mimic_gate thresholds must be finite and non-negative");
        }
      }
      for (float value : {p.ramp_seconds, p.bridge_min_seconds, p.bridge_max_seconds,
          p.bridge_joint_speed, p.bridge_torso_speed})
      {
        if (!std::isfinite(value) || value <= 0.f) {
          throw std::runtime_error(
              "mimic_gate durations and bridge speeds must be finite and positive");
        }
      }
      if (p.bridge_min_seconds > p.bridge_max_seconds) {
        throw std::runtime_error("mimic_gate bridge_min_seconds exceeds bridge_max_seconds");
      }
      for (float value : p.acceleration_limits) {
        if (!std::isfinite(value) || value < 0.f) {
          throw std::runtime_error(
              "mimic_gate acceleration limits must be finite and non-negative");
        }
      }
      if (p.inter_clip >= 0 &&
        (p.inter_hold_frame >= result_.clips.at(p.inter_clip).frames() ||
        p.inter_low_frame >= result_.clips.at(p.inter_low_clip).frames()))
      {
        throw std::runtime_error("mimic_gate interactive frame outside clip");
      }
    }

    void validate_contract()
    {
      const auto & names = names_;
      const auto & cfg = cfg_;
      const auto step_dt = step_dt_;

      const std::vector<std::string> expected = {
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint"};
      if (names != expected || !std::isfinite(step_dt) || std::abs(step_dt - 0.02) > 1e-9) {
        throw std::runtime_error("mimic_gate requires the K1 23-joint 50 Hz policy order");
      }
      if (!cfg || !cfg["clips"] || !cfg["clips"].IsSequence() || cfg["clips"].size() == 0) {
        throw std::runtime_error("mimic_gate needs params.clips");
      }
    }

    void read_timing_and_trigger()
    {
      auto & result = result_;
      auto & p = result_.parameters;
      const auto & cfg = cfg_;
      const auto step_dt = step_dt_;
      result.trigger_channel = cfg["trigger_channel"].as<int>();
      result.trigger_min = cfg["trigger_on_min_us"].as<int>();
      result.trigger_max = cfg["trigger_on_max_us"].as<int>();
      result.dial_base = cfg["dial_base"].as<int>();
      if (result.trigger_channel < 1 || result.trigger_channel > 16 || result.trigger_min < 500 ||
        result.trigger_max > 2500 ||
        result.trigger_min > result.trigger_max)
      {
        throw std::runtime_error("Invalid mimic_gate trigger channel / PWM bounds");
      }
      p.dt = static_cast<float>(step_dt);
      p.abort_on_release = cfg["abort_on_release"] && cfg["abort_on_release"].as<bool>();
      const auto opt = [&](const char * key, float & value) {
          if (cfg[key]) {
            value = cfg[key].as<float>();
          }
        };
      opt("quiet_seconds", p.quiet_seconds);
      opt("pending_timeout_seconds", p.pending_timeout_seconds);
      opt("ramp_seconds", p.ramp_seconds);
      opt("bridge_min_seconds", p.bridge_min_seconds);
      opt("bridge_max_seconds", p.bridge_max_seconds);
      opt("bridge_joint_speed", p.bridge_joint_speed);
      opt("bridge_torso_speed", p.bridge_torso_speed);
      opt("abort_max_reference_tilt", p.abort_max_reference_tilt);
      opt("quiet_command", p.quiet_command);
      opt("quiet_leg_speed", p.quiet_leg_speed);
      opt("quiet_ang_speed", p.quiet_ang_speed);
      if (cfg["acceleration_limits"]) {
        for (int i = 0; i < 3; ++i) {
          p.acceleration_limits[i] = cfg["acceleration_limits"][i].as<float>();
        }
      }
    }

    void load_clips()
    {
      auto & result = result_;
      auto & p = result_.parameters;
      auto & clips = result_.clips;
      const auto & names = names_;
      const auto & cfg = cfg_;
      const auto & policy_config = policy_config_;
      const auto params_dir = policy_config.parent_path();
      for (size_t i = 0; i < cfg["clips"].size(); ++i) {
        const auto path = params_dir / cfg["clips"][i].as<std::string>();
        clips.push_back(load_clip(path.string(), p.waist_index, p.dt, names));
        result.clip_names.push_back(
        cfg["clip_names"] &&
          i <
          cfg["clip_names"].size() ? cfg["clip_names"][i].as<std::string>() : path.stem().string());
      }
    }

    void read_clip_options()
    {
      auto & result = result_;
      auto & p = result_.parameters;
      auto & clips = result_.clips;
      const auto & cfg = cfg_;

      const auto flags = [&](const char * key) {
          std::vector<bool> out(result.clip_names.size(), false);
          if (!cfg[key]) {return out;}
          for (const auto & n : cfg[key]) {
            const auto it = std::find(result.clip_names.begin(), result.clip_names.end(),
            n.as<std::string>());
            if (it == result.clip_names.end()) {
              throw std::runtime_error(std::string("mimic_gate ") + key + ": unknown clip " +
              n.as<std::string>());
            }
            out[static_cast<size_t>(it - result.clip_names.begin())] = true;
          }
          return out;
        };
      p.walk_allowed = flags("walk_clips");
      p.arms_only_ref = flags("arms_only_ref_clips");
      p.fast_start = flags("fast_start_clips");
      p.hold_frames.assign(clips.size(), -1);
      if (const auto hold_clips = cfg["hold_clips"]) {
        if (!hold_clips.IsMap()) {throw std::runtime_error("mimic_gate hold_clips must be a map");}
        for (const auto & item : hold_clips) {
          const auto name = item.first.as<std::string>();
          const auto it = std::find(result.clip_names.begin(), result.clip_names.end(), name);
          if (it == result.clip_names.end()) {
            throw std::runtime_error("mimic_gate hold_clips: unknown clip " + name);
          }
          if (!item.second.IsSequence() || item.second.size() != 1U) {
            throw std::runtime_error("mimic_gate hold_clips needs one frame per clip: " + name);
          }
          const auto index = static_cast<size_t>(it - result.clip_names.begin());
          const int frame = item.second[0].as<int>();
          if (frame < 0 || frame >= clips[index].frames() - 1) {
            throw std::runtime_error("mimic_gate hold_clips frame out of range: " + name);
          }
          p.hold_frames[index] = frame;
        }
      }
      if (cfg["hold_max_seconds"]) {p.hold_max_seconds = cfg["hold_max_seconds"].as<float>();}
      if (!std::isfinite(p.hold_max_seconds) || p.hold_max_seconds < 0.f) {
        throw std::runtime_error("Invalid mimic_gate hold_max_seconds");
      }
      if (cfg["switch_walk_seconds"]) {
        p.switch_walk_seconds = cfg["switch_walk_seconds"].as<float>();
      }
      if (!(p.switch_walk_seconds >= 0.f && p.switch_walk_seconds <= 2.f)) {
        throw std::runtime_error("Invalid switch_walk_seconds");
      }
      if (cfg["fast_start_ramp_seconds"]) {
        p.fast_ramp_seconds = cfg["fast_start_ramp_seconds"].as<float>();
      }
      if (!(p.fast_ramp_seconds >= 0.f && p.fast_ramp_seconds <= p.ramp_seconds)) {
        throw std::runtime_error("Invalid fast_start_ramp_seconds");
      }
    }

    void read_interactive()
    {
      auto & result = result_;
      auto & p = result_.parameters;
      const auto & cfg = cfg_;
      if (const auto inter = cfg["interactive"]) {
        const auto index_of = [&](const std::string & n) {
            const auto it = std::find(result.clip_names.begin(), result.clip_names.end(), n);
            if (it == result.clip_names.end()) {
              throw std::runtime_error("mimic_gate interactive: unknown clip " + n);
            }
            return static_cast<int>(it - result.clip_names.begin());
          };
        p.inter_clip = index_of(inter["clip"].as<std::string>());
        p.inter_hold_frame = inter["hold_frame"].as<int>();
        p.inter_low_clip = index_of(inter["low_clip"].as<std::string>());
        p.inter_low_frame = inter["low_frame"].as<int>();
        result.select_channel = inter["select_channel"].as<int>();
        result.select_low_max = inter["select_low_max_us"].as<int>();
        result.select_high_min = inter["select_high_min_us"].as<int>();
        result.confirm_channel = inter["confirm_channel"].as<int>();
        p.confirm_is_trigger = result.confirm_channel == result.trigger_channel;
        if (result.select_channel < 1 || result.select_channel > 16 || result.confirm_channel < 1 ||
          result.confirm_channel > 16 || result.select_low_max >= result.select_high_min ||
          p.inter_hold_frame < 0 || p.inter_low_frame < 0)
        {
          throw std::runtime_error("Invalid mimic_gate interactive config");
        }
      }
    }

    void read_dial_clips()
    {
      auto & result = result_;
      const auto & cfg = cfg_;

      result.dial_clips.clear();
      if (const auto dial_clips = cfg["dial_clips"]) {
        for (const auto & n : dial_clips) {
          const auto it = std::find(result.clip_names.begin(), result.clip_names.end(),
            n.as<std::string>());
          if (it == result.clip_names.end()) {
            throw std::runtime_error("mimic_gate dial_clips: unknown clip " + n.as<std::string>());
          }
          result.dial_clips.push_back(static_cast<int>(it - result.clip_names.begin()));
        }
      }
    }

    void read_loops()
    {
      auto & result = result_;
      auto & p = result_.parameters;
      const auto & cfg = cfg_;

      if (const auto dial_loops = cfg["dial_loops"]) {
        if (result.dial_clips.empty()) {
          throw std::runtime_error("mimic_gate dial_loops needs dial_clips");
        }
        for (const auto & seq : dial_loops) {
          std::vector<int> ids;
          for (const auto & n : seq) {
            const auto it = std::find(result.clip_names.begin(), result.clip_names.end(),
              n.as<std::string>());
            if (it == result.clip_names.end()) {
              throw std::runtime_error("mimic_gate dial_loops: unknown clip " +
                  n.as<std::string>());
            }
            ids.push_back(static_cast<int>(it - result.clip_names.begin()));
          }
          if (ids.empty()) {throw std::runtime_error("mimic_gate dial_loops: empty loop");}
          p.loops.push_back(ids);
        }
      }
    }
  };
}  // namespace
LocomotionMimicConfig LocomotionMimicConfig::read(
  const YAML::Node & term,
  const std::filesystem::path & policy_config, double step_dt,
  const std::vector<std::string> & names)
{
  return ConfigReader(term, policy_config, step_dt, names).read();
}
}  // namespace ai_sapiens_sim2real
