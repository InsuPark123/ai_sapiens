#ifndef AI_SAPIENS_SIM2REAL__POLICY__MIMIC_HEADING_ALIGNMENT_HPP_
#define AI_SAPIENS_SIM2REAL__POLICY__MIMIC_HEADING_ALIGNMENT_HPP_

#include <cmath>
#include <Eigen/Geometry>

namespace ai_sapiens_sim2real
{
// Best world-Z rotation aligning two torso orientations. Unlike subtracting
// their Euler yaws, this remains defined when both forward axes are vertical
// (supine/prone). Keep roll/pitch tracking error; do not align full attitude.
inline Eigen::Quaternionf mimic_heading_alignment(
  const Eigen::Quaternionf & reference, const Eigen::Quaternionf & actual)
{
  const Eigen::Quaternionf delta = actual.normalized() * reference.normalized().conjugate();
  const float norm = std::hypot(delta.w(), delta.z());
  // Opposing tilt makes yaw unobservable. Deterministic identity is preferable
  // to amplifying noise; such a pose is not a valid matching entry posture.
  if (norm < 1.0e-6f) {
    return Eigen::Quaternionf::Identity();
  }
  return Eigen::Quaternionf(delta.w() / norm, 0.0f, 0.0f, delta.z() / norm);
}
// Match the SuperMimic simulation adapter's difference of torso Euler yaws.
inline Eigen::Quaternionf supermimic_heading_alignment(
  const Eigen::Quaternionf & reference, const Eigen::Quaternionf & actual)
{
  const auto yaw = [](const Eigen::Quaternionf & input) {
    const auto q = input.normalized();
    return std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
      1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
  };
  return Eigen::Quaternionf(Eigen::AngleAxisf(yaw(actual) - yaw(reference), Eigen::Vector3f::UnitZ()));
}
}  // namespace ai_sapiens_sim2real

#endif
