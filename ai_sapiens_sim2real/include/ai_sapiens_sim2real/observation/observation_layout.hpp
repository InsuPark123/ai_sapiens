// Copyright 2026 ROBOTIS CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#ifndef AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_LAYOUT_HPP_
#define AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_LAYOUT_HPP_

#include <cstddef>
#include <string>

namespace ai_sapiens_sim2real
{
// Read-only indices into the actual ONNX tensor, after scaling, clipping and
// history replacement. Frames are ordered oldest to newest.
struct ObservationSlice
{
  std::string name;
  size_t offset;
  size_t width;
  size_t frames;
  size_t frame_stride;
  size_t element_stride{1};
};
}  // namespace ai_sapiens_sim2real
#endif  // AI_SAPIENS_SIM2REAL__OBSERVATION__OBSERVATION_LAYOUT_HPP_
