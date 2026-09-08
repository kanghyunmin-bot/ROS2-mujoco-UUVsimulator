// Copyright 2026 AUV Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CAPTURE_STAMP_SELECTOR_HPP_
#define CAPTURE_STAMP_SELECTOR_HPP_

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace auv_imx219_camera
{

class CaptureStampSelector
{
public:
  enum class Correction
  {
    kNone,
    kFramePeriodCapped,
    kMinimumIncrement,
  };

  struct Selection
  {
    std::int64_t stamp_ns;
    Correction correction;
    std::uint64_t correction_count;
  };

  explicit CaptureStampSelector(std::int64_t frame_period_ns)
  : frame_period_ns_(frame_period_ns)
  {
    if (frame_period_ns_ <= 0) {
      throw std::invalid_argument("frame_period_ns must be greater than zero");
    }
  }

  Selection select_valid(std::int64_t candidate_ns, std::int64_t ros_now_ns)
  {
    if (!has_last_stamp_ || candidate_ns > last_stamp_ns_) {
      return commit(candidate_ns, Correction::kNone);
    }

    return select_next_frame(ros_now_ns, Correction::kFramePeriodCapped);
  }

  Selection select_fallback(std::int64_t ros_now_ns)
  {
    if (!has_last_stamp_) {
      return commit(ros_now_ns, Correction::kNone);
    }

    return select_next_frame(ros_now_ns, Correction::kNone);
  }

private:
  Selection select_next_frame(
    std::int64_t ros_now_ns, Correction advancing_correction)
  {
    const std::int64_t maximum = std::numeric_limits<std::int64_t>::max();
    const std::int64_t next_frame_ns =
      last_stamp_ns_ > maximum - frame_period_ns_ ?
      maximum : last_stamp_ns_ + frame_period_ns_;
    const std::int64_t capped_next_frame_ns = std::min(next_frame_ns, ros_now_ns);
    if (capped_next_frame_ns > last_stamp_ns_) {
      return commit(capped_next_frame_ns, advancing_correction);
    }

    if (last_stamp_ns_ == maximum) {
      throw std::overflow_error("camera timestamp cannot increase beyond int64 maximum");
    }
    return commit(last_stamp_ns_ + 1, Correction::kMinimumIncrement);
  }

  Selection commit(std::int64_t candidate_ns, Correction correction)
  {
    if (correction != Correction::kNone &&
      correction_count_ < std::numeric_limits<std::uint64_t>::max())
    {
      ++correction_count_;
    }

    last_stamp_ns_ = candidate_ns;
    has_last_stamp_ = true;
    return Selection{candidate_ns, correction, correction_count_};
  }

  std::int64_t frame_period_ns_;
  std::int64_t last_stamp_ns_{0};
  std::uint64_t correction_count_{0};
  bool has_last_stamp_{false};
};

}  // namespace auv_imx219_camera

#endif  // CAPTURE_STAMP_SELECTOR_HPP_
