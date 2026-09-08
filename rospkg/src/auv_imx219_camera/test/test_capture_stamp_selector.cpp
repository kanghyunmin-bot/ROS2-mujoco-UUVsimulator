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

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

#include "capture_stamp_selector.hpp"

namespace
{

void expect_equal(std::int64_t actual, std::int64_t expected, const std::string & context)
{
  if (actual != expected) {
    throw std::runtime_error(
            context + ": expected " + std::to_string(expected) + ", got " +
            std::to_string(actual));
  }
}

void expect_correction(
  auv_imx219_camera::CaptureStampSelector::Correction actual,
  auv_imx219_camera::CaptureStampSelector::Correction expected,
  const std::string & context)
{
  if (actual != expected) {
    throw std::runtime_error(context + ": unexpected correction policy");
  }
}

void test_valid_invalid_valid_sequence()
{
  constexpr std::int64_t frame_period_ns = 33'333'333;
  auv_imx219_camera::CaptureStampSelector selector(frame_period_ns);

  const auto first_valid = selector.select_valid(920'000'000, 1'000'000'000);
  const auto fallback = selector.select_fallback(1'033'000'000);
  const auto next_valid = selector.select_valid(986'000'000, 1'066'000'000);

  expect_equal(first_valid.stamp_ns, 920'000'000, "first valid PTS");
  expect_equal(fallback.stamp_ns, 953'333'333, "fallback advances by one frame period");
  expect_equal(next_valid.stamp_ns, 986'000'000, "valid PTS after fallback remains monotonic");
}

void test_fallback_is_capped_by_ros_now()
{
  auv_imx219_camera::CaptureStampSelector selector(30);
  expect_equal(selector.select_valid(100, 100).stamp_ns, 100, "initial valid stamp");
  expect_equal(selector.select_fallback(115).stamp_ns, 115, "fallback ROS-now cap");
}

void test_regressing_valid_uses_frame_period_not_one_nanosecond()
{
  auv_imx219_camera::CaptureStampSelector selector(30);
  expect_equal(selector.select_valid(100, 100).stamp_ns, 100, "initial valid stamp");

  const auto duplicate = selector.select_valid(100, 140);
  expect_equal(duplicate.stamp_ns, 130, "duplicate valid stamp frame-period correction");
  expect_correction(
    duplicate.correction,
    auv_imx219_camera::CaptureStampSelector::Correction::kFramePeriodCapped,
    "duplicate valid stamp correction");
  expect_equal(duplicate.correction_count, 1, "first correction count");

  const auto regressing = selector.select_valid(90, 145);
  expect_equal(regressing.stamp_ns, 145, "regressing valid stamp ROS-now cap");
  expect_correction(
    regressing.correction,
    auv_imx219_camera::CaptureStampSelector::Correction::kFramePeriodCapped,
    "regressing valid stamp correction");
  expect_equal(regressing.correction_count, 2, "second correction count");
}

void test_minimum_increment_is_reserved_for_nonadvancing_ros_time()
{
  auv_imx219_camera::CaptureStampSelector selector(30);
  expect_equal(selector.select_valid(100, 100).stamp_ns, 100, "initial valid stamp");

  const auto corrected = selector.select_valid(90, 90);
  expect_equal(corrected.stamp_ns, 101, "backward ROS time minimum correction");
  expect_correction(
    corrected.correction,
    auv_imx219_camera::CaptureStampSelector::Correction::kMinimumIncrement,
    "backward ROS time correction");
}

void test_first_fallback_uses_ros_now()
{
  auv_imx219_camera::CaptureStampSelector selector(30);
  const auto fallback = selector.select_fallback(250);
  const auto valid = selector.select_valid(200, 280);

  expect_equal(fallback.stamp_ns, 250, "first fallback stamp");
  expect_equal(valid.stamp_ns, 280, "valid PTS after first fallback");
  expect_correction(
    valid.correction,
    auv_imx219_camera::CaptureStampSelector::Correction::kFramePeriodCapped,
    "valid PTS after first fallback correction");
}

}  // namespace

int main()
{
  try {
    test_valid_invalid_valid_sequence();
    test_fallback_is_capped_by_ros_now();
    test_regressing_valid_uses_frame_period_not_one_nanosecond();
    test_minimum_increment_is_reserved_for_nonadvancing_ros_time();
    test_first_fallback_uses_ros_now();
  } catch (const std::exception & error) {
    std::cerr << "CaptureStampSelector regression test failed: " << error.what() << '\n';
    return 1;
  }

  std::cout << "CaptureStampSelector regression tests passed\n";
  return 0;
}
