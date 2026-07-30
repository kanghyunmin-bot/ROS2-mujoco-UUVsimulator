#pragma once

#include <cmath>
#include <stdexcept>

#include "auv_lane_vision_control/geometry.hpp"

namespace auv_lane_vision_control
{
class ArenaFrameTransform
{
public:
  // 기준 위치와 방향을 받아 수조 좌표 변환을 초기화한다.
  bool initialize(const Vec2 & odom_origin, const double odom_yaw_rad)
  {
    if (initialized_) {
      return false;
    }
    if (!finite(odom_origin) || !std::isfinite(odom_yaw_rad)) {
      throw std::invalid_argument("start-frame origin and yaw must be finite");
    }

    odom_origin_ = odom_origin;
    initial_yaw_rad_ = wrap_pi(odom_yaw_rad);
    cos_yaw_ = std::cos(initial_yaw_rad_);
    sin_yaw_ = std::sin(initial_yaw_rad_);
    initialized_ = true;
    return true;
  }

  // 좌표 변환의 초기화 여부를 반환한다.
  bool initialized() const
  {
    return initialized_;
  }

  // 주행 좌표의 위치를 수조 기준 위치로 변환한다.
  Vec2 position_from_odom(const Vec2 & odom_position) const
  {
    require_initialized();
    const Vec2 relative = odom_position - odom_origin_;
    return {
      cos_yaw_ * relative.x + sin_yaw_ * relative.y,
      -sin_yaw_ * relative.x + cos_yaw_ * relative.y};
  }

  // 주행 좌표의 방향을 수조 기준 방향으로 변환한다.
  double yaw_from_odom(const double odom_yaw_rad) const
  {
    require_initialized();
    return wrap_pi(odom_yaw_rad - initial_yaw_rad_);
  }

  // 회전 표현값에서 평면 방향각을 계산한다.
  static double yaw_from_quaternion(
    const double w, const double x, const double y, const double z)
  {
    return std::atan2(
      2.0 * (w * z + x * y),
      1.0 - 2.0 * (y * y + z * z));
  }

private:
  // 좌표 변환 사용 전에 초기화 상태를 검사한다.
  void require_initialized() const
  {
    if (!initialized_) {
      throw std::logic_error("arena frame transform is not initialized");
    }
  }

  Vec2 odom_origin_{};
  double initial_yaw_rad_{0.0};
  double cos_yaw_{1.0};
  double sin_yaw_{0.0};
  bool initialized_{false};
};
}  // namespace auv_lane_vision_control
