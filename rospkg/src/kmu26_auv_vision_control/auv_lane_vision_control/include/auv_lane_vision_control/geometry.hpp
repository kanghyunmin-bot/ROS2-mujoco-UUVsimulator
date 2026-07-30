#pragma once

#include <cmath>

namespace auv_lane_vision_control
{
struct Vec2
{
  double x{0.0};
  double y{0.0};
};

// 두 좌표의 각 성분을 더한다.
inline Vec2 operator+(const Vec2 & a, const Vec2 & b)
{
  return {a.x + b.x, a.y + b.y};
}

// 두 좌표의 각 성분을 뺀다.
inline Vec2 operator-(const Vec2 & a, const Vec2 & b)
{
  return {a.x - b.x, a.y - b.y};
}

// 좌표의 각 성분에 배율을 곱한다.
inline Vec2 operator*(const Vec2 & value, const double scale)
{
  return {value.x * scale, value.y * scale};
}

// 두 벡터의 내적을 계산한다.
inline double dot(const Vec2 & a, const Vec2 & b)
{
  return a.x * b.x + a.y * b.y;
}

// 벡터의 길이를 계산한다.
inline double norm(const Vec2 & value)
{
  return std::hypot(value.x, value.y);
}

// 두 위치 사이의 직선거리를 계산한다.
inline double distance(const Vec2 & a, const Vec2 & b)
{
  return norm(a - b);
}

// 좌표의 모든 성분이 유효한 수인지 확인한다.
inline bool finite(const Vec2 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y);
}

// 각도를 -π부터 π 범위로 정규화한다.
inline double wrap_pi(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace auv_lane_vision_control
