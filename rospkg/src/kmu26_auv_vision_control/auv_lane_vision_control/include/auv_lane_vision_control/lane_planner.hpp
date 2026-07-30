#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "auv_lane_vision_control/geometry.hpp"

namespace auv_lane_vision_control
{
struct ArenaBounds
{
  double x_min{0.0};
  double x_max{0.0};
  double y_min{0.0};
  double y_max{0.0};
};

struct ArenaConfig
{
  double length_m{15.0};
  double width_m{16.0};
  double offset_x_m{-0.3};
  double offset_y_m{0.3};
  double safety_margin_m{0.5};
  double lane_search_offset_m{2.0};
  std::string start_corner{"bottom_left"};
};

struct Lane
{
  std::size_t index{0};
  Vec2 endpoint_a{};
  Vec2 endpoint_b{};
};

struct LaneEndpointChoice
{
  std::size_t lane_index{0};
  bool start_at_a{true};
  Vec2 start{};
  Vec2 finish{};
  double distance_m{0.0};
};

class LanePlanner
{
public:
  // 수조 설정을 바탕으로 전체 주행 레인을 생성한다.
  explicit LanePlanner(const ArenaConfig & config);

  // 안전 여유를 반영한 수조 경계를 반환한다.
  const ArenaBounds & safe_bounds() const;
  // 생성된 모든 레인을 반환한다.
  const std::vector<Lane> & lanes() const;
  // 자동 계산된 실제 레인 간격을 반환한다.
  double actual_lane_spacing_m() const;

  // 현재 위치가 안전 경계 안에 있는지 확인한다.
  bool inside_safe_bounds(const Vec2 & position) const;
  // 현재 위치에서 가장 가까운 미완료 레인 끝점을 찾는다.
  std::optional<LaneEndpointChoice> closest_uncompleted_endpoint(
    const Vec2 & position, const std::vector<bool> & completed) const;
  // 현재 위치를 지정한 레인의 가장 가까운 점으로 투영한다.
  Vec2 project_to_lane(const Vec2 & position, std::size_t lane_index) const;
  // 현재 위치와 지정한 레인 사이의 수직 거리를 계산한다.
  double cross_track_distance(const Vec2 & position, std::size_t lane_index) const;
  // 지정한 주행 시작점 기준으로 레인 진행 거리를 계산한다.
  double progress_from_start(
    const Vec2 & position, std::size_t lane_index, bool start_at_a) const;
  // 지정한 레인의 전체 길이를 반환한다.
  double lane_length(std::size_t lane_index) const;

private:
  // 레인 번호를 검사한 뒤 해당 레인을 반환한다.
  const Lane & lane_at(std::size_t lane_index) const;

  ArenaConfig config_;
  ArenaBounds safe_bounds_{};
  std::vector<Lane> lanes_;
  double actual_lane_spacing_m_{0.0};
};
}  // namespace auv_lane_vision_control
