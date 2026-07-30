#include "auv_lane_vision_control/lane_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace auv_lane_vision_control
{
// 수조 크기와 탐색 폭을 검증하고 레인 중심선을 자동 생성한다.
LanePlanner::LanePlanner(const ArenaConfig & config)
: config_(config)
{
  if (
    !std::isfinite(config_.length_m) || !std::isfinite(config_.width_m) ||
    !std::isfinite(config_.offset_x_m) || !std::isfinite(config_.offset_y_m) ||
    !std::isfinite(config_.safety_margin_m) ||
    !std::isfinite(config_.lane_search_offset_m))
  {
    throw std::invalid_argument("arena and lane dimensions must be finite");
  }
  if (
    config_.length_m <= 0.0 || config_.width_m <= 0.0 ||
    config_.safety_margin_m < 0.0 || config_.lane_search_offset_m <= 0.0)
  {
    throw std::invalid_argument("arena dimensions and lane offset are invalid");
  }
  if (config_.start_corner != "bottom_left" && config_.start_corner != "bottom_right") {
    throw std::invalid_argument("start_corner must be bottom_left or bottom_right");
  }

  safe_bounds_.x_min = config_.offset_x_m + config_.safety_margin_m;
  safe_bounds_.x_max =
    config_.offset_x_m + config_.length_m - config_.safety_margin_m;
  if (config_.start_corner == "bottom_left") {
    safe_bounds_.y_min =
      config_.offset_y_m - config_.width_m + config_.safety_margin_m;
    safe_bounds_.y_max = config_.offset_y_m - config_.safety_margin_m;
  } else {
    safe_bounds_.y_min = config_.offset_y_m + config_.safety_margin_m;
    safe_bounds_.y_max =
      config_.offset_y_m + config_.width_m - config_.safety_margin_m;
  }

  const double usable_length = safe_bounds_.x_max - safe_bounds_.x_min;
  const double usable_width = safe_bounds_.y_max - safe_bounds_.y_min;
  if (usable_length <= 0.0 || usable_width <= 0.0) {
    throw std::invalid_argument("safety margin leaves no usable arena");
  }

  const double nominal_coverage_width = 2.0 * config_.lane_search_offset_m;
  const auto lane_count = static_cast<std::size_t>(
    std::max(1.0, std::ceil(usable_width / nominal_coverage_width)));
  actual_lane_spacing_m_ = usable_width / static_cast<double>(lane_count);
  lanes_.reserve(lane_count);

  for (std::size_t index = 0; index < lane_count; ++index) {
    double y = 0.0;
    if (config_.start_corner == "bottom_left") {
      y = safe_bounds_.y_max -
        (static_cast<double>(index) + 0.5) * actual_lane_spacing_m_;
    } else {
      y = safe_bounds_.y_min +
        (static_cast<double>(index) + 0.5) * actual_lane_spacing_m_;
    }
    lanes_.push_back(
      Lane{
        index,
        {safe_bounds_.x_min, y},
        {safe_bounds_.x_max, y}});
  }
}

// 안전 여유를 반영한 수조 경계를 반환한다.
const ArenaBounds & LanePlanner::safe_bounds() const
{
  return safe_bounds_;
}

// 생성된 모든 레인을 반환한다.
const std::vector<Lane> & LanePlanner::lanes() const
{
  return lanes_;
}

// 자동 계산된 실제 레인 간격을 반환한다.
double LanePlanner::actual_lane_spacing_m() const
{
  return actual_lane_spacing_m_;
}

// 현재 위치가 안전 경계 안에 있는지 확인한다.
bool LanePlanner::inside_safe_bounds(const Vec2 & position) const
{
  return position.x >= safe_bounds_.x_min && position.x <= safe_bounds_.x_max &&
         position.y >= safe_bounds_.y_min && position.y <= safe_bounds_.y_max;
}

// 현재 위치에서 가장 가까운 미완료 레인의 시작 끝점을 선택한다.
std::optional<LaneEndpointChoice> LanePlanner::closest_uncompleted_endpoint(
  const Vec2 & position, const std::vector<bool> & completed) const
{
  if (completed.size() != lanes_.size()) {
    throw std::invalid_argument("completed vector size does not match lane count");
  }

  std::optional<LaneEndpointChoice> best;
  for (const auto & lane : lanes_) {
    if (completed[lane.index]) {
      continue;
    }
    const double distance_a = distance(position, lane.endpoint_a);
    const double distance_b = distance(position, lane.endpoint_b);
    LaneEndpointChoice candidate;
    candidate.lane_index = lane.index;
    candidate.start_at_a = distance_a <= distance_b;
    candidate.start = candidate.start_at_a ? lane.endpoint_a : lane.endpoint_b;
    candidate.finish = candidate.start_at_a ? lane.endpoint_b : lane.endpoint_a;
    candidate.distance_m = std::min(distance_a, distance_b);
    if (!best || candidate.distance_m < best->distance_m) {
      best = candidate;
    }
  }
  return best;
}

// 현재 위치를 지정한 레인 구간 위의 가장 가까운 점으로 투영한다.
Vec2 LanePlanner::project_to_lane(
  const Vec2 & position, const std::size_t lane_index) const
{
  const auto & lane = lane_at(lane_index);
  const Vec2 segment = lane.endpoint_b - lane.endpoint_a;
  const double squared_length = dot(segment, segment);
  const double ratio = std::clamp(
    dot(position - lane.endpoint_a, segment) / squared_length, 0.0, 1.0);
  return lane.endpoint_a + segment * ratio;
}

// 현재 위치와 지정한 레인 사이의 수직 거리를 계산한다.
double LanePlanner::cross_track_distance(
  const Vec2 & position, const std::size_t lane_index) const
{
  return distance(position, project_to_lane(position, lane_index));
}

// 지정한 진행 방향을 기준으로 현재 레인 진행 거리를 계산한다.
double LanePlanner::progress_from_start(
  const Vec2 & position, const std::size_t lane_index, const bool start_at_a) const
{
  const auto & lane = lane_at(lane_index);
  const Vec2 start = start_at_a ? lane.endpoint_a : lane.endpoint_b;
  const Vec2 finish = start_at_a ? lane.endpoint_b : lane.endpoint_a;
  const Vec2 direction = finish - start;
  const double length = norm(direction);
  const Vec2 unit = direction * (1.0 / length);
  return std::clamp(dot(position - start, unit), 0.0, length);
}

// 지정한 레인의 전체 길이를 계산한다.
double LanePlanner::lane_length(const std::size_t lane_index) const
{
  const auto & lane = lane_at(lane_index);
  return distance(lane.endpoint_a, lane.endpoint_b);
}

// 레인 번호가 유효한지 검사하고 해당 레인을 반환한다.
const Lane & LanePlanner::lane_at(const std::size_t lane_index) const
{
  if (lane_index >= lanes_.size()) {
    throw std::out_of_range("lane index out of range");
  }
  return lanes_[lane_index];
}
}  // namespace auv_lane_vision_control
