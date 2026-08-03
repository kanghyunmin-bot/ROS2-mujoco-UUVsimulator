#pragma once

namespace kmu26_auv_surface_buoy_mission
{

enum class SurfaceMissionState
{
  IDLE,
  ASCEND_TO_COLLECTION_DEPTH,
  INITIAL_TOP_CHECK,
  SURFACE_SEARCH,
  SURFACE_ALIGN,
  SURFACE_CAPTURE,
  MOVE_TO_BONUS,
  DESCEND_TO_DUMP_DEPTH,
  MOVE_TO_BONUS_CENTER,
  DUMP_EJECT,
  DUMP_CHECK,
  RETURN_TO_BONUS_CENTER,
  RETURN_TO_WORK_DEPTH,
  FAILSAFE
};

struct VisionGate
{
  bool front_enabled{false};
  bool top_enabled{false};
};

constexpr VisionGate vision_gate_for_state(
  SurfaceMissionState state, bool surface_heading_to_start = false)
{
  switch (state) {
    case SurfaceMissionState::SURFACE_SEARCH:
      // 레인 시작점으로 이동할 때는 다른 레인의 부표를 추론하지 않는다.
      return {!surface_heading_to_start, false};
    case SurfaceMissionState::SURFACE_ALIGN:
      return {true, false};
    case SurfaceMissionState::SURFACE_CAPTURE:
      // 정면에서 상단 수집망으로 넘어가는 영상 전이를 확인한다.
      return {true, true};
    case SurfaceMissionState::INITIAL_TOP_CHECK:
    case SurfaceMissionState::DUMP_EJECT:
    case SurfaceMissionState::DUMP_CHECK:
      return {false, true};
    default:
      return {false, false};
  }
}

}  // namespace kmu26_auv_surface_buoy_mission
