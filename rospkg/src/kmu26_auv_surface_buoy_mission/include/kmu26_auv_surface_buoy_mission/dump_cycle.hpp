#pragma once

#include <cstdint>

namespace kmu26_auv_surface_buoy_mission
{
enum class TopNetStatus {OCCUPIED, EMPTY, STALE};

inline void reset_dump_attempts_for_lane(uint32_t & attempts)
{
  attempts = 0;
}

inline uint32_t next_dump_attempt(uint32_t & attempts)
{
  return ++attempts;
}

inline TopNetStatus reconcile_dump_status(
  const TopNetStatus vision_status, const bool physical_collector_occupied)
{
  // 시뮬레이터/실물 Collector가 아직 부표를 잡고 있으면 카메라의 빈 화면보다
  // 물리 상태를 우선한다. 카메라가 망 내부의 부표를 가리지 못하는 경우가 있다.
  return physical_collector_occupied ? TopNetStatus::OCCUPIED : vision_status;
}

inline bool should_repeat_dump(
  const TopNetStatus status, const uint32_t attempts, const uint32_t max_attempts)
{
  return status == TopNetStatus::OCCUPIED && attempts < max_attempts;
}
}  // namespace kmu26_auv_surface_buoy_mission
