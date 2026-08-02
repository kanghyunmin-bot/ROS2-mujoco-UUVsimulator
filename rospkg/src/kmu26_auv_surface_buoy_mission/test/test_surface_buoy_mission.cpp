#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "kmu26_auv_surface_buoy_mission/lane_planner.hpp"
#include "kmu26_auv_surface_buoy_mission/dump_cycle.hpp"
#include "kmu26_auv_surface_buoy_mission/dump_motion.hpp"

namespace
{
using kmu26_auv_surface_buoy_mission::ArenaConfig;
using kmu26_auv_surface_buoy_mission::LanePlanner;
using kmu26_auv_surface_buoy_mission::LaneOrientation;
using kmu26_auv_surface_buoy_mission::Vec2;
using kmu26_auv_surface_buoy_mission::TopNetStatus;
using kmu26_auv_surface_buoy_mission::dump_forward_target_reached;
using kmu26_auv_surface_buoy_mission::dump_reverse_target_reached;
using kmu26_auv_surface_buoy_mission::next_dump_attempt;
using kmu26_auv_surface_buoy_mission::reset_dump_attempts_for_lane;
using kmu26_auv_surface_buoy_mission::should_repeat_dump;

constexpr double kTolerance = 1.0e-9;

TEST(SurfaceMission, BonusCircleFitsSharedCompetitionArena)
{
  const LanePlanner planner(ArenaConfig{17.5, 30.0, -1.619, 13.695, 0.45, 3.6375, "bottom_left"});
  const auto & bounds = planner.safe_bounds();
  const Vec2 center{
    0.5 * (bounds.x_min + bounds.x_max) + 1.95,
    0.5 * (bounds.y_min + bounds.y_max)};
  constexpr double radius = 0.65 + 0.35;
  EXPECT_NEAR(center.x, 9.081, kTolerance);
  EXPECT_NEAR(center.y, -1.305, kTolerance);
  EXPECT_GE(center.x - radius, bounds.x_min);
  EXPECT_LE(center.x + radius, bounds.x_max);
  EXPECT_GE(center.y - radius, bounds.y_min);
  EXPECT_LE(center.y + radius, bounds.y_max);
}

TEST(SurfaceMission, DumpRepeatsOnlyForFreshOccupiedNet)
{
  EXPECT_TRUE(should_repeat_dump(TopNetStatus::OCCUPIED, 1, 3));
  EXPECT_TRUE(should_repeat_dump(TopNetStatus::OCCUPIED, 2, 3));
  EXPECT_FALSE(should_repeat_dump(TopNetStatus::OCCUPIED, 3, 3));
  EXPECT_FALSE(should_repeat_dump(TopNetStatus::EMPTY, 1, 3));
  EXPECT_FALSE(should_repeat_dump(TopNetStatus::STALE, 1, 3));
}

ArenaConfig vertical_surface_config()
{
  ArenaConfig config{10.0, 10.0, 0.0, 0.0, 0.0, 5.0, "bottom_left"};
  config.lane_orientation = LaneOrientation::VERTICAL;
  return config;
}

TEST(SurfaceLanePlanner, VerticalLaneWithoutBonusIntersectionIsOneSegment)
{
  const LanePlanner planner(vertical_surface_config());
  ASSERT_EQ(planner.lanes().size(), 1U);
  EXPECT_NEAR(planner.lanes()[0].endpoint_a.x, 5.0, kTolerance);
  EXPECT_NEAR(planner.lanes()[0].endpoint_b.x, 5.0, kTolerance);
  EXPECT_NEAR(planner.lanes()[0].endpoint_a.y, -10.0, kTolerance);
  EXPECT_NEAR(planner.lanes()[0].endpoint_b.y, 0.0, kTolerance);
}

TEST(SurfaceLanePlanner, BonusIntersectionCreatesIndependentLowerAndUpperSegments)
{
  auto config = vertical_surface_config();
  config.circular_exclusion_enabled = true;
  config.circular_exclusion_center = {5.0, -5.0};
  config.circular_exclusion_radius_m = 2.0;
  config.min_lane_segment_length_m = 1.0;
  const LanePlanner planner(config);

  ASSERT_EQ(planner.lanes().size(), 2U);
  EXPECT_NEAR(planner.lanes()[0].endpoint_a.y, -10.0, kTolerance);
  EXPECT_NEAR(planner.lanes()[0].endpoint_b.y, -7.0, kTolerance);
  EXPECT_NEAR(planner.lanes()[1].endpoint_a.y, -3.0, kTolerance);
  EXPECT_NEAR(planner.lanes()[1].endpoint_b.y, 0.0, kTolerance);
  for (const auto & lane : planner.lanes()) {
    for (int sample = 0; sample <= 20; ++sample) {
      const double ratio = static_cast<double>(sample) / 20.0;
      const Vec2 point = lane.endpoint_a + (lane.endpoint_b - lane.endpoint_a) * ratio;
      EXPECT_GE(distance(point, config.circular_exclusion_center), 2.0 - kTolerance);
    }
  }
}

TEST(SurfaceLanePlanner, TooShortSplitSegmentIsOmitted)
{
  auto config = vertical_surface_config();
  config.circular_exclusion_enabled = true;
  config.circular_exclusion_center = {5.0, -8.0};
  config.circular_exclusion_radius_m = 1.5;
  config.min_lane_segment_length_m = 1.0;
  const LanePlanner planner(config);

  ASSERT_EQ(planner.lanes().size(), 1U);
  EXPECT_NEAR(planner.lanes()[0].endpoint_a.y, -6.5, kTolerance);
  EXPECT_NEAR(planner.lanes()[0].endpoint_b.y, 0.0, kTolerance);
}

TEST(SurfaceMission, DumpMotionUsesForwardOvershootThenReverseDistance)
{
  const Vec2 center{0.0, 0.0};
  EXPECT_FALSE(dump_forward_target_reached({0.69, 0.0}, center, 0.0, 0.70));
  EXPECT_TRUE(dump_forward_target_reached({0.70, 0.0}, center, 0.0, 0.70));
  const Vec2 peak{0.80, 0.0};
  EXPECT_FALSE(dump_reverse_target_reached({0.20, 0.0}, peak, center, 0.0, 0.70));
  EXPECT_TRUE(dump_reverse_target_reached({0.10, 0.0}, peak, center, 0.0, 0.70));
  EXPECT_TRUE(dump_reverse_target_reached({-0.01, 0.0}, peak, center, 0.0, 5.0));
}

TEST(SurfaceMission, EveryLaneStartsDumpAtAttemptOne)
{
  uint32_t attempts = 3;
  reset_dump_attempts_for_lane(attempts);
  EXPECT_EQ(next_dump_attempt(attempts), 1U);
  EXPECT_EQ(next_dump_attempt(attempts), 2U);

  reset_dump_attempts_for_lane(attempts);
  EXPECT_EQ(next_dump_attempt(attempts), 1U);
}
}  // namespace
