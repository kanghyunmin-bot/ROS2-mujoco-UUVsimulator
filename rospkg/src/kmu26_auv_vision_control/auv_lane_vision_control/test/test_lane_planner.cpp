#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "auv_lane_vision_control/lane_planner.hpp"
#include "auv_lane_vision_control/arena_frame_transform.hpp"
#include "auv_lane_vision_control/depth_p_controller.hpp"
#include "auv_lane_vision_control/dump_cycle.hpp"

namespace
{
using auv_lane_vision_control::ArenaConfig;
using auv_lane_vision_control::DepthPConfig;
using auv_lane_vision_control::LanePlanner;
using auv_lane_vision_control::TopNetStatus;
using auv_lane_vision_control::Vec2;
using auv_lane_vision_control::depth_p_pwm;
using auv_lane_vision_control::should_repeat_dump;

constexpr double kTolerance = 1.0e-9;

LanePlanner competition_a_planner()
{
  return LanePlanner(
    ArenaConfig{
      17.5,
      30.0,
      -1.619,
      13.695,
      0.45,
      3.6375,
      "bottom_left"});
}

TEST(SurfaceMission, BonusCircleFitsSharedCompetitionArena)
{
  const LanePlanner planner(ArenaConfig{17.5, 30.0, -1.619, 13.695, 0.45, 3.6375, "bottom_left"});
  const auto & bounds = planner.safe_bounds();
  const Vec2 center{9.081, -1.305};
  constexpr double radius = 0.65;
  EXPECT_GE(center.x - radius, bounds.x_min);
  EXPECT_LE(center.x + radius, bounds.x_max);
  EXPECT_GE(center.y - radius, bounds.y_min);
  EXPECT_LE(center.y + radius, bounds.y_max);
}

TEST(SurfaceMission, DepthPUsesPositiveDownAndClamps)
{
  const DepthPConfig config{100.0, 180, 1500, true};
  EXPECT_EQ(depth_p_pwm(0.85, 0.30, config), 1445);
  EXPECT_EQ(depth_p_pwm(0.20, 1.00, config), 1580);
  EXPECT_EQ(depth_p_pwm(4.00, 0.00, config), 1320);
}

TEST(SurfaceMission, DumpRepeatsOnlyForFreshOccupiedNet)
{
  EXPECT_TRUE(should_repeat_dump(TopNetStatus::OCCUPIED, 1, 3));
  EXPECT_TRUE(should_repeat_dump(TopNetStatus::OCCUPIED, 2, 3));
  EXPECT_FALSE(should_repeat_dump(TopNetStatus::OCCUPIED, 3, 3));
  EXPECT_FALSE(should_repeat_dump(TopNetStatus::EMPTY, 1, 3));
  EXPECT_FALSE(should_repeat_dump(TopNetStatus::STALE, 1, 3));
}

TEST(LanePlanner, CompetitionAHalfProducesExactlyFourSweeps)
{
  const auto planner = competition_a_planner();

  ASSERT_EQ(planner.lanes().size(), 4U);
  EXPECT_NEAR(planner.actual_lane_spacing_m(), 7.275, kTolerance);
  EXPECT_NEAR(planner.safe_bounds().x_min, -1.169, kTolerance);
  EXPECT_NEAR(planner.safe_bounds().x_max, 15.431, kTolerance);
  EXPECT_NEAR(planner.safe_bounds().y_min, -15.855, kTolerance);
  EXPECT_NEAR(planner.safe_bounds().y_max, 13.245, kTolerance);

  const std::array<double, 4> expected_y{9.6075, 2.3325, -4.9425, -12.2175};
  for (std::size_t index = 0; index < expected_y.size(); ++index) {
    EXPECT_EQ(planner.lanes()[index].index, index);
    EXPECT_NEAR(planner.lanes()[index].endpoint_a.y, expected_y[index], kTolerance);
    EXPECT_NEAR(planner.lanes()[index].endpoint_b.y, expected_y[index], kTolerance);
  }
}

TEST(LanePlanner, FourSweepsCoverAllCompetitionAUnderwaterBuoys)
{
  const auto planner = competition_a_planner();
  constexpr double world_origin_x = -15.881;
  constexpr double world_origin_y = 1.305;
  const std::array<Vec2, 7> world_targets{{
    {-11.139, 8.951},
    {-12.800, -1.300},
    {-11.698, 4.757},
    {-5.430, -7.655},
    {-5.821, 7.668},
    {-4.000, 4.900},
    {-9.379, 3.743},
  }};

  for (const auto & world_target : world_targets) {
    const Vec2 arena_target{
      world_target.x - world_origin_x,
      world_target.y - world_origin_y};
    EXPECT_TRUE(planner.inside_safe_bounds(arena_target));
    double nearest_cross_track = std::numeric_limits<double>::infinity();
    for (const auto & lane : planner.lanes()) {
      nearest_cross_track = std::min(
        nearest_cross_track,
        std::abs(arena_target.y - lane.endpoint_a.y));
    }
    EXPECT_LE(nearest_cross_track, 3.6375);
  }
}
}  // namespace
