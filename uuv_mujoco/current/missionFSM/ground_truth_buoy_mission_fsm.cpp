// C++ replacement boundary for the old ground_truth_buoy_mission_fsm.py.
//
// The mission FSM now lives in ground_truth_buoy_fsm.cpp and is the source
// built by the ROS package.  This file preserves the former module split as
// C++ source while sharing the canonical FSM implementation.

#define MISSION_FSM_CORE_ONLY
#include "ground_truth_buoy_fsm.cpp"

namespace mission_fsm_cpp {

int mission_fsm_compile_anchor() {
  MissionConfig cfg;
  std::vector<Target> targets = {
      {"course_buoy_pinger_white_1_float", {-14.4, 9.0, -8.5}, "pinger", "white", 1},
  };
  MissionController controller(targets, cfg, 0);
  const Step step = controller.update({{-15.4, 9.1, -1.0}, 0.0}, 0.0);
  return step.state.empty() ? 1 : 0;
}

}  // namespace mission_fsm_cpp
