// C++ replacement boundary for the old ground_truth_buoy_controller_core.py.
//
// The canonical implementation is ground_truth_buoy_fsm.cpp.  This translation
// unit keeps the former "controller core" module as C++ source without
// duplicating mission logic.

#define MISSION_FSM_CORE_ONLY
#include "ground_truth_buoy_fsm.cpp"

namespace mission_fsm_cpp {

int controller_core_compile_anchor(const char *scene_path) {
  Options opt;
  if (scene_path != nullptr && scene_path[0] != '\0') opt.scene = scene_path;
  Command neutral;
  const auto channels = rc_channels(neutral, true, true, DEFAULT_RC_SPAN);
  return channels[CH_FORWARD] == RC_NEUTRAL ? 0 : 1;
}

}  // namespace mission_fsm_cpp
