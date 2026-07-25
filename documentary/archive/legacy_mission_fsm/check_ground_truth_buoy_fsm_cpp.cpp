#define MISSION_FSM_CORE_ONLY
#include "ground_truth_buoy_fsm.cpp"

namespace {

void require_true(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

void require_close(double actual, double expected, const char *message, double tol = 1.0e-9) {
  if (std::abs(actual - expected) > tol) throw std::runtime_error(message);
}

Pose pose_for_intake_error(const Target &target, const MissionConfig &cfg, Vec3 p_intake, double yaw = 0.0) {
  const Vec3 body_offset = add(cfg.intake_offset, p_intake);
  return {sub(target.xyz, rotate_body_to_world(body_offset, yaw)), yaw};
}

Pose pose_for_surface_collector_error(Vec3 target_xyz, const MissionConfig &cfg, Vec3 p_collector, double yaw = 0.0) {
  const Vec3 body_offset = add(cfg.surface_collector_offset, p_collector);
  return {sub(target_xyz, rotate_body_to_world(body_offset, yaw)), yaw};
}

LiveState live_state_for_target(const Target &target, Vec3 xyz, double received_s) {
  LiveState live;
  live.name = target.name;
  live.target_xyz = xyz;
  live.body_xyz = xyz;
  live.course = target.course;
  live.color = target.color;
  live.number = target.number;
  live.detached = true;
  live.eq_active = false;
  live.collector_net_enabled = true;
  live.received_s = received_s;
  return live;
}

std::unordered_map<std::string, LiveState> live_surface_states(
    const std::vector<Target> &targets, double received_s) {
  std::unordered_map<std::string, LiveState> states;
  for (const auto &target : targets) {
    states[target.name] = live_state_for_target(target, {target.xyz.x, target.xyz.y, 0.05}, received_s);
  }
  return states;
}

std::unordered_map<std::string, LiveState> live_score_states(
    const std::vector<Target> &targets, Vec3 score_zone, double received_s) {
  std::unordered_map<std::string, LiveState> states;
  for (size_t i = 0; i < targets.size(); ++i) {
    const double y_offset = (static_cast<double>(i % 5) - 2.0) * 0.06;
    states[targets[i].name] = live_state_for_target(
        targets[i], {score_zone.x, score_zone.y + y_offset, 0.05}, received_s);
    states[targets[i].name].net_score_released = true;
    states[targets[i].name].net_score_release_time_s = received_s;
  }
  return states;
}

void check_scene_target_loading(const std::string &scene) {
  Options all;
  all.scene = scene;
  all.course = "all";
  const auto all_targets = load_targets(all);
  require_true(all_targets.size() == 25, "all targets should include A/B red/yellow/orange and pinger buoys");
  require_true(std::any_of(all_targets.begin(), all_targets.end(), [](const Target &target) {
                 return target.name == "course_buoy_pinger_white_1_float";
               }),
               "pinger target should be available in all-course mode");

  Options course_a;
  course_a.scene = scene;
  course_a.course = "a";
  course_a.no_pinger = true;
  const auto a_targets = load_targets(course_a);
  require_true(a_targets.size() == 12, "course A should include 5 red, 5 yellow, and 2 orange buoys");
  std::map<std::string, int> counts;
  for (const auto &target : a_targets) {
    counts[target.color] += 1;
    if (target.color == "red") require_true(std::abs(target.xyz.z) < 0.2, "red targets must be surface buoys");
    else require_true(target.xyz.z < -1.0, "non-red targets must be underwater");
  }
  require_true(counts["red"] == 5, "course A should include five red surface buoys");
  require_true(counts["yellow"] == 5, "course A should include five yellow buoys");
  require_true(counts["orange"] == 2, "course A should include two orange buoys");
  require_true(a_targets.front().name == "course_buoy_a_red_1_float", "targets should be deterministic");
}

void check_live_buoy_status_parse() {
  const auto states = parse_live_status(R"json(
    {
      "source": "mujoco_live",
      "time_s": 12.5,
      "buoys": [
        {
          "id": "course_buoy_a_yellow_1_float",
          "class_name": "yellow",
          "course": "a",
          "number": 1,
          "target_xyz": [-14.4, 9.0, -8.5],
          "body_xyz": [-14.4, 9.0, -8.5],
          "attach_xyz": [-14.4, 9.0, -8.585],
          "detached": false,
          "eq_active": true,
          "collector_net_enabled": true,
          "netted": true,
          "netted_time_s": 12.0,
          "net_score_released": false,
          "target_kind": "float_center"
        }
      ]
    }
  )json", 100.0);
  const auto it = states.find("course_buoy_a_yellow_1_float");
  require_true(it != states.end(), "live status parser should return the named buoy");
  require_close(it->second.target_xyz.z, -8.5, "live target z should parse");
  require_true(it->second.color == "yellow", "live color should parse");
  require_true(it->second.eq_active && *it->second.eq_active, "live equality active flag should parse");
  require_true(it->second.collector_net_enabled, "live collector-net enabled flag should parse");
  require_true(it->second.netted, "live netted flag should parse");
  require_close(*it->second.netted_time_s, 12.0, "live netted time should parse");
}

std::vector<Target> planned_course_a_targets(const std::string &scene);

void check_attached_targets_use_live_attach_point(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  const auto it = std::find_if(targets.begin(), targets.end(), [](const Target &target) {
    return target.color == "yellow";
  });
  require_true(it != targets.end(), "attach-point target test requires an underwater target");

  MissionConfig cfg;
  cfg.live_buoy_timeout = 100.0;
  MissionController controller(targets, cfg, 0);
  LiveState live = live_state_for_target(*it, {it->xyz.x, it->xyz.y, -8.50}, 1.0);
  live.detached = false;
  live.eq_active = true;
  live.attach_xyz = Vec3{it->xyz.x, it->xyz.y, -8.86};
  live.magnet_xyz = Vec3{it->xyz.x, it->xyz.y, -8.90};
  controller.update_live({{it->name, live}});

  const Vec3 attached_xyz = controller.target_xyz_for_test(it->name, 1.0);
  require_close(attached_xyz.z, -8.86,
                "attached underwater targets should use live attach point instead of float center", 1.0e-6);
  controller.force_detached_for_test(it->name, 1.2, true);
  live.detached = true;
  live.target_xyz = {it->xyz.x, it->xyz.y, 0.05};
  controller.update_live({{it->name, live}});
  const Vec3 floating_xyz = controller.target_xyz_for_test(it->name, 1.2);
  require_close(floating_xyz.z, 0.05, "floating targets should return to live float center", 1.0e-6);
}

void check_required_live_status_stops_static_fallback(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  targets.resize(1);
  MissionConfig cfg;
  cfg.require_live_status = true;
  cfg.live_buoy_timeout = 0.4;
  cfg.live_status_timeout = 0.4;
  MissionController controller(targets, cfg, 0);
  const Target &target = targets.front();
  const Pose pose = pose_for_intake_error(target, cfg, {2.0, 0.0, 0.0});

  Step no_live = controller.update(pose, 0.0);
  require_true(no_live.state == MISSION_SIM_STALE, "required live status should stop before static fallback");
  require_close(no_live.command.forward, 0.0, "stale live status should command neutral forward");

  LiveState live;
  live.name = target.name;
  live.target_xyz = target.xyz;
  live.body_xyz = target.xyz;
  live.course = target.course;
  live.color = target.color;
  live.number = target.number;
  live.detached = false;
  live.eq_active = true;
  live.received_s = 0.1;
  controller.update_live({{target.name, live}});
  Step fresh = controller.update(pose, 0.1);
  require_true(fresh.state != MISSION_SIM_STALE, "fresh live status should allow mission logic to run");

  Step stale = controller.update(pose, 0.6);
  require_true(stale.state == MISSION_SIM_STALE, "expired live status should re-enter SIM_STALE");
  const std::string status = controller.status_json(stale, pose, 0.6, false, true, false);
  require_true(status.find("\"live_status\"") != std::string::npos, "status JSON must expose live status");
  require_true(status.find("\"stale\":true") != std::string::npos, "status JSON must expose stale live status");
}

void check_rc_frame() {
  Command cmd;
  cmd.forward = 0.25;
  cmd.sway = -0.25;
  cmd.heave = 0.50;
  cmd.yaw = 0.10;
  cmd.pitch = 0.25;
  const auto channels = rc_channels(cmd, true, true, DEFAULT_RC_SPAN);
  require_true(channels[CH_PITCH] == 1600, "pitch channel should scale from command");
  require_true(channels[CH_HEAVE] == 1300, "positive-down heave should lower RC3 when inverted");
  require_true(channels[CH_YAW] == 1460, "positive yaw should lower RC4 when inverted");
  require_true(channels[CH_FORWARD] == 1600, "forward channel should scale from command");
  require_true(channels[CH_SWAY] == 1400, "sway channel should scale from command");
  for (size_t i = 0; i < PRIMARY_RC_CHANNEL_COUNT; ++i) {
    if (i == CH_PITCH || i == CH_HEAVE || i == CH_YAW || i == CH_FORWARD || i == CH_SWAY) continue;
    require_true(channels[i] == RC_NEUTRAL, "unused primary channels should remain neutral");
  }
}

std::vector<Target> planned_full_targets(const std::string &scene, const std::string &own_course) {
  Options opt;
  opt.scene = scene;
  opt.course = "all";
  opt.own_course = own_course;
  opt.nearest_first = true;
  opt.cfg.own_course = opt.own_course;
  const Vec3 start = load_body_positions(opt.scene)["base_link"];
  return plan_targets(load_targets(opt), opt, start);
}

std::vector<Target> planned_course_a_targets(const std::string &scene) {
  return planned_full_targets(scene, "a");
}

std::vector<Target> planned_course_b_targets(const std::string &scene) {
  return planned_full_targets(scene, "b");
}

void check_planning_pinger_then_home_course(const std::string &scene) {
  const auto targets = planned_course_a_targets(scene);
  require_true(!targets.empty(), "planned target list must not be empty");
  require_true(targets.size() == 13, "all-course A mission must include pinger + 12 A-course buoys");
  require_true(targets.front().course == "pinger", "all-course mission must target pinger first");
  require_true(targets.front().color == "white", "first target must be the white pinger buoy");
  std::map<std::string, int> counts;
  for (const auto &target : targets) {
    require_true(target.course == "pinger" || target.course == "a", "all-course plan must exclude opponent course targets");
    counts[target.color] += 1;
  }
  require_true(counts["white"] == 1, "mission plan must include one pinger buoy");
  require_true(counts["red"] == 5, "mission plan must include five A-course red surface buoys");
  require_true(counts["yellow"] == 5, "mission plan must include five A-course yellow buoys");
  require_true(counts["orange"] == 2, "mission plan must include two A-course orange buoys");
}

void check_planning_pinger_then_selected_b_course(const std::string &scene) {
  const auto targets = planned_course_b_targets(scene);
  require_true(!targets.empty(), "planned B target list must not be empty");
  require_true(targets.size() == 13, "all-course B mission must include pinger + 12 B-course buoys");
  require_true(targets.front().course == "pinger", "all-course B mission must target pinger first");
  std::map<std::string, int> counts;
  for (const auto &target : targets) {
    require_true(target.course == "pinger" || target.course == "b", "B all-course plan must exclude A-course targets");
    if (target.course == "b") require_true(target.xyz.x >= 0.0, "B-course planned targets must stay on B side of split");
    counts[target.color] += 1;
  }
  require_true(counts["white"] == 1, "B mission plan must include one pinger buoy");
  require_true(counts["red"] == 5, "B mission plan must include five B-course red surface buoys");
  require_true(counts["yellow"] == 5, "B mission plan must include five B-course yellow buoys");
  require_true(counts["orange"] == 2, "B mission plan must include two B-course orange buoys");
}

void check_planning_underwater_order_and_surface_tail(const std::string &scene) {
  const auto targets = planned_course_a_targets(scene);
  require_true(targets.size() == 13, "planned A mission should include pinger plus 12 home targets");
  require_true(targets.front().course == "pinger", "planned A mission must start with pinger");
  for (size_t i = 1; i <= 7; ++i) {
    require_true(targets[i].course == "a", "underwater mission segment must stay in course A");
    require_true(targets[i].color == "yellow" || targets[i].color == "orange",
                 "underwater mission segment must include only yellow/orange buoys");
  }
  for (size_t i = 8; i < targets.size(); ++i) {
    require_true(targets[i].course == "a", "surface collection tail must stay in course A");
    require_true(targets[i].color == "red", "surface collection tail must contain red surface buoys only");
  }

  std::vector<Target> remaining(targets.begin() + 1, targets.begin() + 8);
  Vec3 cur = targets.front().xyz;
  for (size_t expected_index = 1; expected_index <= 7; ++expected_index) {
    const auto best = std::min_element(remaining.begin(), remaining.end(), [&](const Target &a, const Target &b) {
      return norm(sub(a.xyz, cur)) < norm(sub(b.xyz, cur));
    });
    require_true(best != remaining.end(), "greedy nearest target should exist");
    require_true(best->name == targets[expected_index].name, "underwater targets must be greedy nearest-first from current target");
    cur = best->xyz;
    remaining.erase(best);
  }
}

void check_full_mission_defaults_allow_complete_course() {
  MissionConfig cfg;
  require_true(cfg.mission_time_limit <= 420.0, "default mission limit should enforce the 7-minute full-mission target");
  require_true(cfg.max_surface_forward >= 0.85, "surface transit should be fast enough for score-zone validation");
  require_true(!cfg.surface_collect_ground_truth, "surface collection must default to physical collector verification");
}

void check_dive_moves_diagonally_toward_pinger(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  targets.resize(1);
  require_true(targets.front().course == "pinger", "diagonal dive test requires pinger as first target");
  MissionConfig cfg;
  MissionController controller(targets, cfg, 0);

  const Pose initial_pose{{-15.4, 9.1, -1.0}, 0.0};
  Step step = controller.update(initial_pose, 0.0);
  require_true(step.state == MISSION_DIVE, "pinger-first startup should remain in dive phase while shallow");
  require_true(step.target_id && *step.target_id == targets.front().name, "dive should immediately track the pinger");
  require_true(step.command.forward >= 0.20 && step.command.forward <= 0.80,
               "off-axis pinger hydrophone dive should move forward while yaw-aligning");
  require_true(std::abs(step.command.sway) <= 0.02, "off-axis pinger dive should not crab sideways");
  require_true(std::abs(step.command.yaw) >= 0.10, "off-axis pinger dive should command bounded yaw toward the pinger");
  require_true(step.command.heave > 0.10, "shallow pinger dive should still command positive-down heave");

  Pose turning_pose = initial_pose;
  turning_pose.yaw = std::copysign(0.10, step.command.yaw);
  Step damped = controller.update(turning_pose, 0.10);
  require_true(damped.command.yaw * step.command.yaw <= 0.0 ||
                   std::abs(damped.command.yaw) < std::abs(step.command.yaw),
               "pinger yaw command should reduce or reverse when braking a turn toward the target");

  const Pose aligned_pose = pose_for_intake_error(targets.front(), cfg, {8.0, 0.05, -6.5});
  MissionController aligned_controller(targets, cfg, 0);
  Step aligned = aligned_controller.update(aligned_pose, 0.0);
  require_true(aligned.command.forward > 0.90, "aligned pinger dive should allow fast forward motion");
}

void check_pinger_uses_yolo_for_final_alignment(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  targets.resize(1);
  require_true(targets.front().course == "pinger", "YOLO final pinger test requires pinger as first target");
  MissionConfig cfg;
  cfg.pinger_yolo_final_range = 0.45;
  MissionController controller(targets, cfg, 0);
  controller.force_state_for_test(MISSION_APPROACH_NEAR, "UNDERWATER", 0.0);
  controller.force_target_for_test(targets.front().name);

  YoloGuidance yolo;
  yolo.valid = true;
  yolo.active = true;
  yolo.model_found = true;
  yolo.has_target = true;
  yolo.count = 1;
  yolo.received_s = 1.0;
  yolo.error_x_norm = 0.36;
  yolo.error_y_norm = 0.0;
  yolo.height_ratio = 0.18;
  yolo.area_ratio = 0.03;
  controller.update_yolo_for_test(yolo);

  const Pose close_pose = pose_for_intake_error(targets.front(), cfg, {0.36, 0.02, 0.01});
  Step step = controller.update(close_pose, 1.0);
  require_true(step.state == MISSION_APPROACH_NEAR, "near pinger should stay in approach while YOLO aligns");
  require_true(step.command.forward <= 0.12, "off-center final pinger YOLO should slow forward motion");
  require_true(step.command.yaw < -0.05, "pixel-right final pinger YOLO should command rightward yaw");

  yolo.received_s = 1.2;
  yolo.error_x_norm = 0.02;
  yolo.height_ratio = 0.34;
  yolo.area_ratio = 0.08;
  yolo.capture = true;
  controller.update_yolo_for_test(yolo);
  Step centered = controller.update(close_pose, 1.2);
  require_true(centered.command.forward >= 0.28, "centered final pinger YOLO should drive into the buoy");
}

void check_pinger_direct_aligns_before_close_forward() {
  MissionConfig cfg;
  MissionController controller({}, cfg, 0);

  Detection far_diagonal;
  far_diagonal.buoy_id = "course_buoy_pinger_white_1_float";
  far_diagonal.p_intake = {8.0, 0.8, -2.0};
  far_diagonal.distance = norm(far_diagonal.p_intake);
  Command far = controller.pinger_hydrophone_direct_for_test(far_diagonal);
  require_true(far.forward >= 0.70, "far pinger direct approach should still move diagonally while aligning");

  Detection close_vertical = far_diagonal;
  close_vertical.p_intake = {0.23, 0.0, 1.93};
  close_vertical.distance = norm(close_vertical.p_intake);
  Command vertical = controller.pinger_hydrophone_direct_for_test(close_vertical);
  require_true(std::abs(vertical.forward) <= 0.02,
               "close pinger with large vertical error must align depth before driving forward");
  require_true(vertical.heave < -0.20, "close pinger below vehicle should command upward heave before contact");

  Detection close_yaw = far_diagonal;
  close_yaw.p_intake = {0.55, 0.55, 0.04};
  close_yaw.distance = norm(close_yaw.p_intake);
  Command yaw = controller.pinger_hydrophone_direct_for_test(close_yaw);
  require_true(std::abs(yaw.forward) <= 0.10, "close off-yaw pinger should yaw in place before forward contact");
  require_true(yaw.yaw >= 0.06, "close off-yaw pinger should command bounded yaw toward the target");
}

void check_status_json_reports_robot_state(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  targets.resize(1);
  MissionConfig cfg;
  MissionController controller(targets, cfg, 0);
  const Pose pose{{-15.4, 9.1, -1.0}, 0.0};
  const Step step = controller.update(pose, 0.0);
  const std::string status = controller.status_json(step, pose, 0.0, false, true, false);
  require_true(status.find("\"robot_state\"") != std::string::npos, "status JSON must include robot_state");
  require_true(status.find("BUOY_DETECTED") != std::string::npos, "pinger-visible dive should report BUOY_DETECTED");
}

void check_commit_timeout_retries_same_pinger(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  targets.resize(1);
  require_true(targets.front().course == "pinger", "retry test requires pinger as first target");

  MissionConfig cfg;
  cfg.commit_timeout = 1.0;
  MissionController controller(targets, cfg, 0);
  const Target &target = targets.front();

  controller.update(pose_for_intake_error(target, cfg, {1.0, 0.0, 0.38}), 0.0);
  controller.update(pose_for_intake_error(target, cfg, {1.0, 0.0, 0.38}), 0.2);
  controller.update(pose_for_intake_error(target, cfg, {1.2, 0.0, 0.10}), 0.4);
  controller.update(pose_for_intake_error(target, cfg, {1.2, 0.0, 0.10}), 0.6);
  controller.update(pose_for_intake_error(target, cfg, {0.65, 0.00, 0.05}), 0.8);
  Step committed = controller.update(pose_for_intake_error(target, cfg, {0.65, 0.00, 0.05}), 1.0);
  require_true(committed.state == MISSION_COMMIT || committed.state == MISSION_CAPTURE_CHECK,
               "close alignment should reach commit/capture path");

  Step retried = controller.update(pose_for_intake_error(target, cfg, {1.10, 0.90, 0.00}), 3.0);
  require_true(retried.failed_count == 0, "commit timeout must not mark pinger failed");
  require_true(retried.target_id && *retried.target_id == target.name, "commit timeout must keep the same pinger target");
  require_true(retried.state == MISSION_APPROACH_FAR || retried.state == MISSION_COMMIT ||
                   retried.state == MISSION_CAPTURE_CHECK || retried.state == MISSION_LIFT_DETACH,
               "commit timeout should retry current target instead of advancing");
}

void check_capture_enters_lift(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  targets.resize(1);
  MissionConfig cfg;
  MissionController controller(targets, cfg, 0);
  const Target &target = targets.front();

  controller.update(pose_for_intake_error(target, cfg, {1.0, 0.0, 0.38}), 0.0);
  controller.update(pose_for_intake_error(target, cfg, {1.0, 0.0, 0.38}), 0.2);
  controller.update(pose_for_intake_error(target, cfg, {1.2, 0.0, 0.10}), 0.4);
  controller.update(pose_for_intake_error(target, cfg, {1.2, 0.0, 0.10}), 0.6);
  controller.update(pose_for_intake_error(target, cfg, {0.65, 0.08, 0.04}), 0.8);
  controller.update(pose_for_intake_error(target, cfg, {0.65, 0.08, 0.04}), 1.0);
  Step step = controller.update(pose_for_intake_error(target, cfg, {0.18, 0.06, 0.04}), 1.2);
  require_true(step.state == MISSION_CAPTURE_CHECK || step.state == MISSION_LIFT_DETACH ||
                   step.state == MISSION_DETACH_CONFIRM,
                   "capture volume should enter capture/lift path");
}

void check_near_capture_volume_fast_paths_to_capture(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  targets.resize(1);
  MissionConfig cfg;
  MissionController controller(targets, cfg, 0);
  const Target &target = targets.front();

  controller.force_target_for_test(target.name);
  controller.force_state_for_test(MISSION_APPROACH_NEAR, "UNDERWATER", 0.0);
  Step step = controller.update(pose_for_intake_error(target, cfg, {-0.02, 0.00, 0.11}), 0.2);
  require_true(step.state == MISSION_CAPTURE_CHECK, "capture-box target should immediately enter capture check");
  require_true(step.capture_flag, "capture-box target should expose capture flag in status");
}

void check_capture_does_not_fake_detach_when_live_stays_attached(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  targets.resize(1);
  MissionConfig cfg;
  cfg.lift_timeout = 0.5;
  cfg.live_buoy_timeout = 100.0;
  MissionController controller(targets, cfg, 0);
  const Target &target = targets.front();

  LiveState live;
  live.name = target.name;
  live.target_xyz = target.xyz;
  live.body_xyz = target.xyz;
  live.course = target.course;
  live.color = target.color;
  live.number = target.number;
  live.detached = false;
  live.eq_active = true;
  live.received_s = 0.0;
  controller.update_live({{target.name, live}});

  controller.update(pose_for_intake_error(target, cfg, {1.0, 0.0, 0.38}), 0.0);
  controller.update(pose_for_intake_error(target, cfg, {1.0, 0.0, 0.38}), 0.2);
  controller.update(pose_for_intake_error(target, cfg, {1.2, 0.0, 0.10}), 0.4);
  controller.update(pose_for_intake_error(target, cfg, {1.2, 0.0, 0.10}), 0.6);
  controller.update(pose_for_intake_error(target, cfg, {0.65, 0.08, 0.04}), 0.8);
  controller.update(pose_for_intake_error(target, cfg, {0.65, 0.08, 0.04}), 1.0);
  controller.update(pose_for_intake_error(target, cfg, {0.18, 0.06, 0.04}), 1.2);
  Step lift = controller.update(pose_for_intake_error(target, cfg, {0.18, 0.06, 0.04}), 1.4);
  require_true(lift.state == MISSION_CAPTURE_CHECK || lift.state == MISSION_COMMIT,
               "capture should hold contact-push before physical detach");
  Step held = controller.update(pose_for_intake_error(target, cfg, {0.20, 0.08, -0.06}), 2.2);
  require_true(held.state == MISSION_CAPTURE_CHECK || held.state == MISSION_COMMIT ||
                   held.state == MISSION_APPROACH_FAR,
               "capture path must keep trying/retry instead of faking detach when live eq stays active");
  require_true(held.target_state == BUOY_TARGETED || held.target_state == BUOY_ATTACHED,
               "target must not become rising without physical detach");
  require_true(held.processed_count == 0, "physical-attached buoy must not be counted processed");
}

void check_yaw_first_underwater_control() {
  MissionConfig cfg;
  MissionController controller({}, cfg, 0);

  Detection side_target;
  side_target.p_intake = {2.20, 2.20, 0.0};
  side_target.p_rate = {};
  Command side_cmd = controller.approach_far_for_test(side_target);
  require_true(side_cmd.yaw >= cfg.max_yaw_far - 1.0e-6, "side target should command bounded yaw toward the buoy");
  require_true(side_cmd.forward >= 0.35 && side_cmd.forward <= 0.60,
               "front-side target should keep forward drive while yaw-aligning");
  require_true(std::abs(side_cmd.sway) <= 0.02, "side target should not crab sideways during yaw alignment");

  Detection rear_target;
  rear_target.p_intake = {-1.20, 0.35, 0.0};
  rear_target.p_rate = {};
  Command rear_cmd = controller.approach_far_for_test(rear_target);
  require_true(std::abs(rear_cmd.forward) <= 0.02, "rear target should force rotate-in-place instead of reversing/advancing");
  require_true(std::abs(rear_cmd.sway) <= 0.02, "rear target should not crab while turning around");
  require_true(std::abs(rear_cmd.yaw) >= cfg.max_yaw_far - 1.0e-6,
               "rear target should command the bounded maximum yaw turn");

  Detection aligned_target;
  aligned_target.p_intake = {2.80, 0.04, 0.0};
  aligned_target.p_rate = {};
  Command aligned_cmd = controller.approach_far_for_test(aligned_target);
  require_true(aligned_cmd.forward >= 0.95, "aligned underwater target should allow near-full forward approach");
  require_true(std::abs(aligned_cmd.sway) <= 0.05, "aligned underwater target should only use small lateral trim");

  Detection deep_far_target;
  deep_far_target.p_intake = {8.0, 0.0, -6.0};
  deep_far_target.p_rate = {};
  Command deep_far_cmd = controller.approach_far_for_test(deep_far_target);
  require_true(deep_far_cmd.heave >= 0.90, "far underwater target should allow fast heave while depth error is large");

  Detection close_aligned_target;
  close_aligned_target.p_intake = {0.20, 0.02, 0.12};
  close_aligned_target.p_rate = {};
  Command close_cmd = controller.approach_far_for_test(close_aligned_target);
  require_true(close_cmd.forward >= 0.25, "close aligned target should not stall after retrying from commit");

  Detection overshot_target;
  overshot_target.p_intake = {-0.10, 0.06, 0.23};
  overshot_target.p_rate = {};
  Command overshot_unstick = controller.approach_unstick_for_test(overshot_target, true);
  require_true(overshot_unstick.forward < 0.0, "overshot close target should reverse for re-entry instead of driving past it");
  require_true(std::abs(overshot_unstick.yaw) < 0.25, "overshot close target should not spin in place with pi yaw error");

  Detection near_side_target;
  near_side_target.p_intake = {0.75, 0.90, 0.0};
  near_side_target.p_rate = {};
  Command near_side_cmd = controller.approach_near_for_test(near_side_target);
  require_true(near_side_cmd.forward >= 0.10 && near_side_cmd.forward <= 0.36,
               "near side target should keep controlled forward drive while yaw-aligning");
  require_true(std::abs(near_side_cmd.sway) <= 0.02, "near side target should not crab into capture");
  require_true(near_side_cmd.yaw >= cfg.max_yaw_near - 1.0e-6,
               "near side target should use bounded yaw toward centerline");
}

void check_yaw_first_surface_control() {
  MissionConfig cfg;
  MissionController controller({}, cfg, 0);
  const Pose pose{{0.0, 0.0, cfg.surface_ready_z}, 0.0};

  Command side_cmd = controller.surface_waypoint_for_test(pose, {0.0, 5.0, cfg.surface_ready_z});
  require_true(side_cmd.yaw >= cfg.max_surface_yaw - 1.0e-6,
               "surface side waypoint should use bounded rotation toward the target first");
  require_true(std::abs(side_cmd.forward) <= 0.02, "surface side waypoint should not drive forward before yaw alignment");
  require_true(std::abs(side_cmd.sway) <= 0.02, "surface side waypoint should not crab sideways before yaw alignment");

  Command aligned_cmd = controller.surface_waypoint_for_test(pose, {5.0, 0.05, cfg.surface_ready_z});
  require_true(aligned_cmd.forward >= 0.85, "aligned surface waypoint should run fast forward");
  require_true(std::abs(aligned_cmd.sway) <= 0.08, "aligned surface waypoint should only use trim sway");

  const Command deep_ascent = controller.surface_depth_command_for_test({{0.0, 0.0, -8.0}, 0.0});
  const Command mid_ascent = controller.surface_depth_command_for_test({{0.0, 0.0, -1.5}, 0.0});
  const Command near_ascent = controller.surface_depth_command_for_test({{0.0, 0.0, -0.55}, 0.0});
  require_true(deep_ascent.heave < -0.60, "deep ascent should retain useful upward speed");
  require_true(std::abs(mid_ascent.heave) <= cfg.max_surface_ascent_mid_heave + 1.0e-9,
               "surface ascent should brake inside two metres");
  require_true(std::abs(near_ascent.heave) <= cfg.max_surface_ascent_near_heave + 1.0e-9,
               "surface ascent should use a low final heave command");
}

void check_boundary_guard_keeps_course_a() {
  MissionConfig cfg;
  cfg.own_course = "a";
  MissionController controller({}, cfg, 0);
  Command toward_opponent;
  toward_opponent.forward = 1.0;
  toward_opponent.sway = 0.0;
  toward_opponent.phase = MISSION_APPROACH_FAR;
  const Pose near_boundary{{-0.65, 0.0, cfg.search_depth_z}, 0.0};
  const Command guarded = controller.boundary_guard_for_test(near_boundary, toward_opponent);
  require_true(guarded.forward <= 0.05, "course A boundary guard should stop forward motion toward opponent side");

  const Pose safe_home_side{{-3.0, 0.0, cfg.search_depth_z}, 0.0};
  const Command safe = controller.boundary_guard_for_test(safe_home_side, toward_opponent);
  require_true(safe.forward > 0.95, "course A boundary guard should not limit commands well inside home side");
}

void check_underwater_done_naturally_enters_ascend(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  MissionConfig cfg;
  MissionController controller(targets, cfg, 0);
  for (const auto &target : targets) {
    if (target.color != "red") controller.force_detached_for_test(target.name, 0.0);
  }
  controller.force_state_for_test(MISSION_NEXT_TARGET, "UNDERWATER", 0.0);
  Step step = controller.update({{targets.front().xyz.x, targets.front().xyz.y, cfg.search_depth_z}, 0.0}, 0.2);
  require_true(step.state == MISSION_ASCEND || step.state == MISSION_SURFACE_READY,
               "after all underwater targets detach, FSM should enter ascent path naturally");
  require_true(step.remaining_attached == 0, "no underwater attached targets should remain before ascent");
}

void check_time_limit_does_not_skip_remaining_underwater(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  MissionConfig cfg;
  MissionController controller(targets, cfg, 0);
  std::string held_target;
  for (const auto &target : targets) {
    if (target.color == "red") continue;
    if (held_target.empty()) {
      held_target = target.name;
      continue;
    }
    controller.force_detached_for_test(target.name, 0.0);
  }
  require_true(!held_target.empty(), "timeout test needs one attached underwater target");
  controller.force_state_for_test(MISSION_NEXT_TARGET, "UNDERWATER", 0.0);
  Step step = controller.update({{targets.front().xyz.x, targets.front().xyz.y, cfg.search_depth_z}, 0.0},
                                cfg.mission_time_limit + 5.0);
  require_true(step.state == MISSION_SEARCH, "time limit must not force ascent while underwater targets remain");
  require_true(step.remaining_attached == 1, "one underwater target should remain after timeout guard test");
}

void check_surface_collects_all_then_scores(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  require_true(targets.size() == 13, "surface score test requires full A-course target set");
  MissionConfig cfg;
  cfg.rise_to_float = 0.1;
  cfg.score_confirm = 0.2;
  cfg.live_buoy_timeout = 100.0;
  MissionController controller(targets, cfg, 0);
  require_true(controller.surface_candidate_count_for_test(0.0) == 5, "red buoys should start as surface collection candidates");
  for (const auto &target : targets) {
    if (target.color != "red") controller.force_detached_for_test(target.name, 0.0);
  }
  auto live_surface = live_surface_states(targets, 1.0);
  controller.update_live(live_surface);
  controller.force_state_for_test(MISSION_SURFACE_READY, "SURFACE", 0.0);

  Step ready = controller.update({{0.0, 0.0, cfg.surface_ready_z}, 0.0}, 1.0);
  require_true(ready.state == MISSION_SURFACE_COLLECT, "surface-ready should enter surface collection");

  Step far = controller.update({{0.0, 0.0, cfg.surface_ready_z}, 0.0}, 1.1);
  require_true(far.state == MISSION_SURFACE_COLLECT, "surface collection should keep running before physical capture");
  require_true(controller.collected_count_for_test() == 0, "surface collection must not fake-collect all candidates");
  const Command lateral_surface =
      controller.surface_waypoint_for_test({{0.0, 0.0, cfg.surface_ready_z}, 0.0}, {0.6, 4.0, cfg.surface_ready_z});
  require_true(std::abs(lateral_surface.forward) < 1.0e-9,
               "surface waypoint must rotate in place before advancing toward an off-axis buoy");

  double now = 1.2;
  for (size_t i = 0; i < targets.size(); ++i) {
    const Vec3 live_xyz{targets[i].xyz.x, targets[i].xyz.y, 0.05};
    live_surface[targets[i].name] = live_state_for_target(targets[i], live_xyz, now);
    live_surface[targets[i].name].netted = true;
    live_surface[targets[i].name].netted_time_s = now;
    controller.update_live(live_surface);
    Step collected = controller.update(pose_for_surface_collector_error(live_xyz, cfg, {0.0, 0.0, 0.0}), now);
    require_true(
        controller.collected_count_for_test() == i + 1,
        "surface collector should mark exactly one runtime-netted buoy per target");
    require_true(controller.all_required_collected_for_test() == (i + 1 == targets.size()),
                 "all-required collection gate must stay false until the final planned buoy is netted");
    if (i + 1 < targets.size()) {
      require_true(collected.state == MISSION_SURFACE_COLLECT, "surface collection should continue until all candidates are collected");
      require_true(collected.command.forward <= cfg.max_surface_loaded_forward + 1.0e-9,
                   "loaded surface collector must limit forward acceleration");
      require_true(std::abs(collected.command.sway) <= cfg.max_surface_loaded_sway + 1.0e-9,
                   "loaded surface collector must limit sway acceleration");
      require_true(std::abs(collected.command.yaw) <= cfg.max_surface_loaded_yaw + 1.0e-9,
                   "loaded surface collector must limit yaw acceleration");
      require_true(std::abs(collected.command.heave) <= cfg.max_surface_loaded_heave + 1.0e-9,
                   "loaded surface collector must limit vertical acceleration");
    }
    now += 0.2;
  }
  require_true(controller.surface_candidate_count_for_test(now) == 0, "surface candidates should be empty after physical collection");

  const Pose score_pose = pose_for_surface_collector_error(cfg.score_zone_a, cfg, {0.0, 0.0, 0.0});
  Step confirm = controller.update(score_pose, now + 0.2);
  require_true(confirm.state == MISSION_SCORE_CONFIRM, "collector score-zone entry should start confirmation");
  Step not_done = controller.update(score_pose, now + 0.5);
  require_true(not_done.state == MISSION_SCORE_CONFIRM, "score confirmation must wait for live buoy positions in score zone");
  require_true(std::abs(not_done.command.forward) < 1.0e-9 && std::abs(not_done.command.yaw) < 1.0e-9,
               "score confirmation must hold XY/yaw while the physical net releases buoys");
  require_true(controller.scored_count_for_test() == 0, "robot-only score-zone entry must not score collected buoys");

  auto released_outside = live_score_states(targets, cfg.score_zone_a, now + 0.6);
  released_outside[targets.front().name].target_xyz.x = cfg.score_zone_a.x + 3.0;
  released_outside[targets.front().name].body_xyz = released_outside[targets.front().name].target_xyz;
  controller.update_live(released_outside);
  Step outside = controller.update(score_pose, now + 0.6);
  require_true(outside.state == MISSION_SCORE_CONFIRM,
               "release flag with a buoy physically outside the score zone must not complete");
  Step outside_held = controller.update(score_pose, now + 1.0);
  require_true(outside_held.state == MISSION_SCORE_CONFIRM,
               "score hold must require every released buoy coordinate inside the score zone");
  require_true(controller.scored_count_for_test() == 0,
               "released flag alone must not score a physically outside buoy");

  controller.update_live(live_score_states(targets, cfg.score_zone_a, now + 1.1));
  Step holding = controller.update(score_pose, now + 1.1);
  require_true(holding.state == MISSION_SCORE_CONFIRM, "live buoys in score zone should start score hold");
  Step complete = controller.update(score_pose, now + 1.5);
  require_true(complete.state == MISSION_COMPLETE, "score confirmation should complete the mission");
  require_true(controller.scored_count_for_test() == targets.size(), "all collected pinger/red/yellow/orange buoys should be scored");
  require_true(controller.all_required_scored_for_test(), "mission completion must prove every planned buoy scored");
}

void check_yolo_surface_centering_collects(const std::string &scene) {
  auto targets = planned_course_a_targets(scene);
  MissionConfig cfg;
  cfg.surface_collect_yolo = true;
  cfg.yolo_surface_capture_hold = 0.25;
  MissionController controller(targets, cfg, 0);
  controller.force_state_for_test(MISSION_SURFACE_COLLECT, "SURFACE", 0.0);

  YoloGuidance right;
  right.valid = true;
  right.active = true;
  right.model_found = true;
  right.has_target = true;
  right.count = 1;
  right.received_s = 1.0;
  right.error_x_norm = 0.40;
  right.error_y_norm = 0.0;
  right.height_ratio = 0.10;
  right.area_ratio = 0.02;
  controller.update_yolo_for_test(right);
  Step turn = controller.update({{0.0, 0.0, cfg.surface_ready_z}, 0.0}, 1.0);
  require_true(turn.state == MISSION_SURFACE_COLLECT, "YOLO surface control should stay in surface collect");
  require_true(turn.command.yaw < -0.05, "pixel-right YOLO target should command rightward yaw");
  require_true(turn.command.forward < cfg.max_surface_forward, "off-center YOLO target should slow forward drive");

  YoloGuidance centered = right;
  centered.received_s = 1.2;
  centered.error_x_norm = 0.02;
  centered.height_ratio = 0.34;
  centered.area_ratio = 0.08;
  centered.capture = true;
  controller.update_yolo_for_test(centered);
  controller.update({{0.0, 0.0, cfg.surface_ready_z}, 0.0}, 1.2);
  require_true(controller.collected_count_for_test() == 0, "YOLO capture should require a short hold before marking collected");
  centered.received_s = 1.6;
  controller.update_yolo_for_test(centered);
  controller.update({{0.0, 0.0, cfg.surface_ready_z}, 0.0}, 1.6);
  require_true(controller.collected_count_for_test() == 0, "YOLO capture must not fake collection without runtime netted state");
  auto surface = live_surface_states(targets, 1.7);
  for (const auto &target : targets) {
    if (target.color != "red") continue;
    surface[target.name].netted = true;
    surface[target.name].netted_time_s = 1.7;
    break;
  }
  controller.update_live(surface);
  controller.update({{0.0, 0.0, cfg.surface_ready_z}, 0.0}, 1.7);
  require_true(controller.collected_count_for_test() == 1, "runtime netted state should mark one surface buoy collected");
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const std::string scene =
        argc > 1 ? argv[1] : "sim/current/scenes/tank_current_scene.xml";
    check_scene_target_loading(scene);
    check_live_buoy_status_parse();
    check_attached_targets_use_live_attach_point(scene);
    check_required_live_status_stops_static_fallback(scene);
    check_rc_frame();
    check_planning_pinger_then_home_course(scene);
    check_planning_pinger_then_selected_b_course(scene);
    check_planning_underwater_order_and_surface_tail(scene);
    check_full_mission_defaults_allow_complete_course();
    check_dive_moves_diagonally_toward_pinger(scene);
    check_pinger_uses_yolo_for_final_alignment(scene);
    check_pinger_direct_aligns_before_close_forward();
    check_status_json_reports_robot_state(scene);
    check_commit_timeout_retries_same_pinger(scene);
    check_capture_enters_lift(scene);
    check_near_capture_volume_fast_paths_to_capture(scene);
    check_capture_does_not_fake_detach_when_live_stays_attached(scene);
    check_yaw_first_underwater_control();
    check_yaw_first_surface_control();
    check_boundary_guard_keeps_course_a();
    check_underwater_done_naturally_enters_ascend(scene);
    check_time_limit_does_not_skip_remaining_underwater(scene);
    check_surface_collects_all_then_scores(scene);
    check_yolo_surface_centering_collects(scene);
    std::cout << "ground_truth_buoy_fsm_cpp=PASS\n";
    return 0;
  } catch (const std::exception &exc) {
    std::cerr << "ground_truth_buoy_fsm_cpp=FAIL: " << exc.what() << "\n";
    return 1;
  }
}
