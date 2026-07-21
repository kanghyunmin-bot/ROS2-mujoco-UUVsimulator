#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifndef MISSION_FSM_CORE_ONLY
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#endif

namespace {

constexpr int RC_NEUTRAL = 1500;
constexpr double DEFAULT_RC_SPAN = 400.0;
constexpr uint16_t RC_NOCHANGE = 65535;
constexpr uint16_t RC_RELEASE = 0;
constexpr size_t RC_CHANNEL_COUNT = 18;
constexpr size_t PRIMARY_RC_CHANNEL_COUNT = 8;
constexpr size_t CH_PITCH = 0;
constexpr size_t CH_HEAVE = 2;
constexpr size_t CH_YAW = 3;
constexpr size_t CH_FORWARD = 4;
constexpr size_t CH_SWAY = 5;

const char *BUOY_ATTACHED = "ATTACHED";
const char *BUOY_TARGETED = "TARGETED";
const char *BUOY_CAPTURED = "CAPTURED";
const char *BUOY_RISING = "RISING";
const char *BUOY_FLOATING = "FLOATING";
const char *BUOY_COLLECTED = "COLLECTED";
const char *BUOY_SCORED = "SCORED";
const char *BUOY_FAILED = "FAILED";

const char *MISSION_INIT = "INIT";
const char *MISSION_DIVE = "DIVE_TO_SEARCH_DEPTH";
const char *MISSION_SEARCH = "UNDERWATER_SEARCH";
const char *MISSION_SELECT = "SELECT_TARGET";
const char *MISSION_APPROACH_FAR = "APPROACH_FAR";
const char *MISSION_APPROACH_NEAR = "APPROACH_NEAR";
const char *MISSION_COMMIT = "COMMIT";
const char *MISSION_CAPTURE_CHECK = "CAPTURE_CHECK";
const char *MISSION_LIFT_DETACH = "LIFT_DETACH";
const char *MISSION_DETACH_CONFIRM = "DETACH_CONFIRM";
const char *MISSION_NEXT_TARGET = "NEXT_TARGET_DECISION";
const char *MISSION_ASCEND = "ASCEND_TO_SURFACE";
const char *MISSION_SURFACE_READY = "SURFACE_READY";
const char *MISSION_SURFACE_COLLECT = "SURFACE_COLLECT";
const char *MISSION_SCORE_TRANSIT = "SCORE_ZONE_TRANSIT";
const char *MISSION_SCORE_CONFIRM = "SCORE_ZONE_CONFIRM";
const char *MISSION_SIM_STALE = "SIM_STALE";
const char *MISSION_COMPLETE = "MISSION_COMPLETE";
const char *WAIT_ARM_PHASE = "WAIT_ARM";
const char *MISSION_FSM_BUILD_TAG = "cpp_fast_direct_mission_20260708g";

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Target {
  std::string name;
  Vec3 xyz;
  std::string course;
  std::string color;
  int number{0};
};

struct LiveState {
  std::string name;
  Vec3 target_xyz;
  std::optional<Vec3> body_xyz;
  std::optional<Vec3> attach_xyz;
  std::optional<Vec3> magnet_xyz;
  std::string course;
  std::string color;
  int number{0};
  bool detached{false};
  std::optional<bool> eq_active;
  std::string target_kind{"float_center"};
  std::optional<double> probe_release_margin_m;
  bool probe_release_proximity{false};
  bool collector_net_enabled{false};
  bool netted{false};
  std::optional<double> netted_time_s;
  bool net_score_released{false};
  std::optional<double> net_score_release_time_s;
  double received_s{0.0};
  std::string source{"mujoco_live"};
};

struct Pose {
  Vec3 xyz;
  double yaw{0.0};
};

struct Command {
  double forward{0.0};
  double sway{0.0};
  double heave{0.0};
  double yaw{0.0};
  double pitch{0.0};
  std::string phase{"idle"};
  double distance{0.0};
  Vec3 error;
  bool done{false};
};

struct Detection {
  std::string buoy_id;
  std::string class_name;
  Vec3 p_intake;
  Vec3 p_rate;
  double distance{0.0};
  double confidence{0.0};
  double relative_speed{0.0};
  Vec3 target_xyz;
  bool has_target_xyz{false};
  std::string coordinate_source{"scene_static"};
};

struct YoloGuidance {
  bool valid{false};
  bool active{false};
  bool model_found{false};
  bool has_target{false};
  bool capture{false};
  int count{0};
  double received_s{0.0};
  double center_x_norm{NAN};
  double center_y_norm{NAN};
  double error_x_norm{NAN};
  double error_y_norm{NAN};
  double height_ratio{0.0};
  double area_ratio{0.0};
  double forward{0.0};
  double sway{0.0};
  double heave{0.0};
  double yaw{0.0};
  double confidence{0.0};
  std::string state;
  std::string label;
  std::string side;
};

struct MissionConfig {
  Vec3 intake_offset{0.40, -0.248, -0.082};
  std::string own_course{"a"};
  double course_boundary_x{0.0};
  double course_boundary_margin{0.80};
  double course_boundary_standoff{0.70};
  double search_depth_z{-8.8};
  double surface_ready_z{-0.3};
  double depth_tolerance{0.2};
  double dive_min_depth{6.0};
  double dive_search_vertical_window{0.45};
  double dive_search_distance{1.5};
  double live_buoy_timeout{3.0};
  double live_status_timeout{3.0};
  bool require_live_status{false};
  double surface_threshold_z{-0.5};
  double mission_time_limit{420.0};
  double fake_vision_range{30.0};
  double target_lost_timeout{1.0};
  double approach_far_x{1.0};
  double approach_near_entry_y{0.35};
  double approach_near_entry_z{0.35};
  double commit_x{0.35};
  double alignment_yz_tolerance{0.14};
  double alignment_hold{0.12};
  double commit_forward{1.00};
  double commit_timeout{6.0};
  double capture_x{0.22};
  double capture_y{0.14};
  double capture_z{0.12};
  double capture_speed_max{1.35};
  double capture_hold{0.05};
  double lift_heave_up{0.82};
  double lift_forward_hold{0.82};
  double lift_detach_dz{0.25};
  double lift_timeout{3.0};
  double rise_to_float{3.0};
  Vec3 surface_collector_offset{-0.011, 0.0, 0.360};
  double surface_collect_range{0.55};
  double surface_collect_x_window{0.34};
  double surface_collect_y_window{0.24};
  double surface_collect_z_window{0.30};
  double surface_collect_min_x_window{0.85};
  double surface_collect_min_y_window{0.75};
  double surface_collect_min_z_window{0.65};
  double surface_collect_timeout{90.0};
  bool surface_collect_ground_truth{false};
  bool surface_collect_yolo{true};
  double yolo_surface_timeout{1.0};
  double yolo_surface_center_tolerance{0.11};
  double yolo_surface_close_height_ratio{0.30};
  double yolo_surface_close_area_ratio{0.075};
  double yolo_surface_capture_hold{0.30};
  double yolo_surface_yaw_gain{0.78};
  double yolo_surface_sway_gain{0.22};
  double yolo_surface_search_yaw{0.24};
  bool pinger_hydrophone_enable{true};
  double pinger_yolo_final_range{0.20};
  double pinger_yolo_near_range{0.35};
  double pinger_hydrophone_forward_fast{1.00};
  double pinger_hydrophone_forward_mid{0.90};
  double pinger_hydrophone_forward_slow{0.44};
  double pinger_hydrophone_yaw_gain{0.38};
  double pinger_yolo_forward{0.40};
  double pinger_yolo_yaw_gain{0.84};
  double pinger_yolo_sway_gain{0.16};
  Vec3 score_zone_a{-6.8, 0.0, -0.3};
  Vec3 score_zone_b{6.8, 0.0, -0.3};
  double score_zone_radius{0.35};
  double score_zone_z_window{0.9};
  double score_buoy_tolerance{0.35};
  double score_confirm{0.55};
  double score_dump_forward{0.35};
  double score_dump_pitch{0.55};
  double k_depth{0.70};
  double max_depth_heave{0.95};
  double min_depth_command{0.25};
  double max_surface_ascent_far_heave{0.65};
  double max_surface_ascent_mid_heave{0.30};
  double max_surface_ascent_near_heave{0.12};
  double max_surface_collect_heave{0.18};
  double max_surface_loaded_heave{0.08};
  double min_align_heave{0.18};
  double kx_far{1.00};
  double ky_far{0.45};
  double kz_far{0.42};
  double kyaw_far{0.42};
  double kx_near{1.12};
  double ky_near{0.50};
  double kz_near{0.48};
  double kyaw_near{0.48};
  double max_vx_far{1.00};
  double max_vy_far{0.55};
  double max_vz_far{0.38};
  double max_yaw_far{0.12};
  double far_drive_min{0.62};
  double far_drive_fast_min{0.86};
  double approach_no_progress_timeout{8.0};
  double approach_progress_epsilon{0.18};
  double max_vx_near{0.92};
  double max_vx_reverse_near{0.35};
  double min_vx_near{0.62};
  double max_vy_near{0.40};
  double max_vz_near{0.34};
  double max_yaw_near{0.08};
  double k_surface_forward{0.85};
  double k_surface_sway{0.68};
  double k_surface_heave{0.55};
  double k_surface_yaw{0.48};
  double max_surface_forward{1.00};
  double max_surface_loaded_forward{0.35};
  double max_surface_reverse{0.22};
  double max_surface_sway{0.55};
  double max_surface_loaded_sway{0.10};
  double max_surface_heave{0.50};
  double max_surface_yaw{0.14};
  double max_surface_loaded_yaw{0.10};
  double search_yaw{0.08};
  double yaw_rate_damping{0.60};
};

struct MissionBuoy {
  Target target;
  std::string state{BUOY_ATTACHED};
  bool processed{false};
  bool failed{false};
  std::optional<double> detach_time;
  double release_force_threshold{12.0};
};

struct Step {
  Command command;
  std::string state;
  std::string mode;
  std::optional<std::string> target_id;
  std::string target_class;
  std::string target_state;
  std::optional<Detection> detection;
  bool capture_flag{false};
  int remaining_attached{0};
  int processed_count{0};
  int failed_count{0};
};

struct Options {
  std::string scene{"uuv_mujoco/current/scenes/tank_current_scene.xml"};
  std::string course{"all"};
  std::string own_course{"a"};
  std::string colors{"red,yellow,orange,white"};
  bool no_pinger{false};
  bool nearest_first{false};
  int max_targets{0};
  double rate_hz{10.0};
  bool dry_run{false};
  bool wait_armed{true};
  std::string pose_topic{"/mujoco/ground_truth/pose"};
  std::string buoy_status_topic{"/mujoco/course_buoys/status"};
  std::string yolo_detection_topic{"/uuv_mujoco/yolo_buoy_detections"};
  std::string state_topic{"/mavros/state"};
  std::string rc_topic{"/mavros/rc/override"};
  std::string manual_topic{"/mavros/manual_control/send"};
  std::string command_override_topic{"/uuv_mujoco/sitl/command_override"};
  std::string transport{"rc_override"};
  std::string mission_log{"auto"};
  std::string status_json;
  double rc_pwm_span{DEFAULT_RC_SPAN};
  bool no_invert_heave_rc{false};
  bool no_invert_yaw_rc{false};
  MissionConfig cfg;
};

double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 scale(Vec3 v, double s) { return {v.x * s, v.y * s, v.z * s}; }
double norm(Vec3 v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

Vec3 rotate_body_to_world(Vec3 v, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}

Vec3 rotate_world_to_body(Vec3 v, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v.x + s * v.y, -s * v.x + c * v.y, v.z};
}

double yaw_from_quat(double x, double y, double z, double w) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

std::string trim(const std::string &s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::vector<std::string> split_csv(const std::string &text) {
  std::vector<std::string> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = lower(trim(item));
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

std::string json_escape(const std::string &s) {
  std::ostringstream out;
  for (char c : s) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << c; break;
    }
  }
  return out.str();
}

std::string q(const std::string &s) { return "\"" + json_escape(s) + "\""; }

std::string vec_json(Vec3 v) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "[" << v.x << "," << v.y << "," << v.z << "]";
  return out.str();
}

std::string number_json(double v) {
  if (!std::isfinite(v)) return "null";
  std::ostringstream out;
  out << std::fixed << std::setprecision(6) << v;
  return out.str();
}

std::optional<std::string> attr(const std::string &line, const std::string &name) {
  std::regex re(name + "=\"([^\"]*)\"");
  std::smatch m;
  if (std::regex_search(line, m, re)) return m[1].str();
  return std::nullopt;
}

Vec3 parse_vec3_attr(const std::optional<std::string> &text) {
  if (!text) return {};
  std::stringstream ss(*text);
  Vec3 v;
  ss >> v.x >> v.y >> v.z;
  return v;
}

struct ParsedName {
  std::string course;
  std::string color;
  int number{0};
};

std::optional<ParsedName> parse_buoy_name(const std::string &name) {
  static const std::regex re(R"(^course_buoy_(?:(a|b)_)?(pinger_)?([a-z]+)_([0-9]+)_float$)");
  std::smatch m;
  if (!std::regex_match(name, m, re)) return std::nullopt;
  ParsedName p;
  p.course = m[2].matched ? "pinger" : m[1].str();
  p.color = m[3].str();
  p.number = std::stoi(m[4].str());
  if (p.course.empty()) return std::nullopt;
  return p;
}

std::map<std::string, Vec3> load_body_positions(const std::string &scene) {
  std::ifstream in(scene);
  if (!in) throw std::runtime_error("cannot open scene: " + scene);
  std::map<std::string, Vec3> positions;
  std::vector<Vec3> stack;
  stack.push_back({});
  std::string line;
  while (std::getline(in, line)) {
    if (line.find("</body>") != std::string::npos && stack.size() > 1) {
      stack.pop_back();
    }
    const auto body_pos = line.find("<body ");
    if (body_pos == std::string::npos) continue;
    const Vec3 local = parse_vec3_attr(attr(line, "pos"));
    const Vec3 world = add(stack.back(), local);
    if (auto name = attr(line, "name")) positions[*name] = world;
    stack.push_back(world);
  }
  return positions;
}

int course_order(const std::string &course) {
  if (course == "a") return 0;
  if (course == "b") return 1;
  if (course == "pinger") return 2;
  return 99;
}

int color_order(const std::string &color) {
  if (color == "red") return 0;
  if (color == "yellow") return 1;
  if (color == "orange") return 2;
  if (color == "white") return 3;
  return 99;
}

bool is_surface_buoy(const Target &target) {
  return target.color == "red";
}

bool target_on_own_side(const Target &target, const Options &opt) {
  if (target.course == "pinger") return !opt.no_pinger;
  if (target.course != opt.own_course) return false;
  if (opt.own_course == "a") return target.xyz.x <= opt.cfg.course_boundary_x + 1.0e-6;
  if (opt.own_course == "b") return target.xyz.x >= opt.cfg.course_boundary_x - 1.0e-6;
  return false;
}

std::vector<Target> load_targets(const Options &opt) {
  const auto positions = load_body_positions(opt.scene);
  const auto colors = split_csv(opt.colors);
  const std::set<std::string> color_set(colors.begin(), colors.end());
  std::vector<Target> targets;
  for (const auto &[name, xyz] : positions) {
    auto parsed = parse_buoy_name(name);
    if (!parsed) continue;
    if (!color_set.empty() && !color_set.count(parsed->color)) continue;
    if (parsed->course == "pinger" && opt.no_pinger) continue;
    if (opt.course != "all" && parsed->course != opt.course) continue;
    targets.push_back({name, xyz, parsed->course, parsed->color, parsed->number});
  }
  std::sort(targets.begin(), targets.end(), [](const Target &a, const Target &b) {
    return std::make_tuple(course_order(a.course), color_order(a.color), a.number, a.name) <
           std::make_tuple(course_order(b.course), color_order(b.color), b.number, b.name);
  });
  return targets;
}

std::vector<Target> nearest_first(std::vector<Target> targets, Vec3 start) {
  std::vector<Target> ordered;
  Vec3 cur = start;
  while (!targets.empty()) {
    auto best = std::min_element(targets.begin(), targets.end(), [&](const Target &a, const Target &b) {
      return norm(sub(a.xyz, cur)) < norm(sub(b.xyz, cur));
    });
    ordered.push_back(*best);
    cur = best->xyz;
    targets.erase(best);
  }
  return ordered;
}

std::vector<Target> plan_targets(const std::vector<Target> &targets, const Options &opt, Vec3 start) {
  std::vector<Target> zone_targets;
  zone_targets.reserve(targets.size());
  for (const auto &t : targets) {
    if (t.course == "pinger" || target_on_own_side(t, opt)) zone_targets.push_back(t);
  }
  if (opt.course != "all") return opt.nearest_first ? nearest_first(zone_targets, start) : zone_targets;
  std::vector<Target> pingers;
  std::vector<Target> home_underwater;
  std::vector<Target> home_surface;
  for (const auto &t : zone_targets) {
    if (t.course == "pinger") pingers.push_back(t);
    if (t.course == opt.own_course && is_surface_buoy(t)) home_surface.push_back(t);
    if (t.course == opt.own_course && !is_surface_buoy(t)) home_underwater.push_back(t);
  }
  if (opt.nearest_first) home_underwater = nearest_first(home_underwater, pingers.empty() ? start : pingers.front().xyz);
  std::vector<Target> out;
  if (!pingers.empty()) out.push_back(pingers.front());
  out.insert(out.end(), home_underwater.begin(), home_underwater.end());
  out.insert(out.end(), home_surface.begin(), home_surface.end());
  return out;
}

std::optional<std::string> json_string(const std::string &obj, const std::string &key) {
  std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch m;
  if (std::regex_search(obj, m, re)) return m[1].str();
  return std::nullopt;
}

std::optional<double> json_number(const std::string &obj, const std::string &key) {
  std::regex re("\"" + key + R"("\s*:\s*([-+0-9.eE]+))");
  std::smatch m;
  if (!std::regex_search(obj, m, re)) return std::nullopt;
  try {
    return std::stod(m[1].str());
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<bool> json_bool(const std::string &obj, const std::string &key) {
  std::regex re("\"" + key + R"("\s*:\s*(true|false|0|1))");
  std::smatch m;
  if (!std::regex_search(obj, m, re)) return std::nullopt;
  const auto v = m[1].str();
  return v == "true" || v == "1";
}

std::optional<Vec3> json_vec3(const std::string &obj, const std::string &key) {
  std::regex re("\"" + key + R"("\s*:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+))");
  std::smatch m;
  if (!std::regex_search(obj, m, re)) return std::nullopt;
  try {
    return Vec3{std::stod(m[1].str()), std::stod(m[2].str()), std::stod(m[3].str())};
  } catch (...) {
    return std::nullopt;
  }
}

std::vector<double> json_number_array(const std::string &obj, const std::string &key) {
  std::regex re("\"" + key + R"("\s*:\s*\[([^\]]*)\])");
  std::smatch m;
  if (!std::regex_search(obj, m, re)) return {};
  std::vector<double> values;
  std::stringstream ss(m[1].str());
  std::string item;
  while (std::getline(ss, item, ',')) {
    try {
      values.push_back(std::stod(trim(item)));
    } catch (...) {
    }
  }
  return values;
}

YoloGuidance parse_yolo_guidance(const std::string &text, double received_s) {
  YoloGuidance y;
  if (trim(text).empty()) return y;
  y.valid = true;
  y.received_s = received_s;
  y.active = json_bool(text, "active").value_or(false);
  y.model_found = json_bool(text, "model_found").value_or(false);
  y.count = static_cast<int>(json_number(text, "count").value_or(0.0));
  y.side = json_string(text, "side").value_or("");
  y.state = json_string(text, "state").value_or("");
  y.label = json_string(text, "label").value_or("buoy");
  y.confidence = json_number(text, "confidence").value_or(0.0);
  y.height_ratio = json_number(text, "height_ratio").value_or(0.0);
  y.area_ratio = json_number(text, "area_ratio").value_or(0.0);
  const auto center = json_number_array(text, "center_norm");
  if (center.size() >= 2) {
    y.center_x_norm = center[0];
    y.center_y_norm = center[1];
  }
  const auto error = json_number_array(text, "error_norm");
  if (error.size() >= 2) {
    y.error_x_norm = error[0];
    y.error_y_norm = error[1];
  } else if (std::isfinite(y.center_x_norm) && std::isfinite(y.center_y_norm)) {
    y.error_x_norm = 2.0 * (y.center_x_norm - 0.50);
    y.error_y_norm = 2.0 * (y.center_y_norm - 0.56);
  }
  y.forward = json_number(text, "forward").value_or(0.0);
  y.sway = json_number(text, "sway").value_or(0.0);
  y.heave = json_number(text, "heave").value_or(0.0);
  y.yaw = json_number(text, "yaw").value_or(0.0);
  y.capture = json_bool(text, "capture").value_or(false);
  y.has_target = y.count > 0 && std::isfinite(y.error_x_norm);
  return y;
}

std::vector<std::string> buoy_objects_from_json(const std::string &text) {
  const auto key = text.find("\"buoys\"");
  if (key == std::string::npos) return {};
  const auto start = text.find('[', key);
  if (start == std::string::npos) return {};
  std::vector<std::string> objects;
  bool in_string = false;
  int depth = 0;
  size_t obj_start = std::string::npos;
  for (size_t i = start + 1; i < text.size(); ++i) {
    const char c = text[i];
    if (c == '"' && (i == 0 || text[i - 1] != '\\')) in_string = !in_string;
    if (in_string) continue;
    if (c == '{') {
      if (depth == 0) obj_start = i;
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0 && obj_start != std::string::npos) {
        objects.push_back(text.substr(obj_start, i - obj_start + 1));
        obj_start = std::string::npos;
      }
    } else if (c == ']' && depth == 0) {
      break;
    }
  }
  return objects;
}

std::unordered_map<std::string, LiveState> parse_live_status(const std::string &text, double received_s) {
  std::unordered_map<std::string, LiveState> states;
  const auto source = json_string(text, "source").value_or("mujoco_live");
  for (const auto &obj : buoy_objects_from_json(text)) {
    std::string name = json_string(obj, "id")
                           .value_or(json_string(obj, "name").value_or(json_string(obj, "body_name").value_or("")));
    if (!name.empty() && name.rfind("course_buoy_", 0) == 0 && name.find("_float") == std::string::npos) {
      name += "_float";
    }
    auto parsed = parse_buoy_name(name);
    if (!parsed) continue;
    auto target_xyz = json_vec3(obj, "target_xyz");
    if (!target_xyz) target_xyz = json_vec3(obj, "xyz");
    if (!target_xyz) target_xyz = json_vec3(obj, "body_xyz");
    if (!target_xyz) target_xyz = json_vec3(obj, "attach_xyz");
    if (!target_xyz) continue;
    LiveState live;
    live.name = name;
    live.target_xyz = *target_xyz;
    live.body_xyz = json_vec3(obj, "body_xyz");
    live.attach_xyz = json_vec3(obj, "attach_xyz");
    live.magnet_xyz = json_vec3(obj, "magnet_xyz");
    live.course = json_string(obj, "course").value_or(parsed->course);
    live.color = json_string(obj, "class_name").value_or(json_string(obj, "color").value_or(parsed->color));
    live.number = static_cast<int>(json_number(obj, "number").value_or(parsed->number));
    live.detached = json_bool(obj, "detached").value_or(false);
    live.eq_active = json_bool(obj, "eq_active");
    live.target_kind = json_string(obj, "target_kind").value_or("float_center");
    live.probe_release_margin_m = json_number(obj, "probe_release_margin_m");
    live.probe_release_proximity = json_bool(obj, "probe_release_proximity").value_or(false);
    live.collector_net_enabled = json_bool(obj, "collector_net_enabled").value_or(false);
    live.netted = json_bool(obj, "netted").value_or(false);
    live.netted_time_s = json_number(obj, "netted_time_s");
    live.net_score_released = json_bool(obj, "net_score_released").value_or(false);
    live.net_score_release_time_s = json_number(obj, "net_score_release_time_s");
    live.received_s = received_s;
    live.source = source;
    states[name] = live;
  }
  return states;
}

class MissionController {
 public:
  MissionController(std::vector<Target> targets, MissionConfig cfg, int max_targets)
      : cfg_(std::move(cfg)) {
    if (max_targets > 0 && static_cast<size_t>(max_targets) < targets.size()) {
      targets.resize(static_cast<size_t>(max_targets));
    }
    int index = 0;
    for (const auto &t : targets) {
      MissionBuoy b;
      b.target = t;
      b.release_force_threshold = 10.0 + std::fmod(static_cast<double>(index) * 1.37, 5.0);
      if (is_surface_buoy(t)) {
        b.state = BUOY_FLOATING;
        b.detach_time = 0.0;
      }
      index_by_name_[t.name] = buoys_.size();
      buoys_.push_back(b);
      ++index;
    }
  }

  void update_live(const std::unordered_map<std::string, LiveState> &states) {
    for (const auto &[name, state] : states) {
      if (index_by_name_.count(name)) {
        live_[name] = state;
        if (!latest_live_received_s_ || state.received_s > *latest_live_received_s_) {
          latest_live_received_s_ = state.received_s;
        }
      }
    }
  }

  void update_yolo(const YoloGuidance &guidance) {
    if (!guidance.valid) return;
    yolo_ = guidance;
  }

  Step update(const Pose &pose, double now) {
    if (!mission_start_) {
      mission_start_ = now;
      state_entered_ = now;
    }
    const double dt = last_update_ ? std::max(0.0, now - *last_update_) : 0.0;
    last_update_ = now;
    update_yaw_rate(pose, now);
    if (live_status_stale(now)) {
      transition(MISSION_SIM_STALE, now);
      return step(zero(MISSION_SIM_STALE), std::nullopt, false);
    }
    if (state_ == MISSION_SIM_STALE) {
      transition(target_id_ ? MISSION_APPROACH_FAR : MISSION_SEARCH, now);
    }
    update_rising(now);
    sync_live(now);
    auto detections = fake_vision(pose, now);
    apply_detection_rates(detections, now);
    Command command = zero(MISSION_INIT);
    bool capture = false;

    if (target_id_ && target_state_is({BUOY_RISING, BUOY_FLOATING, BUOY_COLLECTED, BUOY_SCORED})) {
      target_id_.reset();
      if (!state_is_terminal_path()) transition(MISSION_NEXT_TARGET, now);
    }

    if (state_ == MISSION_INIT) {
      if (remaining_attached() <= 0 && !surface_candidates(now).empty()) {
        transition(MISSION_ASCEND, now);
        command = surface_depth_command(pose, MISSION_ASCEND);
      } else {
        transition(MISSION_DIVE, now);
        if (auto pinger = pinger_detection(detections)) {
          track_pinger_during_dive(*pinger);
          command = pinger_dive_approach(pose, *pinger, now);
        } else {
          command = depth_command(pose, cfg_.search_depth_z, MISSION_DIVE);
        }
      }
    } else if (state_ == MISSION_DIVE) {
      if (remaining_attached() <= 0 && !surface_candidates(now).empty()) {
        transition(MISSION_ASCEND, now);
        command = surface_depth_command(pose, MISSION_ASCEND);
      } else if (auto pinger = pinger_detection(detections)) {
        track_pinger_during_dive(*pinger);
        command = pinger_dive_approach(pose, *pinger, now);
      } else {
        command = depth_command(pose, cfg_.search_depth_z, MISSION_DIVE);
      }
      if (state_ == MISSION_DIVE && (depth_reached(pose, cfg_.search_depth_z) || ready_to_search_from_dive(pose, detections))) {
        transition(MISSION_SEARCH, now);
      }
    } else if (state_ == MISSION_SEARCH) {
      if (remaining_attached() <= 0 && !surface_candidates(now).empty()) transition(MISSION_ASCEND, now);
      else if (!attached_detections(detections).empty()) transition(MISSION_SELECT, now);
      else command = make(0, 0, 0, cfg_.search_yaw, MISSION_SEARCH, {});
    } else if (state_ == MISSION_SELECT) {
      auto d = select_detection(detections);
      if (!d) transition(MISSION_SEARCH, now);
      else {
        target_id_ = d->buoy_id;
        buoy(*target_id_).state = BUOY_TARGETED;
        target_lost_since_.reset();
        alignment_timer_ = 0.0;
        capture_timer_ = 0.0;
        transition(MISSION_APPROACH_FAR, now);
      }
    } else if (state_ == MISSION_APPROACH_FAR) {
      auto d = target_detection(detections);
      if (!d) handle_target_lost(now);
      else {
        target_lost_since_.reset();
        command = detection_is_pinger(*d) ? pinger_target_approach(pose, *d, now, MISSION_APPROACH_FAR) : approach_far(*d);
        if (approach_stalled(*d, MISSION_APPROACH_FAR, now)) command = approach_unstick(*d, false);
        if ((detection_is_pinger(*d) && d->distance <= cfg_.pinger_yolo_near_range) || ready_for_near(*d)) {
          transition(MISSION_APPROACH_NEAR, now);
        }
      }
	    } else if (state_ == MISSION_APPROACH_NEAR) {
	      auto d = target_detection(detections);
	      if (!d) handle_target_lost(now);
	      else {
	        target_lost_since_.reset();
	        command = detection_is_pinger(*d) ? pinger_target_approach(pose, *d, now, MISSION_APPROACH_NEAR) : approach_near(*d);
	        if (approach_stalled(*d, MISSION_APPROACH_NEAR, now)) command = approach_unstick(*d, true);
	        const bool pinger_yolo_ready = detection_is_pinger(*d) && d->distance <= cfg_.pinger_yolo_final_range &&
	                                       fresh_yolo(now).has_value() && yolo_surface_capture_ready(*fresh_yolo(now));
	        if (inside_capture(*d)) {
	          capture = true;
	          commit_start_ = now;
	          transition(MISSION_CAPTURE_CHECK, now);
	        } else {
	          alignment_timer_ = (aligned_for_commit(*d) || pinger_yolo_ready) ? alignment_timer_ + dt : 0.0;
	        }
	        if (state_ == MISSION_APPROACH_NEAR && alignment_timer_ >= cfg_.alignment_hold) {
	          commit_start_ = now;
	          transition(MISSION_COMMIT, now);
	        }
	      }
    } else if (state_ == MISSION_COMMIT) {
      auto d = target_detection(detections);
      command = target_state_is({BUOY_CAPTURED}) ? lift(d) : commit(d);
      if (target_physical_detached(pose, now)) {
        capture = true;
        detach_target(now);
        transition(MISSION_DETACH_CONFIRM, now);
      } else if (target_state_is({BUOY_CAPTURED})) {
        transition(MISSION_LIFT_DETACH, now);
      } else if (d && inside_capture(*d)) {
        capture = true;
        transition(MISSION_CAPTURE_CHECK, now);
      } else if (commit_start_ && now - *commit_start_ > cfg_.commit_timeout) {
        retry_current_target(now);
      }
    } else if (state_ == MISSION_CAPTURE_CHECK) {
      auto d = target_detection(detections);
      command = contact_push(d);
      if (target_physical_detached(pose, now)) {
        capture = true;
        detach_target(now);
        transition(MISSION_DETACH_CONFIRM, now);
      } else if (d && inside_capture(*d)) {
        capture = true;
        if (commit_start_ && now - *commit_start_ > cfg_.commit_timeout) retry_current_target(now);
      } else if (!d || !inside_capture(*d)) {
        capture_timer_ = 0.0;
        transition(MISSION_COMMIT, now);
      }
    } else if (state_ == MISSION_LIFT_DETACH) {
      auto d = target_detection(detections);
      command = lift(d);
      if (target_physical_detached(pose, now)) {
        detach_target(now);
        transition(MISSION_DETACH_CONFIRM, now);
      } else if (lift_retry_needed(now)) {
        lift_start_ = now;
        lift_start_z_ = pose.xyz.z;
      }
    } else if (state_ == MISSION_DETACH_CONFIRM) {
      if (target_state_is({BUOY_RISING, BUOY_FLOATING})) {
        target_id_.reset();
        transition(MISSION_NEXT_TARGET, now);
      }
    } else if (state_ == MISSION_NEXT_TARGET) {
      if (remaining_attached() > 0) transition(MISSION_SEARCH, now);
      else transition(MISSION_ASCEND, now);
    } else if (state_ == MISSION_ASCEND) {
      command = surface_depth_command(pose, MISSION_ASCEND);
      if (pose.xyz.z > cfg_.surface_threshold_z) {
        mode_ = "SURFACE";
        transition(MISSION_SURFACE_READY, now);
      }
    } else if (state_ == MISSION_SURFACE_READY) {
      mode_ = "SURFACE";
      command = zero(MISSION_SURFACE_READY);
      if (!surface_candidates(now).empty()) {
        surface_collect_start_ = now;
        transition(MISSION_SURFACE_COLLECT, now);
      } else if (all_required_collected()) {
        begin_score_transit(pose, now);
      }
    } else if (state_ == MISSION_SURFACE_COLLECT) {
      mode_ = "SURFACE";
      mark_surface_contacts(pose, now);
      if (surface_candidates(now).empty() && all_required_collected()) {
        begin_score_transit(pose, now);
        target_id_.reset();
        command = surface_waypoint_for_collector(pose, score_zone_, MISSION_SCORE_TRANSIT);
      } else
      if (cfg_.surface_collect_ground_truth && !surface_candidates(now).empty()) {
        mark_surface_candidates_collected(now);
        target_id_.reset();
        begin_score_transit(pose, now);
        command = surface_waypoint_for_collector(pose, score_zone_, MISSION_SCORE_TRANSIT);
      } else {
        auto idx = select_surface_buoy(pose, now);
        if (!idx) {
          if (all_required_collected()) {
            begin_score_transit(pose, now);
            command = surface_waypoint_for_collector(pose, score_zone_, MISSION_SCORE_TRANSIT);
          } else {
            command = zero(MISSION_SURFACE_COLLECT);
          }
        } else {
          target_id_ = buoys_[*idx].target.name;
          command = surface_collect(pose, buoys_[*idx], now);
          if (auto yolo = fresh_yolo(now)) {
            command = merge_surface_yolo_guidance(command, *yolo);
            if (yolo_surface_capture_ready(*yolo)) {
              if (!surface_yolo_capture_start_) surface_yolo_capture_start_ = now;
              if (now - *surface_yolo_capture_start_ >= cfg_.yolo_surface_capture_hold) {
                mark_surface_contacts(pose, now);
                surface_yolo_capture_start_.reset();
              }
            } else {
              surface_yolo_capture_start_.reset();
            }
          } else {
            surface_yolo_capture_start_.reset();
          }
          mark_surface_contacts(pose, now);
          if (surface_candidates(now).empty() && all_required_collected()) {
            begin_score_transit(pose, now);
            target_id_.reset();
          }
        }
      }
    } else if (state_ == MISSION_SCORE_TRANSIT) {
      mode_ = "SURFACE";
      if (!score_zone_) begin_score_transit(pose, now);
      command = surface_waypoint_for_collector(pose, score_zone_, MISSION_SCORE_TRANSIT);
      if (inside_score_zone(pose)) {
        score_confirm_start_.reset();
        transition(MISSION_SCORE_CONFIRM, now);
      }
    } else if (state_ == MISSION_SCORE_CONFIRM) {
      mode_ = "SURFACE";
      const bool collector_in_zone = inside_score_zone(pose);
      const bool collected_in_zone = collected_buoys_inside_score_zone(now);
      if (!collector_in_zone && !collected_in_zone) {
        transition(MISSION_SCORE_TRANSIT, now);
        command = surface_waypoint_for_collector(pose, score_zone_, MISSION_SCORE_TRANSIT);
      } else {
        command = score_dump(pose);
        if (collected_in_zone) {
          if (!score_confirm_start_) score_confirm_start_ = now;
        } else {
          score_confirm_start_.reset();
        }
        if (collected_in_zone && all_required_collected() && score_confirm_start_ &&
            now - *score_confirm_start_ >= cfg_.score_confirm) {
          mark_scored();
          if (all_required_scored()) {
            target_id_.reset();
            transition(MISSION_COMPLETE, now);
            command = zero(MISSION_COMPLETE);
          }
        }
      }
    } else if (state_ == MISSION_COMPLETE) {
      mode_ = "SURFACE";
      command = zero(MISSION_COMPLETE);
    }

    auto d = target_detection(detections);
    if (d && inside_capture(*d)) capture = true;
    command = phase_command_guard(command);
    command = boundary_guard(pose, command);
    command = phase_command_guard(command);
    return step(command, d, capture);
  }

  std::string status_json(const std::optional<Step> &step_opt, const std::optional<Pose> &pose, double now,
                          bool waiting_for_pose, std::optional<bool> armed, bool waiting_for_arm) const {
    const Command command = step_opt ? step_opt->command : zero_const(state_);
    const std::string state = waiting_for_arm ? WAIT_ARM_PHASE : (step_opt ? step_opt->state : state_);
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{";
    out << "\"armed\":" << (armed ? (*armed ? "true" : "false") : "null") << ",";
    out << "\"waiting_for_arm\":" << (waiting_for_arm ? "true" : "false") << ",";
    out << "\"waiting_for_pose\":" << (waiting_for_pose ? "true" : "false") << ",";
    out << "\"build_tag\":" << q(MISSION_FSM_BUILD_TAG) << ",";
    out << "\"time_s\":" << now << ",";
    out << "\"mission_elapsed_s\":" << elapsed(now) << ",";
    out << "\"state\":" << q(state) << ",";
    const std::string robot_state = robot_state_for_status(state, step_opt, waiting_for_pose, waiting_for_arm);
    out << "\"robot_state\":" << q(robot_state) << ",";
    out << "\"robot_state_label\":" << q(robot_state_label(robot_state)) << ",";
    out << "\"mode\":" << q(step_opt ? step_opt->mode : mode_) << ",";
    const auto tid = step_opt ? step_opt->target_id : target_id_;
    out << "\"target_id\":" << (tid ? q(*tid) : "null") << ",";
    out << "\"target_class\":" << q(step_opt ? step_opt->target_class : "") << ",";
    out << "\"target_state\":" << q(step_opt ? step_opt->target_state : target_state()) << ",";
    out << "\"capture_flag\":" << ((step_opt && step_opt->capture_flag) ? "true" : "false") << ",";
    out << "\"remaining_attached\":" << (step_opt ? step_opt->remaining_attached : remaining_attached()) << ",";
    out << "\"processed_count\":" << processed_.size() << ",";
    out << "\"failed_count\":" << failed_.size() << ",";
    out << "\"collected_count\":" << collected_.size() << ",";
    out << "\"scored_count\":" << scored_.size() << ",";
    out << "\"required_buoy_count\":" << buoys_.size() << ",";
    out << "\"processed_buoys\":" << string_array(processed_) << ",";
    out << "\"failed_buoys\":" << string_array(failed_) << ",";
    out << "\"collected_buoys\":" << string_array(collected_) << ",";
    out << "\"scored_buoys\":" << string_array(scored_) << ",";
    out << "\"mission_policy\":{\"own_course\":" << q(cfg_.own_course)
        << ",\"course_boundary_x_m\":" << cfg_.course_boundary_x
        << ",\"course_boundary_margin_m\":" << cfg_.course_boundary_margin
        << ",\"course_boundary_standoff_m\":" << cfg_.course_boundary_standoff << "},";
    out << "\"live_status\":{\"required\":" << (cfg_.require_live_status ? "true" : "false")
        << ",\"stale\":" << (live_status_stale(now) ? "true" : "false")
        << ",\"timeout_s\":" << live_status_timeout_s() << ",\"latest_age_s\":";
    if (latest_live_received_s_) out << std::max(0.0, now - *latest_live_received_s_);
    else out << "null";
    out << "},";
    out << "\"surface_collection\":{\"remaining\":" << surface_candidates(now).size()
        << ",\"candidate_ids\":" << candidate_ids_json(now)
        << ",\"ground_truth_collect\":" << (cfg_.surface_collect_ground_truth ? "true" : "false")
        << ",\"collector_offset\":" << vec_json(cfg_.surface_collector_offset)
        << ",\"collector_xyz\":";
    if (pose) out << vec_json(surface_collector_world(*pose));
    else out << "null";
    out << ",\"x_window_m\":" << effective_surface_collect_x_window()
        << ",\"y_window_m\":" << effective_surface_collect_y_window()
        << ",\"z_window_m\":" << effective_surface_collect_z_window()
        << ",\"timeout_s\":" << cfg_.surface_collect_timeout << ",\"elapsed_s\":";
    if (surface_collect_start_) out << std::max(0.0, now - *surface_collect_start_);
    else out << "null";
    out << "},";
    out << "\"yolo_surface\":";
    if (yolo_) {
      const bool fresh = now - yolo_->received_s <= cfg_.yolo_surface_timeout;
      out << "{\"valid\":" << (yolo_->valid ? "true" : "false")
          << ",\"fresh\":" << (fresh ? "true" : "false")
          << ",\"active\":" << (yolo_->active ? "true" : "false")
          << ",\"model_found\":" << (yolo_->model_found ? "true" : "false")
          << ",\"has_target\":" << (yolo_->has_target ? "true" : "false")
          << ",\"capture\":" << (yolo_->capture ? "true" : "false")
          << ",\"count\":" << yolo_->count
          << ",\"state\":" << q(yolo_->state)
          << ",\"label\":" << q(yolo_->label)
          << ",\"side\":" << q(yolo_->side)
          << ",\"center_norm\":[" << number_json(yolo_->center_x_norm) << "," << number_json(yolo_->center_y_norm) << "]"
          << ",\"error_norm\":[" << number_json(yolo_->error_x_norm) << "," << number_json(yolo_->error_y_norm) << "]"
          << ",\"height_ratio\":" << yolo_->height_ratio
          << ",\"area_ratio\":" << yolo_->area_ratio
          << ",\"age_s\":" << std::max(0.0, now - yolo_->received_s)
          << ",\"capture_hold_s\":";
      if (surface_yolo_capture_start_) out << std::max(0.0, now - *surface_yolo_capture_start_);
      else out << "null";
      out << "}";
    } else {
      out << "null";
    }
    out << ",";
    out << "\"score_zone\":{\"id\":" << (score_zone_id_ ? q(*score_zone_id_) : "null")
        << ",\"xyz\":" << (score_zone_ ? vec_json(*score_zone_) : "null")
        << ",\"radius_m\":" << cfg_.score_zone_radius
        << ",\"entered\":" << ((pose && inside_score_zone(*pose)) ? "true" : "false")
        << ",\"released_buoys_in_zone_count\":" << released_buoys_inside_score_zone_count(now)
        << ",\"all_required_collected\":" << (all_required_collected() ? "true" : "false")
        << ",\"all_required_released_in_zone\":" << (collected_buoys_inside_score_zone(now) ? "true" : "false")
        << ",\"all_required_scored\":" << (all_required_scored() ? "true" : "false")
        << ",\"collected_buoys_in_zone\":" << (collected_buoys_inside_score_zone(now) ? "true" : "false")
        << "},";
    out << "\"detection\":";
    if (step_opt && step_opt->detection) detection_json(out, *step_opt->detection);
    else out << "null";
    out << ",";
    out << "\"command\":{\"forward\":" << (waiting_for_arm ? 0.0 : command.forward)
        << ",\"sway\":" << (waiting_for_arm ? 0.0 : command.sway)
        << ",\"heave\":" << (waiting_for_arm ? 0.0 : command.heave)
        << ",\"yaw\":" << (waiting_for_arm ? 0.0 : command.yaw)
        << ",\"pitch\":" << (waiting_for_arm ? 0.0 : command.pitch)
        << ",\"phase\":" << q(waiting_for_arm ? WAIT_ARM_PHASE : command.phase) << "},";
    out << "\"robot\":";
    if (pose) {
      out << "{\"x\":" << pose->xyz.x << ",\"y\":" << pose->xyz.y << ",\"z\":" << pose->xyz.z
          << ",\"depth_m\":" << std::max(0.0, -pose->xyz.z) << ",\"yaw_rad\":" << pose->yaw << "}";
    } else {
      out << "null";
    }
    out << ",\"buoys\":[";
    for (size_t i = 0; i < buoys_.size(); ++i) {
      if (i) out << ",";
      buoy_json(out, buoys_[i], now);
    }
    out << "]}";
    return out.str();
  }

  std::vector<std::string> csv_header() const {
    return {"time_s", "state", "mode", "target_id", "target_class", "target_x_intake",
            "target_y_intake", "target_z_intake", "cmd_vx", "cmd_vy", "cmd_vz", "cmd_yaw", "cmd_pitch",
            "capture_flag", "buoy_state", "remaining_attached", "processed_count", "failed_count",
            "robot_depth_m", "robot_yaw_rad"};
  }

  std::vector<std::string> csv_row(const Step &s, const Pose &pose, double now) const {
    Vec3 p{NAN, NAN, NAN};
    if (s.detection) p = s.detection->p_intake;
    else if (s.command.phase == MISSION_SURFACE_COLLECT || s.command.phase == MISSION_SCORE_TRANSIT ||
             s.command.phase == MISSION_SCORE_CONFIRM) {
      p = s.command.error;
    }
    auto f = [](double v, int prec = 4) {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(prec) << v;
      return ss.str();
    };
    return {f(now, 3), s.state, s.mode, s.target_id.value_or(""), s.target_class, f(p.x), f(p.y), f(p.z),
            f(s.command.forward), f(s.command.sway), f(s.command.heave), f(s.command.yaw),
            f(s.command.pitch), s.capture_flag ? "1" : "0", s.target_state, std::to_string(s.remaining_attached),
            std::to_string(s.processed_count), std::to_string(s.failed_count), f(std::max(0.0, -pose.xyz.z)),
            f(pose.yaw)};
  }

  const std::string &state() const { return state_; }

#ifdef MISSION_FSM_CORE_ONLY
  void force_state_for_test(const std::string &state, const std::string &mode, double now) {
    state_ = state;
    mode_ = mode;
    state_entered_ = now;
    if (!mission_start_) mission_start_ = now;
  }

  void force_detached_for_test(const std::string &id, double now, bool floating = true) {
    mark_detached(buoy(id), now);
    if (floating) buoy(id).state = BUOY_FLOATING;
  }

  void force_target_for_test(const std::string &id) {
    target_id_ = id;
    buoy(id).state = BUOY_TARGETED;
  }

  size_t collected_count_for_test() const { return collected_.size(); }
  size_t scored_count_for_test() const { return scored_.size(); }
  bool all_required_collected_for_test() const { return all_required_collected(); }
  bool all_required_scored_for_test() const { return all_required_scored(); }
  size_t surface_candidate_count_for_test(double now) const { return surface_candidates(now).size(); }
  std::string buoy_state_for_test(const std::string &id) const { return buoy(id).state; }
  Vec3 target_xyz_for_test(const std::string &id, double now) const { return target_xyz(buoy(id), now); }
	  Command approach_far_for_test(const Detection &d) const { return approach_far(d); }
	  Command approach_near_for_test(const Detection &d) const { return approach_near(d); }
	  Command approach_unstick_for_test(const Detection &d, bool near) const { return approach_unstick(d, near); }
  Command surface_waypoint_for_test(const Pose &pose, Vec3 target) const {
    return surface_waypoint(pose, std::optional<Vec3>{target}, MISSION_SCORE_TRANSIT);
  }
  Command surface_depth_command_for_test(const Pose &pose) const {
    return surface_depth_command(pose, MISSION_ASCEND);
  }
  Command pinger_hydrophone_direct_for_test(const Detection &d) const {
    return pinger_hydrophone_direct(d, MISSION_APPROACH_FAR);
  }
  void update_yolo_for_test(const YoloGuidance &guidance) { update_yolo(guidance); }
  Command boundary_guard_for_test(const Pose &pose, const Command &cmd) const { return boundary_guard(pose, cmd); }
#endif

 private:
  bool state_is_terminal_path() const {
    static const std::set<std::string> allowed = {MISSION_NEXT_TARGET, MISSION_ASCEND, MISSION_SURFACE_READY,
                                                  MISSION_SURFACE_COLLECT, MISSION_SCORE_TRANSIT,
                                                  MISSION_SCORE_CONFIRM, MISSION_COMPLETE};
    return allowed.count(state_) > 0;
  }

  MissionBuoy &buoy(const std::string &id) { return buoys_.at(index_by_name_.at(id)); }
  const MissionBuoy &buoy(const std::string &id) const { return buoys_.at(index_by_name_.at(id)); }

  std::optional<LiveState> live_for(const MissionBuoy &b, double now) const {
    auto it = live_.find(b.target.name);
    if (it == live_.end()) return std::nullopt;
    if (now - it->second.received_s > cfg_.live_buoy_timeout) return std::nullopt;
    return it->second;
  }

  double live_status_timeout_s() const {
    if (!cfg_.require_live_status) return cfg_.live_status_timeout;
    const double live_timeout = std::max(0.0, cfg_.live_buoy_timeout);
    const double status_timeout = std::max(0.0, cfg_.live_status_timeout);
    if (live_timeout <= 0.0) return status_timeout;
    if (status_timeout <= 0.0) return live_timeout;
    return std::min(live_timeout, status_timeout);
  }

  bool live_status_stale(double now) const {
    if (!cfg_.require_live_status || buoys_.empty()) return false;
    if (!latest_live_received_s_) return true;
    return now - *latest_live_received_s_ > live_status_timeout_s();
  }

  Vec3 target_xyz(const MissionBuoy &b, double now) const {
    const auto live = live_for(b, now);
    if (live && (b.state == BUOY_ATTACHED || b.state == BUOY_TARGETED || b.state == BUOY_CAPTURED)) {
      if (live->attach_xyz) return *live->attach_xyz;
      if (live->magnet_xyz) return *live->magnet_xyz;
      if (live->body_xyz) return *live->body_xyz;
      return live->target_xyz;
    }
    if (b.state == BUOY_RISING || b.state == BUOY_FLOATING || b.state == BUOY_COLLECTED || b.state == BUOY_SCORED) {
      if (live) return live->target_xyz;
      double z = 0.05;
      if (b.state == BUOY_RISING && b.detach_time) {
        const double progress = clamp((now - *b.detach_time) / std::max(cfg_.rise_to_float, 1.0e-6), 0.0, 1.0);
        z = b.target.xyz.z + progress * (0.05 - b.target.xyz.z);
      }
      return {b.target.xyz.x, b.target.xyz.y, z};
    }
    return b.target.xyz;
  }

  Vec3 p_intake(const Pose &pose, const MissionBuoy &b, double now) const {
    const Vec3 intake_world = add(pose.xyz, rotate_body_to_world(cfg_.intake_offset, pose.yaw));
    return rotate_world_to_body(sub(target_xyz(b, now), intake_world), pose.yaw);
  }

  Vec3 surface_collector_world(const Pose &pose) const {
    return add(pose.xyz, rotate_body_to_world(cfg_.surface_collector_offset, pose.yaw));
  }

  Vec3 p_surface_collector(const Pose &pose, const MissionBuoy &b, double now) const {
    return rotate_world_to_body(sub(target_xyz(b, now), surface_collector_world(pose)), pose.yaw);
  }

  std::vector<Detection> fake_vision(const Pose &pose, double now) {
    std::vector<Detection> detections;
    for (auto &b : buoys_) {
      if (!(b.state == BUOY_ATTACHED || b.state == BUOY_TARGETED || b.state == BUOY_CAPTURED)) continue;
      auto live = live_for(b, now);
      if (live && live->detached) continue;
      const Vec3 p = p_intake(pose, b, now);
      const double dist = norm(p);
      if (dist > cfg_.fake_vision_range) continue;
      Detection d;
      d.buoy_id = b.target.name;
      d.class_name = b.target.color;
      d.p_intake = p;
      d.distance = dist;
      d.confidence = clamp(1.0 - dist / std::max(cfg_.fake_vision_range, 1.0e-6), 0.15, 1.0);
      d.target_xyz = target_xyz(b, now);
      d.has_target_xyz = true;
      d.coordinate_source = live ? live->source : "scene_static";
      detections.push_back(d);
    }
    std::sort(detections.begin(), detections.end(), [](const Detection &a, const Detection &b) {
      return a.distance < b.distance;
    });
    return detections;
  }

  void apply_detection_rates(std::vector<Detection> &detections, double now) {
    std::unordered_map<std::string, std::pair<Vec3, double>> next;
    next.reserve(detections.size());
    for (auto &d : detections) {
      const auto it = last_detection_.find(d.buoy_id);
      if (it != last_detection_.end()) {
        const double dt = now - it->second.second;
        if (dt > 1.0e-4 && dt < 2.0) d.p_rate = scale(sub(d.p_intake, it->second.first), 1.0 / dt);
      }
      d.relative_speed = norm(d.p_rate);
      next[d.buoy_id] = {d.p_intake, now};
    }
    last_detection_ = std::move(next);
  }

  void sync_live(double now) {
    for (auto &b : buoys_) {
      auto live = live_for(b, now);
      if (!live || !(live->detached || (live->eq_active && !*live->eq_active))) continue;
      if (b.state == BUOY_RISING || b.state == BUOY_FLOATING || b.state == BUOY_COLLECTED || b.state == BUOY_SCORED) {
        continue;
      }
      mark_detached(b, now);
    }
  }

  void update_rising(double now) {
    for (auto &b : buoys_) {
      if (b.state == BUOY_RISING && b.detach_time && now - *b.detach_time >= cfg_.rise_to_float) b.state = BUOY_FLOATING;
    }
  }

  void transition(const std::string &s, double now) {
    if (state_ == s) return;
    state_ = s;
    state_entered_ = now;
    reset_approach_progress();
    if (s != MISSION_SURFACE_COLLECT) surface_yolo_capture_start_.reset();
  }

  Command make(double f, double sw, double h, double y, const std::string &phase, Vec3 error, double pitch = 0.0) const {
    return {clamp(f, -1, 1), clamp(sw, -1, 1), clamp(h, -1, 1), clamp(y, -1, 1), clamp(pitch, -1, 1), phase, norm(error), error,
            phase == MISSION_COMPLETE};
  }

  Command zero(const std::string &phase) const { return make(0, 0, 0, 0, phase, {}); }
  Command zero_const(const std::string &phase) const { return make(0, 0, 0, 0, phase, {}); }

  Command depth_command(const Pose &pose, double target_z, const std::string &phase) const {
    const double error_z = target_z - pose.xyz.z;
    double heave = clamp(-cfg_.k_depth * error_z, -cfg_.max_depth_heave, cfg_.max_depth_heave);
    if (std::abs(error_z) > cfg_.depth_tolerance && std::abs(heave) > 0.0 && std::abs(heave) < cfg_.min_depth_command) {
      heave = std::copysign(cfg_.min_depth_command, heave);
    }
    return make(0, 0, heave, 0, phase, {0, 0, error_z});
  }

  Command surface_depth_command(const Pose &pose, const std::string &phase) const {
    const double error_z = cfg_.surface_ready_z - pose.xyz.z;
    double limit = cfg_.max_surface_ascent_near_heave;
    if (pose.xyz.z < -2.0) limit = cfg_.max_surface_ascent_far_heave;
    else if (pose.xyz.z < -1.0) limit = cfg_.max_surface_ascent_mid_heave;
    double heave = clamp(-cfg_.k_depth * error_z, -limit, limit);
    if (std::abs(error_z) <= 0.06) heave = 0.0;
    return make(0, 0, heave, 0, phase, {0, 0, error_z});
  }

  bool depth_reached(const Pose &pose, double target_z) const {
    return std::abs(target_z - pose.xyz.z) <= cfg_.depth_tolerance;
  }

  std::vector<Detection> attached_detections(const std::vector<Detection> &detections) const {
    std::vector<Detection> out;
    for (const auto &d : detections) {
      const auto &b = buoy(d.buoy_id);
      if ((b.state == BUOY_ATTACHED || b.state == BUOY_TARGETED) && !b.failed) out.push_back(d);
    }
    return out;
  }

  bool ready_to_search_from_dive(const Pose &pose, const std::vector<Detection> &detections) const {
    if (std::max(0.0, -pose.xyz.z) < cfg_.dive_min_depth) return false;
    for (const auto &d : attached_detections(detections)) {
      if (d.distance <= cfg_.dive_search_distance && std::abs(d.p_intake.z) <= cfg_.dive_search_vertical_window) return true;
    }
    return false;
  }

  std::optional<Detection> pinger_detection(const std::vector<Detection> &detections) const {
    std::optional<Detection> best;
    for (const auto &d : attached_detections(detections)) {
      if (buoy(d.buoy_id).target.course != "pinger") continue;
      if (!best || d.distance < best->distance) best = d;
    }
    return best;
  }

  void track_pinger_during_dive(const Detection &d) {
    target_id_ = d.buoy_id;
    auto &b = buoy(d.buoy_id);
    if (!b.processed && !b.failed) b.state = BUOY_TARGETED;
    target_lost_since_.reset();
  }

  bool detection_is_pinger(const Detection &d) const {
    return index_by_name_.count(d.buoy_id) && buoy(d.buoy_id).target.course == "pinger";
  }

  Command pinger_dive_approach(const Pose &pose, const Detection &d, double now) const {
    Command cmd = pinger_target_approach(pose, d, now, MISSION_DIVE);
    const Command depth = depth_command(pose, cfg_.search_depth_z, MISSION_DIVE);
    if (d.distance > cfg_.pinger_yolo_final_range && std::abs(depth.heave) > std::abs(cmd.heave)) {
      cmd.heave = depth.heave;
    }
    cmd.phase = MISSION_DIVE;
    return cmd;
  }

  Command pinger_target_approach(const Pose &pose, const Detection &d, double now, const std::string &phase) const {
    (void)pose;
    if (!cfg_.pinger_hydrophone_enable || d.distance <= cfg_.pinger_yolo_final_range) {
      if (auto yolo = fresh_yolo(now)) {
        return pinger_yolo_precision(d, *yolo, phase);
      }
    }
    return pinger_hydrophone_direct(d, phase);
  }

  Command pinger_hydrophone_direct(const Detection &d, const std::string &phase) const {
    const Vec3 p = d.p_intake;
    const double yaw_error = target_bearing_error(p);
    const double yaw_abs = std::abs(yaw_error);
    const double z_abs = std::abs(p.z);
    double forward = cfg_.pinger_hydrophone_forward_fast;
    if (d.distance < 1.0) forward = cfg_.pinger_hydrophone_forward_mid;
    if (d.distance < cfg_.pinger_yolo_near_range) forward = cfg_.pinger_hydrophone_forward_slow;
    const bool diagonal_dive = phase == MISSION_DIVE;
    if (yaw_abs > 0.42) forward = diagonal_dive ? std::min(forward, 0.45) : 0.0;
    else if (yaw_abs > 0.25) forward = diagonal_dive ? std::min(forward, 0.62) : std::min(forward, 0.10);
    else if (yaw_abs > 0.14) forward = std::min(forward, diagonal_dive ? 0.82 : 0.34);
    if (p.x < 0.15) forward = 0.0;
    else if (p.x < 0.70 && (yaw_abs > 0.22 || z_abs > 0.35)) forward = 0.0;
    else if (p.x < 1.35 && (yaw_abs > 0.34 || z_abs > 0.55)) forward = std::min(forward, 0.08);
    else if (p.x < 2.20 && z_abs > 0.85) forward = std::min(forward, 0.18);
    const double yaw_limit = yaw_abs > 0.45 ? cfg_.max_yaw_far : cfg_.max_yaw_near;
    const double heave = heave_align(p.z, cfg_.kz_far, far_heave_limit(p.z), cfg_.capture_z);
    return make(forward, 0.0, heave,
                stabilized_yaw(yaw_error, cfg_.pinger_hydrophone_yaw_gain, yaw_limit),
                phase, p);
  }

  Command pinger_yolo_precision(const Detection &d, const YoloGuidance &y, const std::string &phase) const {
    const double ex = std::isfinite(y.error_x_norm) ? y.error_x_norm : 0.0;
    const double ey = std::isfinite(y.error_y_norm) ? y.error_y_norm : 0.0;
    const bool centered = std::abs(ex) <= cfg_.yolo_surface_center_tolerance;
    double forward = centered ? cfg_.pinger_yolo_forward : 0.08;
    if (y.capture || yolo_surface_capture_ready(y)) forward = std::max(forward, 0.34);
    if (d.distance < 0.10) forward = std::min(forward, 0.20);
    const double yaw = clamp(-cfg_.pinger_yolo_yaw_gain * ex, -cfg_.max_yaw_near, cfg_.max_yaw_near);
    const double sway = clamp(-cfg_.pinger_yolo_sway_gain * ex, -cfg_.max_vy_near, cfg_.max_vy_near);
    const double heave_from_detection = heave_align(d.p_intake.z, cfg_.kz_near, cfg_.max_vz_near, cfg_.capture_z);
    const double heave_from_image = clamp(-0.10 * ey, -0.10, 0.10);
    const double heave = std::abs(heave_from_detection) > std::abs(heave_from_image) ? heave_from_detection : heave_from_image;
    return make(forward, sway, heave, yaw, phase, {d.distance, ex, ey});
  }

  std::optional<Detection> select_detection(const std::vector<Detection> &detections) const {
    const auto candidates = attached_detections(detections);
    for (const auto &d : candidates) {
      if (buoy(d.buoy_id).target.course == "pinger") return d;
    }
    if (!candidates.empty()) {
      return *std::min_element(candidates.begin(), candidates.end(), [](const Detection &a, const Detection &b) {
        return a.distance < b.distance;
      });
    }
    return std::nullopt;
  }

  std::optional<Detection> target_detection(const std::vector<Detection> &detections) const {
    if (!target_id_) return std::nullopt;
    for (const auto &d : detections) {
      if (d.buoy_id == *target_id_) return d;
    }
    return std::nullopt;
  }

  double heave_align(double z, double gain, double limit, double tolerance) const {
    double h = clamp(-gain * z, -limit, limit);
    if (std::abs(z) > tolerance && std::abs(h) > 0.0 && std::abs(h) < cfg_.min_align_heave) {
      h = std::copysign(std::min(limit, cfg_.min_align_heave), h);
    }
    return h;
  }

  double far_heave_limit(double z) const {
    if (std::abs(z) > 1.0) return cfg_.max_depth_heave;
    if (std::abs(z) > 0.55) return std::max(cfg_.max_vz_far, 0.62);
    return cfg_.max_vz_far;
  }

  double yaw_first_forward(double forward, double yaw_abs, double hard_rad, double slow_rad, double trim_rad,
                           double crawl_limit, double slow_limit, double trim_limit) const {
    if (forward <= 0.0) return forward;
    if (yaw_abs > hard_rad) return std::min(forward, crawl_limit);
    if (yaw_abs > slow_rad) return std::min(forward, slow_limit);
    if (yaw_abs > trim_rad) return std::min(forward, trim_limit);
    return forward;
  }

  double yaw_locked_forward(double forward, double yaw_abs, double stop_rad, double crawl_rad, double slow_rad,
                            double crawl_limit, double slow_limit) const {
    if (yaw_abs > stop_rad) return 0.0;
    if (yaw_abs > crawl_rad) return clamp(forward, -crawl_limit, crawl_limit);
    if (yaw_abs > slow_rad) return clamp(forward, -slow_limit, slow_limit);
    return forward;
  }

  double target_bearing_error(const Vec3 &p) const {
    double err = std::atan2(p.y, p.x);
    if (p.x < 0.0 && std::abs(p.y) < 0.12) err = p.y < 0.0 ? -M_PI : M_PI;
    return wrap_pi(err);
  }

  void update_yaw_rate(const Pose &pose, double now) {
    if (!last_pose_yaw_ || !last_pose_time_) {
      last_pose_yaw_ = pose.yaw;
      last_pose_time_ = now;
      return;
    }
    const double dt = now - *last_pose_time_;
    const double dyaw = wrap_pi(pose.yaw - *last_pose_yaw_);
    if (dt < 1.0e-3 || std::abs(dyaw) < 1.0e-7) return;
    const double raw_rate = clamp(dyaw / dt, -4.0, 4.0);
    const double alpha = clamp(dt / 0.12, 0.18, 0.65);
    yaw_rate_ += alpha * (raw_rate - yaw_rate_);
    last_pose_yaw_ = pose.yaw;
    last_pose_time_ = now;
  }

  double stabilized_yaw(double error, double gain, double limit) const {
    if (std::abs(error) < 0.025 && std::abs(yaw_rate_) < 0.04) return 0.0;
    return clamp(gain * error - cfg_.yaw_rate_damping * yaw_rate_, -limit, limit);
  }

  double contact_bearing_error(const Vec3 &p) const {
    return wrap_pi(std::atan2(p.y, std::max(p.x, 0.35)));
  }

  double yaw_first_sway(double requested, double yaw_abs, double trim_rad, double off_rad,
                        double aligned_limit, double trim_limit) const {
    double limit = aligned_limit;
    if (yaw_abs > off_rad) limit = 0.0;
    else if (yaw_abs > trim_rad) limit = std::min(aligned_limit, trim_limit);
    return clamp(requested, -limit, limit);
  }

  double differential_forward(double requested, const Vec3 &p, double yaw_abs, double bearing_rate_abs,
                              bool near, bool allow_midrange_drive = false) const {
    if (requested <= 0.0) return requested;
    const double lateral_abs = std::abs(p.y);
    if (p.x < (near ? 0.18 : 0.35)) return 0.0;
    if (!near && p.x > 4.0) {
      if (yaw_abs > 1.15 || bearing_rate_abs > 1.40) return 0.0;
      if (yaw_abs > 0.85) return std::min(requested, 0.28);
      if (yaw_abs > 0.60) return std::min(requested, 0.42);
      if (yaw_abs > 0.36) return std::min(requested, 0.60);
      if (yaw_abs > 0.18) return std::min(requested, 0.78);
      if (yaw_abs > 0.09 && lateral_abs > 2.5) return std::min(requested, 0.85);
      return requested;
    }
    if (!near && p.x > 1.20) {
      if (yaw_abs > 1.25 || bearing_rate_abs > 1.20) return 0.0;
      double limit = requested;
      if (yaw_abs > 0.95) limit = std::min(requested, allow_midrange_drive ? 0.42 : 0.34);
      else if (yaw_abs > 0.65) limit = std::min(requested, allow_midrange_drive ? 0.58 : 0.50);
      else if (yaw_abs > 0.42) limit = std::min(requested, 0.70);
      else if (yaw_abs > 0.24) limit = std::min(requested, 0.86);
      else if (lateral_abs > 1.20) limit = std::min(requested, 0.90);
      const double floor = (yaw_abs < 0.50 && bearing_rate_abs < 0.45) ? cfg_.far_drive_min : 0.0;
      if (p.x > 2.0 && yaw_abs < 0.36 && bearing_rate_abs < 0.25) {
        return std::max(limit, std::min(requested, cfg_.far_drive_fast_min));
      }
      return floor > 0.0 ? std::max(limit, std::min(requested, floor)) : limit;
    }
    if (yaw_abs > (near ? 0.38 : 0.46) || bearing_rate_abs > (near ? 0.42 : 0.55)) return 0.0;
    if (lateral_abs > (near ? 0.42 : 0.80) && yaw_abs > (near ? 0.10 : 0.12)) {
      return std::min(requested, near ? 0.06 : 0.12);
    }
    if (lateral_abs > (near ? 0.26 : 0.50) && yaw_abs > (near ? 0.06 : 0.08)) {
      return std::min(requested, near ? 0.12 : 0.28);
    }
    if (yaw_abs > (near ? 0.24 : 0.30)) return std::min(requested, near ? 0.08 : 0.12);
    if (yaw_abs > (near ? 0.14 : 0.18)) return std::min(requested, near ? 0.18 : 0.38);
    if (yaw_abs > (near ? 0.08 : 0.10)) return std::min(requested, near ? 0.34 : 0.65);
    return requested;
  }

  double yaw_first_surface_forward(double forward, double yaw_abs) const {
    if (forward <= 0.0) return yaw_abs > 0.55 ? 0.0 : forward;
    if (yaw_abs > 0.70) return 0.0;
    if (yaw_abs > 0.45) return std::min(forward, 0.08);
    if (yaw_abs > 0.28) return std::min(forward, 0.22);
    return forward;
  }

  Command approach_far(const Detection &d) const {
    const auto &p = d.p_intake;
    const bool pinger_target = index_by_name_.count(d.buoy_id) && buoy(d.buoy_id).target.course == "pinger";
    const double yaw_error = target_bearing_error(p);
    const double yaw_abs = std::abs(yaw_error);
    const double bearing_rate =
        (p.x * d.p_rate.y - p.y * d.p_rate.x) / std::max(p.x * p.x + p.y * p.y, 1.0);
    const double bearing_rate_abs = std::abs(bearing_rate);
    const double lateral_abs = std::abs(p.y);

    double forward = 0.0;
    if (p.x > 0.20 && yaw_abs < 1.28 && bearing_rate_abs < 1.25) {
      forward = clamp(cfg_.kx_far * (p.x - 0.45), 0.0, cfg_.max_vx_far);
      if (p.x > 2.0) forward = std::max(forward, cfg_.far_drive_fast_min);
      else if (p.x > 1.20) forward = std::max(forward, cfg_.far_drive_min);
      else if (p.x > 0.65) forward = std::max(forward, 0.42);

      if (yaw_abs > 1.05) forward = std::min(forward, pinger_target ? 0.50 : 0.40);
      else if (yaw_abs > 0.78) forward = std::min(forward, pinger_target ? 0.58 : 0.48);
      else if (yaw_abs > 0.56) forward = std::min(forward, 0.66);
      else if (yaw_abs > 0.36) forward = std::min(forward, 0.82);
      else if (yaw_abs > 0.18 && lateral_abs > 0.85) forward = std::min(forward, 0.92);

      if (bearing_rate_abs > 0.65) forward = std::min(forward, 0.58);
      else if (bearing_rate_abs > 0.40) forward = std::min(forward, 0.76);
    } else if (p.x > 0.08 && yaw_abs < 0.65 && lateral_abs < 0.45) {
      forward = 0.30;
    }

    const double yaw_limit =
        std::min(cfg_.max_yaw_far, yaw_abs > 0.30 ? 0.12 : (yaw_abs > 0.12 ? 0.10 : 0.07));
    const double sway = yaw_abs < 0.06 && lateral_abs < 0.30 ? clamp(0.25 * cfg_.ky_far * p.y, -0.04, 0.04) : 0.0;
    return make(forward, sway,
                heave_align(p.z, cfg_.kz_far, far_heave_limit(p.z), cfg_.approach_near_entry_z),
                stabilized_yaw(yaw_error, cfg_.kyaw_far, yaw_limit),
                MISSION_APPROACH_FAR, p);
  }

  bool approach_stalled(const Detection &d, const std::string &state, double now) {
    if (!approach_progress_target_ || !approach_progress_state_ || *approach_progress_target_ != d.buoy_id ||
        *approach_progress_state_ != state) {
      approach_progress_target_ = d.buoy_id;
      approach_progress_state_ = state;
      approach_progress_since_ = now;
      approach_best_distance_ = d.distance;
      return false;
    }
    if (d.distance < approach_best_distance_ - cfg_.approach_progress_epsilon) {
      approach_best_distance_ = d.distance;
      approach_progress_since_ = now;
      return false;
    }
    return approach_progress_since_ && now - *approach_progress_since_ >= cfg_.approach_no_progress_timeout;
  }

  void reset_approach_progress() {
    approach_progress_target_.reset();
    approach_progress_state_.reset();
    approach_progress_since_.reset();
    approach_best_distance_ = std::numeric_limits<double>::infinity();
  }

  Command approach_unstick(const Detection &d, bool near) const {
    const auto &p = d.p_intake;
    const bool close_or_behind = p.x < 0.35;
    const double yaw_error = close_or_behind ? contact_bearing_error(p) : target_bearing_error(p);
    const double yaw_abs = std::abs(yaw_error);
    double fwd = near ? (p.x > 0.55 ? 0.64 : 0.34) : (p.x > 1.20 ? 1.00 : 0.52);
    if (p.x < -0.12) fwd = -0.24;
    else if (p.x < 0.08) fwd = -0.12;
    else if (p.x < 0.35 && (std::abs(p.y) > cfg_.capture_y || std::abs(p.z) > cfg_.capture_z)) {
      fwd = 0.10;
    }
    if (yaw_abs > 0.35) fwd = 0.0;
    else if (yaw_abs > 0.20) fwd = clamp(fwd, -0.10, 0.18);
    const double yaw_limit = near ? cfg_.max_yaw_near : cfg_.max_yaw_far;
    const double yaw = stabilized_yaw(yaw_error, near ? cfg_.kyaw_near : cfg_.kyaw_far, yaw_limit);
    const double heave =
        heave_align(p.z, cfg_.kz_near, cfg_.max_vz_near, cfg_.capture_z);
    return make(fwd, 0.0, heave, clamp(yaw, -yaw_limit, yaw_limit), near ? MISSION_APPROACH_NEAR : MISSION_APPROACH_FAR, p);
  }

  Command approach_near(const Detection &d) const {
    const auto &p = d.p_intake;
    const double yaw_error = contact_bearing_error(p);
    const double yaw_abs = std::abs(yaw_error);
    const double lateral_abs = std::abs(p.y);
    const double bearing_rate =
        (p.x * d.p_rate.y - p.y * d.p_rate.x) / std::max(p.x * p.x + p.y * p.y, 0.25);
    const double bearing_rate_abs = std::abs(bearing_rate);
    double forward = clamp(cfg_.kx_near * (p.x - 0.12), -0.10, cfg_.max_vx_near);
    if (p.x < -0.35) forward = -std::min(cfg_.max_vx_reverse_near, 0.32);
    else if (p.x < -0.12) forward = -std::min(cfg_.max_vx_reverse_near, 0.22);
    if (lateral_abs > 1.20 || yaw_abs > 0.90 || bearing_rate_abs > 0.75) {
      forward = p.x < 0.55 ? -0.10 : std::min(forward, 0.24);
    } else if (lateral_abs > 0.85 || yaw_abs > 0.60 || bearing_rate_abs > 0.45) {
      forward = std::min(forward, p.x < 0.55 ? 0.0 : 0.34);
    } else if (lateral_abs > 0.58 || yaw_abs > 0.38 || bearing_rate_abs > 0.28) {
      forward = std::min(forward, 0.48);
    } else if (lateral_abs > 0.35 || yaw_abs > 0.22 || bearing_rate_abs > 0.16) {
      forward = std::min(forward, 0.62);
    } else if (p.x > cfg_.commit_x && forward > 0.0 && forward < 0.52) {
      forward = 0.68;
    }
    const double yaw_limit = yaw_abs > 0.40 ? cfg_.max_yaw_near : (yaw_abs > 0.16 ? 0.07 : 0.05);
    const double sway = yaw_abs < 0.28 && lateral_abs < 0.60 ? clamp(0.18 * cfg_.ky_near * p.y, -0.07, 0.07) : 0.0;
    return make(forward, sway,
                heave_align(p.z, cfg_.kz_near, cfg_.max_vz_near, cfg_.capture_z),
                stabilized_yaw(yaw_error, cfg_.kyaw_near, yaw_limit), MISSION_APPROACH_NEAR, p);
  }

  Command commit(const std::optional<Detection> &d) const {
    if (!d) return make(cfg_.commit_forward, 0, 0, 0, MISSION_COMMIT, {});
    const auto &p = d->p_intake;
    const double yaw_error = contact_bearing_error(p);
    const double yaw_abs = std::abs(yaw_error);
    const double lateral_abs = std::abs(p.y);
    const bool lane = lateral_abs <= 0.20 && std::abs(p.z) <= 0.16 && yaw_abs <= 0.30;
    double forward = cfg_.commit_forward;
    if (p.x < -0.22) forward = -std::min(cfg_.max_vx_reverse_near, 0.25);
    else if (!lane) forward = (p.x > cfg_.commit_x && lateral_abs < 0.42 && yaw_abs < 0.48) ? 0.64 : 0.18;
    else if (p.x < -0.08) forward = 0.22;
    if (yaw_abs > 0.85) forward = std::min(forward, 0.18);
    else if (yaw_abs > 0.58) forward = std::min(forward, 0.34);
    else if (yaw_abs > 0.36) forward = std::min(forward, 0.56);
    const double sway = yaw_abs < 0.24 && lateral_abs < 0.42 ? clamp(0.14 * cfg_.ky_near * p.y, -0.04, 0.04) : 0.0;
    return make(forward, sway,
                heave_align(p.z, 0.75 * cfg_.kz_near, cfg_.max_vz_near, cfg_.capture_z),
                stabilized_yaw(yaw_error, 0.85 * cfg_.kyaw_near, cfg_.max_yaw_near), MISSION_COMMIT, p);
  }

  Command lift(const std::optional<Detection> &d) const {
    double sway = 0.0;
    double yaw = 0.0;
    Vec3 error{};
    if (d) {
      const auto &p = d->p_intake;
      const double yaw_error = contact_bearing_error(p);
      sway = yaw_first_sway(0.25 * cfg_.ky_near * p.y, std::abs(yaw_error), 0.10, 0.24, 0.06, 0.02);
      yaw = stabilized_yaw(yaw_error, 0.18 * cfg_.kyaw_near, cfg_.max_yaw_near);
      error = p;
    }
    return make(cfg_.lift_forward_hold, sway, -std::abs(cfg_.lift_heave_up), yaw, MISSION_LIFT_DETACH, error);
  }

  Command contact_push(const std::optional<Detection> &d) const {
    if (!d) return make(0.35, 0, 0, 0, MISSION_CAPTURE_CHECK, {});
    const auto &p = d->p_intake;
    const double yaw_error = contact_bearing_error(p);
    const double yaw_abs = std::abs(yaw_error);
    const double lateral_abs = std::abs(p.y);
    double forward = 0.74;
    if (p.x < -0.18) forward = -0.24;
    else if (p.x < -0.04) forward = -0.10;
    else if (p.x < 0.08 && (lateral_abs > cfg_.capture_y || std::abs(p.z) > cfg_.capture_z)) forward = 0.12;
    else if (p.x > 0.45) forward = 0.82;
    if (yaw_abs > 0.55 || lateral_abs > 0.32) forward = std::min(forward, 0.20);
    else if (yaw_abs > 0.34 || lateral_abs > 0.22) forward = std::min(forward, 0.38);
    const double sway = yaw_first_sway(0.32 * cfg_.ky_near * p.y, yaw_abs, 0.08, 0.18, 0.04, 0.015);
    const double yaw = stabilized_yaw(yaw_error, 0.45 * cfg_.kyaw_near, cfg_.max_yaw_near);
    return make(forward, sway,
                heave_align(p.z, 0.80 * cfg_.kz_near, cfg_.max_vz_near, cfg_.capture_z),
                yaw, MISSION_CAPTURE_CHECK, p);
  }

  bool ready_for_near(const Detection &d) const {
    return d.p_intake.x > -0.10 && d.p_intake.x < 1.8 && std::abs(d.p_intake.y) < 0.90 &&
           std::abs(d.p_intake.z) < 0.55 && std::abs(contact_bearing_error(d.p_intake)) < 0.80;
  }

  bool aligned_for_commit(const Detection &d) const {
    const auto &p = d.p_intake;
    return 0.16 <= p.x && p.x < 0.95 && std::abs(p.y) < 0.16 && std::abs(p.z) < 0.13 &&
           std::abs(contact_bearing_error(p)) < 0.35;
  }

  bool inside_capture(const Detection &d) const {
    return d.p_intake.x > -0.08 && d.p_intake.x < cfg_.capture_x + 0.08 && std::abs(d.p_intake.y) < 0.18 &&
           std::abs(d.p_intake.z) < 0.15;
  }

  bool capture_confirmed(const Detection &d, double dt) {
    if (inside_capture(d) && d.relative_speed < cfg_.capture_speed_max) capture_timer_ += dt;
    else capture_timer_ = 0.0;
    return capture_timer_ >= cfg_.capture_hold;
  }

  void handle_target_lost(double now) {
    if (!target_lost_since_) {
      target_lost_since_ = now;
      return;
    }
    if (now - *target_lost_since_ >= cfg_.target_lost_timeout) {
      if (target_id_ && target_state() == BUOY_TARGETED) buoy(*target_id_).state = BUOY_ATTACHED;
      target_id_.reset();
      transition(MISSION_SEARCH, now);
    }
  }

  bool lift_detach_ready(const Pose &pose, double now) const {
    const bool timer = lift_start_ && now - *lift_start_ >= cfg_.lift_timeout;
    const bool dz = lift_start_z_ && pose.xyz.z - *lift_start_z_ >= cfg_.lift_detach_dz;
    return timer || dz;
  }

  bool target_physical_detached(const Pose &pose, double now) const {
    (void)pose;
    if (!target_id_) return false;
    const auto &b = buoy(*target_id_);
    auto live = live_for(b, now);
    if (!live) return false;
    return live->detached || (live->eq_active && !*live->eq_active);
  }

  bool lift_retry_needed(double now) const {
    if (!target_id_ || !lift_start_) return false;
    const auto &b = buoy(*target_id_);
    auto live = live_for(b, now);
    return live && !live->detached && !(live->eq_active && !*live->eq_active) && now - *lift_start_ >= cfg_.lift_timeout;
  }

  void detach_target(double now) {
    if (!target_id_) return;
    mark_detached(buoy(*target_id_), now);
  }

  void mark_detached(MissionBuoy &b, double now) {
    b.state = BUOY_RISING;
    b.processed = true;
    b.failed = false;
    b.detach_time = now;
    remove_from(failed_, b.target.name);
    push_unique(processed_, b.target.name);
  }

  void mark_collected(size_t idx) {
    auto &b = buoys_[idx];
    b.state = BUOY_COLLECTED;
    b.failed = false;
    b.processed = true;
    remove_from(failed_, b.target.name);
    push_unique(processed_, b.target.name);
    push_unique(collected_, b.target.name);
  }

  void mark_surface_candidates_collected(double now) {
    const auto candidates = surface_candidates(now);
    for (const auto idx : candidates) mark_collected(idx);
  }

  int mark_surface_contacts(const Pose &pose, double now) {
    (void)pose;
    int count = 0;
    const auto candidates = surface_candidates(now);
    for (const auto idx : candidates) {
      if (!live_collector_netted(buoys_[idx], now)) continue;
      mark_collected(idx);
      ++count;
    }
    return count;
  }

  void mark_scored() {
    if (!all_required_collected()) return;
    for (auto &b : buoys_) {
      b.state = BUOY_SCORED;
      push_unique(scored_, b.target.name);
    }
  }

  bool all_required_collected() const {
    if (buoys_.empty() || collected_.size() != buoys_.size()) return false;
    return std::all_of(buoys_.begin(), buoys_.end(), [&](const MissionBuoy &b) {
      return contains(collected_, b.target.name);
    });
  }

  bool all_required_scored() const {
    if (buoys_.empty() || scored_.size() != buoys_.size()) return false;
    return std::all_of(buoys_.begin(), buoys_.end(), [&](const MissionBuoy &b) {
      return contains(scored_, b.target.name) && b.state == BUOY_SCORED;
    });
  }

  void mark_failed() {
    if (!target_id_) return;
    auto &b = buoy(*target_id_);
    b.state = BUOY_FAILED;
    b.failed = true;
    push_unique(failed_, *target_id_);
    target_id_.reset();
  }

  void retry_current_target(double now) {
    if (!target_id_) return;
    auto &b = buoy(*target_id_);
    if (b.state == BUOY_CAPTURED) {
      target_lost_since_.reset();
      capture_timer_ = 0.0;
      if (!lift_start_) lift_start_ = now;
      transition(MISSION_LIFT_DETACH, now);
      return;
    }
    if (!b.processed) b.state = BUOY_TARGETED;
    target_lost_since_.reset();
    alignment_timer_ = 0.0;
    capture_timer_ = 0.0;
    commit_start_.reset();
    lift_start_.reset();
    lift_start_z_.reset();
    transition(MISSION_APPROACH_FAR, now);
  }

  void set_target_state(const std::string &s) {
    if (target_id_) buoy(*target_id_).state = s;
  }

  std::string target_state() const {
    if (!target_id_) return "";
    return buoy(*target_id_).state;
  }

  bool target_state_is(std::initializer_list<std::string> states) const {
    const auto s = target_state();
    return std::find(states.begin(), states.end(), s) != states.end();
  }

  int remaining_attached() const {
    int count = 0;
    for (const auto &b : buoys_) {
      if ((b.state == BUOY_ATTACHED || b.state == BUOY_TARGETED || b.state == BUOY_CAPTURED) && !b.failed && !b.processed) {
        ++count;
      }
    }
    return count;
  }

  std::vector<size_t> surface_candidates(double now) const {
    std::vector<size_t> indices;
    for (size_t i = 0; i < buoys_.size(); ++i) {
      const auto &b = buoys_[i];
      if (!b.failed && (b.state == BUOY_RISING || b.state == BUOY_FLOATING) &&
          !contains(collected_, b.target.name) && !contains(scored_, b.target.name)) {
        indices.push_back(i);
      }
    }
    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
      return target_xyz(buoys_[a], now).x < target_xyz(buoys_[b], now).x;
    });
    return indices;
  }

  std::optional<size_t> select_surface_buoy(const Pose &pose, double now) const {
    const auto c = surface_candidates(now);
    if (c.empty()) return std::nullopt;
    if (target_id_) {
      const auto current = index_by_name_.find(*target_id_);
      if (current != index_by_name_.end() && std::find(c.begin(), c.end(), current->second) != c.end()) {
        return current->second;
      }
    }
    return *std::min_element(c.begin(), c.end(), [&](size_t a, size_t b) {
      return norm(p_surface_collector(pose, buoys_[a], now)) < norm(p_surface_collector(pose, buoys_[b], now));
    });
  }

  bool inside_surface_collect(const Pose &pose, const MissionBuoy &b, double now) const {
    const Vec3 p = p_surface_collector(pose, b, now);
    return std::abs(p.x) <= effective_surface_collect_x_window() &&
           std::abs(p.y) <= effective_surface_collect_y_window() &&
           std::abs(p.z) <= effective_surface_collect_z_window();
  }

  bool live_collector_netted(const MissionBuoy &b, double now) const {
    const auto live = live_for(b, now);
    return live && live->collector_net_enabled && live->netted && !live->net_score_released;
  }

  bool live_score_released(const MissionBuoy &b, double now) const {
    const auto live = live_for(b, now);
    return live && live->collector_net_enabled && live->net_score_released;
  }

  double effective_surface_collect_x_window() const {
    return std::max(cfg_.surface_collect_x_window, cfg_.surface_collect_min_x_window);
  }

  double effective_surface_collect_y_window() const {
    return std::max(cfg_.surface_collect_y_window, cfg_.surface_collect_min_y_window);
  }

  double effective_surface_collect_z_window() const {
    return std::max(cfg_.surface_collect_z_window, cfg_.surface_collect_min_z_window);
  }

  bool surface_collect_timed_out(double now) {
    if (!surface_collect_start_) {
      surface_collect_start_ = now;
      return false;
    }
    return now - *surface_collect_start_ >= cfg_.surface_collect_timeout;
  }

  std::optional<YoloGuidance> fresh_yolo(double now) const {
    if (!cfg_.surface_collect_yolo || !yolo_ || !yolo_->valid) return std::nullopt;
    if (now - yolo_->received_s > cfg_.yolo_surface_timeout) return std::nullopt;
    if (!yolo_->active || !yolo_->model_found) return std::nullopt;
    return yolo_;
  }

  bool yolo_surface_capture_ready(const YoloGuidance &y) const {
    if (!y.has_target) return false;
    const double ex = std::isfinite(y.error_x_norm) ? y.error_x_norm : 0.0;
    const bool centered = std::abs(ex) <= cfg_.yolo_surface_center_tolerance;
    const bool close = y.height_ratio >= cfg_.yolo_surface_close_height_ratio ||
                       y.area_ratio >= cfg_.yolo_surface_close_area_ratio;
    return centered && (close || y.capture);
  }

  Command yolo_surface_collect(const YoloGuidance &y) const {
    if (!y.has_target) {
      return make(0.0, 0.0, 0.0, cfg_.yolo_surface_search_yaw, MISSION_SURFACE_COLLECT, {});
    }
    const double ex = std::isfinite(y.error_x_norm) ? y.error_x_norm : 0.0;
    const double ey = std::isfinite(y.error_y_norm) ? y.error_y_norm : 0.0;
    const double size = std::max(y.height_ratio, 1.8 * y.area_ratio);
    double forward = clamp(cfg_.max_surface_forward * (1.0 - 1.65 * size), 0.18, cfg_.max_surface_forward);
    if (std::abs(ex) > 0.42) forward = std::min(forward, 0.18);
    else if (std::abs(ex) > 0.24) forward = std::min(forward, 0.42);
    if (y.capture) forward = std::max(forward, 0.26);

    // MuJoCo camera x axis points toward body -Y, so pixel-right needs negative yaw/sway.
    const double yaw = clamp(-cfg_.yolo_surface_yaw_gain * ex, -cfg_.max_surface_yaw, cfg_.max_surface_yaw);
    const double sway = clamp(-cfg_.yolo_surface_sway_gain * ex, -cfg_.max_surface_sway, cfg_.max_surface_sway);
    const double heave = clamp(-0.12 * ey, -0.12, 0.12);
    return make(forward, sway, heave, yaw, MISSION_SURFACE_COLLECT, {std::max(0.0, 1.0 - size), ex, ey});
  }

  Command merge_surface_yolo_guidance(Command base, const YoloGuidance &y) const {
    if (!y.has_target) return base;
    const Command visual = yolo_surface_collect(y);
    const double ex = std::isfinite(y.error_x_norm) ? y.error_x_norm : 0.0;
    const double ax = std::abs(ex);

    base.yaw = clamp(0.20 * base.yaw + 0.80 * visual.yaw, -cfg_.max_surface_yaw, cfg_.max_surface_yaw);
    base.sway = clamp(0.35 * base.sway + 0.65 * visual.sway, -cfg_.max_surface_sway, cfg_.max_surface_sway);
    base.heave = clamp(base.heave + 0.50 * visual.heave, -cfg_.max_surface_heave, cfg_.max_surface_heave);

    if (ax > 0.45) base.forward = std::min(base.forward, 0.35);
    else if (ax > 0.25) base.forward = std::min(base.forward, 0.55);
    else base.forward = std::max(base.forward, std::min(visual.forward, cfg_.max_surface_forward));
    if (y.capture || yolo_surface_capture_ready(y)) base.forward = std::max(base.forward, 0.34);
    return base;
  }

  void begin_score_transit(const Pose &pose, double now) {
    if (cfg_.own_course == "a") {
      score_zone_id_ = "course_a_score_zone";
      score_zone_ = cfg_.score_zone_a;
    } else if (cfg_.own_course == "b") {
      score_zone_id_ = "course_b_score_zone";
      score_zone_ = cfg_.score_zone_b;
    } else if (pose.xyz.x < 0.0) {
      score_zone_id_ = "course_a_score_zone";
      score_zone_ = cfg_.score_zone_a;
    } else {
      score_zone_id_ = "course_b_score_zone";
      score_zone_ = cfg_.score_zone_b;
    }
    score_confirm_start_.reset();
    transition(MISSION_SCORE_TRANSIT, now);
  }

  Command surface_collect(const Pose &pose, const MissionBuoy &b, double now) const {
    const Vec3 p = p_surface_collector(pose, b, now);
    const double yaw_error = wrap_pi(std::atan2(p.y, p.x));
    const double yaw_abs = std::abs(yaw_error);
    const double forward = yaw_first_surface_forward(
        clamp(cfg_.k_surface_forward * (p.x - 0.08), -cfg_.max_surface_reverse, cfg_.max_surface_forward), yaw_abs);
    const double sway = yaw_first_sway(cfg_.k_surface_sway * p.y, yaw_abs, 0.16, 0.34, 0.12, 0.035);
    return make(forward, sway,
                heave_align(p.z, cfg_.k_surface_heave, cfg_.max_surface_heave, 0.18),
                stabilized_yaw(yaw_error, cfg_.k_surface_yaw, cfg_.max_surface_yaw),
                MISSION_SURFACE_COLLECT, p);
  }

  Command surface_waypoint_for_collector(const Pose &pose, const std::optional<Vec3> &target,
                                         const std::string &phase) const {
    if (!target) return zero_const(phase);
    const Vec3 collector = surface_collector_world(pose);
    const Vec3 e = rotate_world_to_body(sub(*target, collector), pose.yaw);
    const double yaw_error = wrap_pi(std::atan2(e.y, e.x));
    const double yaw_abs = std::abs(yaw_error);
    const double forward =
        yaw_first_surface_forward(clamp(cfg_.k_surface_forward * e.x, -cfg_.max_surface_reverse, cfg_.max_surface_forward),
                                  yaw_abs);
    const double sway = yaw_first_sway(cfg_.k_surface_sway * e.y, yaw_abs, 0.16, 0.34, 0.12, 0.035);
    return make(forward, sway,
                heave_align(e.z, cfg_.k_surface_heave, cfg_.max_surface_heave, 0.20),
                stabilized_yaw(yaw_error, cfg_.k_surface_yaw, cfg_.max_surface_yaw), phase, e);
  }

  Command surface_waypoint(const Pose &pose, const std::optional<Vec3> &target, const std::string &phase) const {
    if (!target) return zero_const(phase);
    const Vec3 e = rotate_world_to_body(sub(*target, pose.xyz), pose.yaw);
    const double yaw_error = wrap_pi(std::atan2(e.y, e.x));
    const double yaw_abs = std::abs(yaw_error);
    const double forward =
        yaw_first_surface_forward(clamp(cfg_.k_surface_forward * e.x, -cfg_.max_surface_reverse, cfg_.max_surface_forward),
                                  yaw_abs);
    const double sway = yaw_first_sway(cfg_.k_surface_sway * e.y, yaw_abs, 0.16, 0.34, 0.12, 0.035);
    return make(forward, sway,
                heave_align(e.z, cfg_.k_surface_heave, cfg_.max_surface_heave, 0.20),
                stabilized_yaw(yaw_error, cfg_.k_surface_yaw, cfg_.max_surface_yaw), phase, e);
  }

  bool inside_score_zone(const Pose &pose) const {
    if (!score_zone_) return false;
    const Vec3 d = sub(surface_collector_world(pose), *score_zone_);
    return std::hypot(d.x, d.y) <= cfg_.score_zone_radius && std::abs(d.z) <= cfg_.score_zone_z_window;
  }

  bool buoy_inside_score_zone(const MissionBuoy &b, double now) const {
    if (!score_zone_) return false;
    const auto live = live_for(b, now);
    if (live && live->collector_net_enabled && !live_score_released(b, now)) return false;
    const Vec3 d = sub(target_xyz(b, now), *score_zone_);
    return std::hypot(d.x, d.y) <= cfg_.score_zone_radius + cfg_.score_buoy_tolerance &&
           std::abs(d.z) <= cfg_.score_zone_z_window + cfg_.score_buoy_tolerance;
  }

  size_t released_buoys_inside_score_zone_count(double now) const {
    return static_cast<size_t>(std::count_if(buoys_.begin(), buoys_.end(), [&](const MissionBuoy &b) {
      return contains(collected_, b.target.name) && live_score_released(b, now) && buoy_inside_score_zone(b, now);
    }));
  }

  bool collected_buoys_inside_score_zone(double now) const {
    if (!all_required_collected()) return false;
    return std::all_of(buoys_.begin(), buoys_.end(), [&](const MissionBuoy &b) {
      return live_score_released(b, now) && buoy_inside_score_zone(b, now);
    });
  }

  Command score_dump(const Pose &pose) const {
    Command cmd = surface_waypoint_for_collector(pose, score_zone_, MISSION_SCORE_CONFIRM);
    cmd.forward = 0.0;
    cmd.sway = 0.0;
    cmd.yaw = 0.0;
    cmd.pitch = cfg_.score_dump_pitch;
    cmd.phase = MISSION_SCORE_CONFIRM;
    return cmd;
  }

  double elapsed(double now) const { return mission_start_ ? now - *mission_start_ : 0.0; }

  Command boundary_guard(const Pose &pose, const Command &cmd) const {
    const std::string course = lower(cfg_.own_course);
    if (course != "a" && course != "b") return cmd;
    if (target_id_ && buoy(*target_id_).target.course == "pinger") return cmd;
    if (std::abs(cmd.forward) < 1.0e-6 && std::abs(cmd.sway) < 1.0e-6) return cmd;
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    double f = cmd.forward;
    double sw = cmd.sway;
    const double world_x_cmd = c * f - s * sw;
    const double opponent_sign = course == "a" ? 1.0 : -1.0;
    const double stop_x = cfg_.course_boundary_x - opponent_sign * std::max(0.0, cfg_.course_boundary_standoff);
    const double dist = opponent_sign * (stop_x - pose.xyz.x);
    if (dist >= cfg_.course_boundary_margin) return cmd;
    double max_opp = 0.0;
    if (dist > 0.0) max_opp = 0.60 * dist / std::max(cfg_.course_boundary_margin, 1.0e-6);
    else max_opp = -std::min(0.50, 0.16 + 0.80 * (-dist));
    const double opp_cmd = opponent_sign * world_x_cmd;
    if (opp_cmd <= max_opp) return cmd;
    const double desired_world_x = opponent_sign * max_opp;
    const double delta = desired_world_x - world_x_cmd;
    if (std::abs(c) >= 0.35) f += delta / c;
    else if (std::abs(s) >= 0.35) sw -= delta / s;
    else {
      f += delta * c;
      sw -= delta * s;
    }
    Command out = cmd;
    out.forward = clamp(f, -1, 1);
    out.sway = clamp(sw, -1, 1);
    return out;
  }

  Command phase_command_guard(const Command &cmd) const {
    Command out = cmd;
    if (out.phase == MISSION_APPROACH_FAR) {
      const double yaw_abs = std::abs(target_bearing_error(out.error));
      if (yaw_abs > 0.06 || std::abs(out.error.y) > 0.30) out.sway = 0.0;
      else out.sway = clamp(out.sway, -0.04, 0.04);
      out.yaw = clamp(out.yaw, -cfg_.max_yaw_far, cfg_.max_yaw_far);
    } else if (out.phase == MISSION_APPROACH_NEAR) {
      const double yaw_abs = std::abs(contact_bearing_error(out.error));
      const double lateral_abs = std::abs(out.error.y);
      if (lateral_abs > 1.20 || yaw_abs > 0.90) out.forward = out.error.x < 0.55 ? std::min(out.forward, -0.08) : std::min(out.forward, 0.24);
      else if (lateral_abs > 0.85 || yaw_abs > 0.60) out.forward = std::min(out.forward, 0.34);
      else if (lateral_abs > 0.58 || yaw_abs > 0.38) out.forward = std::min(out.forward, 0.48);
      else if (lateral_abs > 0.35 || yaw_abs > 0.22) out.forward = std::min(out.forward, 0.62);
      if (yaw_abs > 0.28 || lateral_abs > 0.60) out.sway = 0.0;
      else out.sway = clamp(out.sway, -0.06, 0.06);
      out.yaw = clamp(out.yaw, -cfg_.max_yaw_near, cfg_.max_yaw_near);
    } else if (out.phase == MISSION_COMMIT) {
      const double yaw_abs = std::abs(contact_bearing_error(out.error));
      if (yaw_abs > 0.85) out.forward = std::min(out.forward, 0.18);
      else if (yaw_abs > 0.58) out.forward = std::min(out.forward, 0.34);
      else if (yaw_abs > 0.36) out.forward = std::min(out.forward, 0.56);
      if (yaw_abs > 0.24 || std::abs(out.error.y) > 0.42) out.sway = 0.0;
      else out.sway = clamp(out.sway, -0.04, 0.04);
      out.yaw = clamp(out.yaw, -cfg_.max_yaw_near, cfg_.max_yaw_near);
    } else if (out.phase == MISSION_SURFACE_COLLECT || out.phase == MISSION_SCORE_TRANSIT ||
               out.phase == MISSION_SCORE_CONFIRM) {
      const double yaw_abs = std::abs(wrap_pi(std::atan2(out.error.y, std::max(std::abs(out.error.x), 0.40))));
      out.forward = yaw_first_surface_forward(out.forward, yaw_abs);
      if (out.phase == MISSION_SURFACE_COLLECT && out.forward > 0.0) {
        if (out.distance < 0.45) out.forward = std::min(out.forward, 0.18);
        else if (out.distance < 0.80) out.forward = std::min(out.forward, 0.28);
        else if (out.distance < 1.50) out.forward = std::min(out.forward, 0.50);
        if (!collected_.empty()) out.forward = std::min(out.forward, cfg_.max_surface_loaded_forward);
      }
      if (out.phase == MISSION_SCORE_TRANSIT && !collected_.empty() && out.forward > 0.0) {
        out.forward = std::min(out.forward, cfg_.max_surface_loaded_forward);
      }
      if (yaw_abs > 0.75) out.sway = 0.0;
      else if (yaw_abs > 0.42) out.sway = clamp(out.sway, -0.12, 0.12);
      out.heave = clamp(out.heave, -cfg_.max_surface_collect_heave, cfg_.max_surface_collect_heave);
      if ((out.phase == MISSION_SURFACE_COLLECT || out.phase == MISSION_SCORE_TRANSIT) && !collected_.empty()) {
        out.sway = clamp(out.sway, -cfg_.max_surface_loaded_sway, cfg_.max_surface_loaded_sway);
        out.heave = clamp(out.heave, -cfg_.max_surface_loaded_heave, cfg_.max_surface_loaded_heave);
        out.yaw = clamp(out.yaw, -cfg_.max_surface_loaded_yaw, cfg_.max_surface_loaded_yaw);
      } else {
        out.yaw = clamp(out.yaw, -cfg_.max_surface_yaw, cfg_.max_surface_yaw);
      }
    }
    return out;
  }

  Step step(const Command &cmd, const std::optional<Detection> &d, bool capture) const {
    Step s;
    s.command = cmd;
    s.state = state_;
    s.mode = mode_;
    s.target_id = target_id_;
    if (target_id_ && index_by_name_.count(*target_id_)) {
      const auto &b = buoy(*target_id_);
      s.target_class = b.target.color;
      s.target_state = b.state;
    }
    s.detection = d;
    s.capture_flag = capture;
    s.remaining_attached = remaining_attached();
    s.processed_count = static_cast<int>(processed_.size());
    s.failed_count = static_cast<int>(failed_.size());
    return s;
  }

  static bool contains(const std::vector<std::string> &v, const std::string &s) {
    return std::find(v.begin(), v.end(), s) != v.end();
  }
  static void push_unique(std::vector<std::string> &v, const std::string &s) {
    if (!contains(v, s)) v.push_back(s);
  }
  static void remove_from(std::vector<std::string> &v, const std::string &s) {
    v.erase(std::remove(v.begin(), v.end(), s), v.end());
  }
  static std::string string_array(const std::vector<std::string> &v) {
    std::ostringstream out;
    out << "[";
    for (size_t i = 0; i < v.size(); ++i) {
      if (i) out << ",";
      out << q(v[i]);
    }
    out << "]";
    return out.str();
  }

  std::string candidate_ids_json(double now) const {
    std::vector<std::string> ids;
    for (auto idx : surface_candidates(now)) ids.push_back(buoys_[idx].target.name);
    return string_array(ids);
  }

  static void detection_json(std::ostringstream &out, const Detection &d) {
    out << "{\"buoy_id\":" << q(d.buoy_id) << ",\"class_name\":" << q(d.class_name)
        << ",\"p_intake\":" << vec_json(d.p_intake) << ",\"distance_m\":" << d.distance
        << ",\"confidence\":" << d.confidence << ",\"p_rate\":" << vec_json(d.p_rate)
        << ",\"relative_speed_mps\":" << d.relative_speed
        << ",\"target_xyz\":" << vec_json(d.target_xyz) << ",\"coordinate_source\":" << q(d.coordinate_source)
        << ",\"bearing_rad\":" << wrap_pi(std::atan2(d.p_intake.y, d.p_intake.x)) << "}";
  }

  static std::string robot_state_label(const std::string &state) {
    if (state == "WAIT_POSE") return "Wait pose";
    if (state == "WAIT_ARM") return "Wait arm";
    if (state == "BUOY_SEARCHING") return "Buoy searching";
    if (state == "BUOY_DETECTED") return "Buoy detected";
    if (state == "BUOY_APPROACHING") return "Approaching";
    if (state == "BUOY_DETACHING") return "Detaching";
    if (state == "SURFACE_ASCENT") return "Surface ascent";
    if (state == "SURFACE_COLLECTION") return "Surface collection";
    if (state == "SCORE_TRANSIT") return "Score transit";
    if (state == "SIM_STALE") return "Sim stale";
    if (state == "MISSION_COMPLETE") return "Complete";
    return state;
  }

  static std::string robot_state_for_status(const std::string &state, const std::optional<Step> &step_opt,
                                            bool waiting_for_pose, bool waiting_for_arm) {
    if (waiting_for_pose) return "WAIT_POSE";
    if (waiting_for_arm) return "WAIT_ARM";
    const bool detected = step_opt && step_opt->detection.has_value();
    if (state == MISSION_INIT || state == MISSION_DIVE || state == MISSION_SEARCH || state == MISSION_NEXT_TARGET) {
      return detected ? "BUOY_DETECTED" : "BUOY_SEARCHING";
    }
    if (state == MISSION_SELECT) return "BUOY_DETECTED";
    if (state == MISSION_APPROACH_FAR || state == MISSION_APPROACH_NEAR || state == MISSION_COMMIT ||
        state == MISSION_CAPTURE_CHECK) {
      return "BUOY_APPROACHING";
    }
    if (state == MISSION_LIFT_DETACH || state == MISSION_DETACH_CONFIRM) return "BUOY_DETACHING";
    if (state == MISSION_ASCEND || state == MISSION_SURFACE_READY) return "SURFACE_ASCENT";
    if (state == MISSION_SURFACE_COLLECT) return "SURFACE_COLLECTION";
    if (state == MISSION_SCORE_TRANSIT || state == MISSION_SCORE_CONFIRM) return "SCORE_TRANSIT";
    if (state == MISSION_SIM_STALE) return "SIM_STALE";
    if (state == MISSION_COMPLETE) return "MISSION_COMPLETE";
    return state;
  }

  void buoy_json(std::ostringstream &out, const MissionBuoy &b, double now) const {
    const auto live = live_for(b, now);
    const Vec3 xyz = target_xyz(b, now);
    out << "{\"id\":" << q(b.target.name) << ",\"class_name\":" << q(b.target.color)
        << ",\"course\":" << q(b.target.course)
        << ",\"state\":" << q(b.state) << ",\"processed\":" << (b.processed ? "true" : "false")
        << ",\"failed\":" << (b.failed ? "true" : "false") << ",\"xyz\":" << vec_json(xyz)
        << ",\"target_xyz\":" << vec_json(xyz) << ",\"body_xyz\":";
    if (live && live->body_xyz) out << vec_json(*live->body_xyz);
    else out << "null";
    out << ",\"attach_xyz\":";
    if (live && live->attach_xyz) out << vec_json(*live->attach_xyz);
    else out << "null";
    out << ",\"magnet_xyz\":";
    if (live && live->magnet_xyz) out << vec_json(*live->magnet_xyz);
    else out << "null";
    out << ",\"coordinate_source\":" << q(live ? live->source : "scene_static")
        << ",\"target_kind\":" << q(live ? live->target_kind : "float_center_static")
        << ",\"live_age_s\":";
    if (live) out << std::max(0.0, now - live->received_s);
    else out << "null";
    out << ",\"physical_detached\":" << ((live && live->detached) ? "true" : "false") << ",\"eq_active\":";
    if (live && live->eq_active) out << (*live->eq_active ? "true" : "false");
    else out << "null";
    out << ",\"probe_release_margin_m\":";
    if (live && live->probe_release_margin_m) out << *live->probe_release_margin_m;
    else out << "null";
    out << ",\"probe_release_proximity\":" << ((live && live->probe_release_proximity) ? "true" : "false");
    out << ",\"collector_net_enabled\":" << ((live && live->collector_net_enabled) ? "true" : "false")
        << ",\"netted\":" << ((live && live->netted) ? "true" : "false") << ",\"netted_time_s\":";
    if (live && live->netted_time_s) out << *live->netted_time_s;
    else out << "null";
    out << ",\"net_score_released\":" << ((live && live->net_score_released) ? "true" : "false")
        << ",\"net_score_release_time_s\":";
    if (live && live->net_score_release_time_s) out << *live->net_score_release_time_s;
    else out << "null";
    out << ",\"release_force_threshold_n\":" << b.release_force_threshold << "}";
  }

  std::vector<MissionBuoy> buoys_;
  std::unordered_map<std::string, size_t> index_by_name_;
  MissionConfig cfg_;
  std::string state_{MISSION_INIT};
  std::string mode_{"UNDERWATER"};
  std::optional<std::string> target_id_;
  std::vector<std::string> processed_;
  std::vector<std::string> failed_;
  std::vector<std::string> collected_;
  std::vector<std::string> scored_;
  std::optional<double> mission_start_;
  std::optional<double> state_entered_;
  std::optional<double> last_update_;
  std::optional<double> target_lost_since_;
  double alignment_timer_{0.0};
  double capture_timer_{0.0};
  std::optional<double> commit_start_;
  std::optional<double> lift_start_;
  std::optional<double> lift_start_z_;
  std::optional<double> surface_collect_start_;
  std::optional<double> score_confirm_start_;
  std::optional<std::string> score_zone_id_;
  std::optional<Vec3> score_zone_;
  std::optional<std::string> approach_progress_target_;
  std::optional<std::string> approach_progress_state_;
  std::optional<double> approach_progress_since_;
  double approach_best_distance_{std::numeric_limits<double>::infinity()};
  std::unordered_map<std::string, LiveState> live_;
  std::optional<YoloGuidance> yolo_;
  std::optional<double> surface_yolo_capture_start_;
  std::unordered_map<std::string, std::pair<Vec3, double>> last_detection_;
  std::optional<double> last_pose_yaw_;
  std::optional<double> last_pose_time_;
  double yaw_rate_{0.0};
  std::optional<double> latest_live_received_s_;
};

int axis_pwm(double value, bool invert, double span) {
  const double command = invert ? -value : value;
  return static_cast<int>(std::llround(RC_NEUTRAL + span * clamp(command, -1.0, 1.0)));
}

std::array<uint16_t, RC_CHANNEL_COUNT> rc_channels(
    const Command &cmd, bool invert_heave, bool invert_yaw, double span) {
  std::array<uint16_t, RC_CHANNEL_COUNT> ch{};
  ch.fill(RC_NOCHANGE);
  for (size_t i = 0; i < PRIMARY_RC_CHANNEL_COUNT; ++i) ch[i] = RC_NEUTRAL;
  ch[CH_PITCH] = static_cast<uint16_t>(axis_pwm(cmd.pitch, false, span));
  ch[CH_HEAVE] = static_cast<uint16_t>(axis_pwm(cmd.heave, invert_heave, span));
  ch[CH_YAW] = static_cast<uint16_t>(axis_pwm(cmd.yaw, invert_yaw, span));
  ch[CH_FORWARD] = static_cast<uint16_t>(axis_pwm(cmd.forward, false, span));
  ch[CH_SWAY] = static_cast<uint16_t>(axis_pwm(cmd.sway, false, span));
  return ch;
}

std::array<uint16_t, RC_CHANNEL_COUNT> rc_release() {
  std::array<uint16_t, RC_CHANNEL_COUNT> ch{};
  ch.fill(RC_NOCHANGE);
  for (size_t i = 0; i < PRIMARY_RC_CHANNEL_COUNT; ++i) ch[i] = RC_RELEASE;
  return ch;
}

double now_seconds() {
  using clock = std::chrono::steady_clock;
  static const auto start = clock::now();
  return std::chrono::duration<double>(clock::now() - start).count();
}

std::string dirname(const std::string &path) {
  const auto pos = path.find_last_of('/');
  return pos == std::string::npos ? "." : path.substr(0, pos);
}

void ensure_parent(const std::string &path) {
  if (path.empty()) return;
  const std::string dir = dirname(path);
  if (dir == "." || dir.empty()) return;
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
}

std::string auto_log_path() {
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream out;
  out << "uuv_mujoco/current/logs/ground_truth_buoy_mission_cpp_"
      << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
  return out.str();
}

Options parse_args(int argc, char **argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
      return argv[++i];
    };
    if (a == "--scene") opt.scene = next();
    else if (a == "--course") opt.course = lower(next());
    else if (a == "--own-course") opt.own_course = lower(next());
    else if (a == "--colors") opt.colors = next();
    else if (a == "--no-pinger") opt.no_pinger = true;
    else if (a == "--nearest-first") opt.nearest_first = true;
    else if (a == "--controller") (void)next();
    else if (a == "--max-targets") opt.max_targets = std::stoi(next());
    else if (a == "--rate-hz") opt.rate_hz = std::stod(next());
    else if (a == "--transport") opt.transport = next();
    else if (a == "--dry-run") opt.dry_run = true;
    else if (a == "--wait-armed") opt.wait_armed = true;
    else if (a == "--no-wait-armed") opt.wait_armed = false;
    else if (a == "--pose-topic") opt.pose_topic = next();
    else if (a == "--buoy-status-topic") opt.buoy_status_topic = next();
    else if (a == "--yolo-detection-topic") opt.yolo_detection_topic = next();
    else if (a == "--state-topic") opt.state_topic = next();
    else if (a == "--rc-topic") opt.rc_topic = next();
    else if (a == "--manual-topic") opt.manual_topic = next();
    else if (a == "--command-override-topic") opt.command_override_topic = next();
    else if (a == "--mission-log") opt.mission_log = next();
    else if (a == "--status-json") opt.status_json = next();
    else if (a == "--rc-pwm-span") opt.rc_pwm_span = clamp(std::stod(next()), 50.0, 700.0);
    else if (a == "--search-depth-z") opt.cfg.search_depth_z = std::stod(next());
    else if (a == "--surface-ready-z") opt.cfg.surface_ready_z = std::stod(next());
    else if (a == "--course-boundary-x") opt.cfg.course_boundary_x = std::stod(next());
    else if (a == "--course-boundary-margin") opt.cfg.course_boundary_margin = std::stod(next());
    else if (a == "--course-boundary-standoff") opt.cfg.course_boundary_standoff = std::stod(next());
    else if (a == "--mission-time-limit-s") opt.cfg.mission_time_limit = std::stod(next());
    else if (a == "--fake-vision-range-m") opt.cfg.fake_vision_range = std::stod(next());
    else if (a == "--live-buoy-timeout-s") opt.cfg.live_buoy_timeout = std::stod(next());
    else if (a == "--live-status-timeout-s") opt.cfg.live_status_timeout = std::stod(next());
    else if (a == "--require-live-status") opt.cfg.require_live_status = true;
    else if (a == "--allow-static-fallback") opt.cfg.require_live_status = false;
    else if (a == "--capture-x-m") opt.cfg.capture_x = std::stod(next());
    else if (a == "--capture-y-m") opt.cfg.capture_y = std::stod(next());
    else if (a == "--capture-z-m") opt.cfg.capture_z = std::stod(next());
    else if (a == "--commit-timeout-s") opt.cfg.commit_timeout = std::stod(next());
    else if (a == "--lift-timeout-s") opt.cfg.lift_timeout = std::stod(next());
    else if (a == "--target-timeout-s") opt.cfg.target_lost_timeout = std::stod(next());
    else if (a == "--max-forward") {
      const double v = std::stod(next());
      opt.cfg.max_vx_far = std::min(v, opt.cfg.max_vx_far);
    } else if (a == "--max-sway") {
      const double v = std::stod(next());
      opt.cfg.max_vy_far = std::min(v, opt.cfg.max_vy_far);
    } else if (a == "--max-heave") {
      const double v = std::stod(next());
      opt.cfg.max_vz_far = std::min(v, opt.cfg.max_vz_far);
      opt.cfg.max_vz_near = std::min(v, opt.cfg.max_vz_near);
    } else if (a == "--k-forward") {
      opt.cfg.kx_far = std::stod(next());
    } else if (a == "--k-sway") {
      opt.cfg.ky_far = std::stod(next());
    } else if (a == "--k-heave") {
      opt.cfg.kz_far = std::stod(next());
      opt.cfg.kz_near = opt.cfg.kz_far;
    } else if (a == "--k-yaw") {
      opt.cfg.kyaw_far = std::stod(next());
    } else if (a == "--collector-x-m") opt.cfg.intake_offset.x = std::stod(next());
    else if (a == "--collector-y-m") opt.cfg.intake_offset.y = std::stod(next());
    else if (a == "--collector-z-m") opt.cfg.intake_offset.z = std::stod(next());
    else if (a == "--surface-collector-x-m") opt.cfg.surface_collector_offset.x = std::stod(next());
    else if (a == "--surface-collector-y-m") opt.cfg.surface_collector_offset.y = std::stod(next());
    else if (a == "--surface-collector-z-m") opt.cfg.surface_collector_offset.z = std::stod(next());
    else if (a == "--surface-collect-x-window-m") opt.cfg.surface_collect_x_window = std::stod(next());
    else if (a == "--surface-collect-y-window-m") opt.cfg.surface_collect_y_window = std::stod(next());
    else if (a == "--surface-collect-z-window-m") opt.cfg.surface_collect_z_window = std::stod(next());
    else if (a == "--surface-collect-ground-truth") opt.cfg.surface_collect_ground_truth = true;
    else if (a == "--no-surface-collect-ground-truth") opt.cfg.surface_collect_ground_truth = false;
    else if (a == "--surface-collect-yolo") opt.cfg.surface_collect_yolo = true;
    else if (a == "--no-surface-collect-yolo") opt.cfg.surface_collect_yolo = false;
    else if (a == "--yolo-surface-timeout-s") opt.cfg.yolo_surface_timeout = std::stod(next());
    else if (a == "--yolo-surface-center-tolerance") opt.cfg.yolo_surface_center_tolerance = std::stod(next());
    else if (a == "--yolo-surface-capture-hold-s") opt.cfg.yolo_surface_capture_hold = std::stod(next());
    else if (a == "--pinger-hydrophone") opt.cfg.pinger_hydrophone_enable = true;
    else if (a == "--no-pinger-hydrophone") opt.cfg.pinger_hydrophone_enable = false;
    else if (a == "--pinger-yolo-final-range-m") opt.cfg.pinger_yolo_final_range = std::stod(next());
    else if (a == "--pinger-yolo-near-range-m") opt.cfg.pinger_yolo_near_range = std::stod(next());
    else if (a == "--score-buoy-tolerance-m") opt.cfg.score_buoy_tolerance = std::stod(next());
    else if (a == "--score-dump-forward") opt.cfg.score_dump_forward = std::stod(next());
    else if (a == "--score-dump-pitch") opt.cfg.score_dump_pitch = std::stod(next());
    else if (a == "--no-invert-heave-rc") opt.no_invert_heave_rc = true;
    else if (a == "--no-invert-yaw-rc") opt.no_invert_yaw_rc = true;
  }
  opt.cfg.own_course = opt.own_course;
  if (opt.course != "a" && opt.course != "b" && opt.course != "all") opt.course = "all";
  if (opt.own_course != "a" && opt.own_course != "b") opt.own_course = "a";
  opt.cfg.own_course = opt.own_course;
  return opt;
}

#ifndef MISSION_FSM_CORE_ONLY
class MissionNode : public rclcpp::Node {
 public:
  MissionNode(const Options &opt, std::vector<Target> targets, size_t target_count, std::string first_target_name)
      : Node("ground_truth_buoy_controller_cpp"),
        opt_(opt),
        mission_(std::move(targets), opt.cfg, opt.max_targets),
        mission_target_count_(target_count),
        first_target_name_(std::move(first_target_name)) {
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        opt.pose_topic, 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          const auto &p = msg->pose.position;
          const auto &qmsg = msg->pose.orientation;
          pose_ = Pose{{p.x, p.y, p.z}, yaw_from_quat(qmsg.x, qmsg.y, qmsg.z, qmsg.w)};
        });
    buoy_sub_ = create_subscription<std_msgs::msg::String>(
        opt.buoy_status_topic, 10, [this](std_msgs::msg::String::SharedPtr msg) {
          mission_.update_live(parse_live_status(msg->data, now_seconds()));
        });
    yolo_sub_ = create_subscription<std_msgs::msg::String>(
        opt.yolo_detection_topic, rclcpp::QoS(1), [this](std_msgs::msg::String::SharedPtr msg) {
          mission_.update_yolo(parse_yolo_guidance(msg->data, now_seconds()));
        });
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        opt.state_topic, 10, [this](mavros_msgs::msg::State::SharedPtr msg) { armed_ = msg->armed; });
    if (opt.transport == "manual_control") {
      manual_pub_ = create_publisher<mavros_msgs::msg::ManualControl>(opt.manual_topic, 10);
    } else if (opt.transport == "command_override") {
      command_pub_ = create_publisher<std_msgs::msg::String>(opt.command_override_topic, 10);
    } else {
      rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(opt.rc_topic, 10);
    }
    open_csv();
    write_status(std::nullopt, now_seconds(), true, false);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(opt.rate_hz, 0.1)), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "loaded C++ ground-truth buoy FSM build=%s targets=%zu first=%s dry_run=%s own_course=%s yolo=%s",
                MISSION_FSM_BUILD_TAG, mission_target_count_, first_target_name_.c_str(),
                opt.dry_run ? "true" : "false", opt.own_course.c_str(), opt.yolo_detection_topic.c_str());
  }

  void release_rc() {
    if (rc_pub_ && !opt_.dry_run && rclcpp::ok()) {
      mavros_msgs::msg::OverrideRCIn msg;
      msg.channels = rc_release();
      for (int i = 0; i < 3; ++i) rc_pub_->publish(msg);
    }
    if (command_pub_ && !opt_.dry_run && rclcpp::ok()) {
      std_msgs::msg::String msg;
      msg.data = direct_command_payload(Command{});
      for (int i = 0; i < 3; ++i) command_pub_->publish(msg);
    }
    if (csv_.is_open()) csv_.close();
  }

 private:
  void tick() {
    const double now = now_seconds();
    if (!pose_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for %s", opt_.pose_topic.c_str());
      write_status(std::nullopt, now, true, false);
      return;
    }
    if (requires_armed() && !armed_.value_or(false)) {
      publish(Command{});
      write_status(std::nullopt, now, false, true);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for armed=true before C++ FSM control");
      return;
    }
    Step step = mission_.update(*pose_, now);
    publish(step.command);
    write_csv(step, now);
    write_status(step, now, false, false);
    if (now - last_log_s_ > 1.0) {
      last_log_s_ = now;
      Vec3 p{NAN, NAN, NAN};
      if (step.detection) p = step.detection->p_intake;
      else if (step.command.phase == MISSION_SURFACE_COLLECT || step.command.phase == MISSION_SCORE_TRANSIT ||
               step.command.phase == MISSION_SCORE_CONFIRM) {
        p = step.command.error;
      }
      RCLCPP_INFO(get_logger(),
                  "%s cmd_phase=%s target=%s class=%s p_intake=(%.2f,%.2f,%.2f) cmd fwd=%+.2f sway=%+.2f heave=%+.2f yaw=%+.2f pitch=%+.2f capture=%s buoy=%s remaining=%d processed=%d",
                  step.state.c_str(), step.command.phase.c_str(), step.target_id.value_or("-").c_str(),
                  step.target_class.empty() ? "-" : step.target_class.c_str(),
                  p.x, p.y, p.z, step.command.forward, step.command.sway, step.command.heave, step.command.yaw,
                  step.command.pitch, step.capture_flag ? "true" : "false",
                  step.target_state.empty() ? "-" : step.target_state.c_str(), step.remaining_attached,
                  step.processed_count);
    }
  }

  bool requires_armed() const { return opt_.wait_armed && !opt_.dry_run; }

  void publish(const Command &cmd) {
    if (opt_.transport == "manual_control") {
      mavros_msgs::msg::ManualControl msg;
      msg.x = static_cast<float>(clamp(cmd.forward, -1, 1));
      msg.y = static_cast<float>(clamp(cmd.sway, -1, 1));
      msg.z = static_cast<float>(clamp(-cmd.heave, -1, 1));
      msg.r = static_cast<float>(clamp(cmd.yaw, -1, 1));
      msg.buttons = 0;
      if (!opt_.dry_run && manual_pub_) manual_pub_->publish(msg);
    } else if (opt_.transport == "command_override") {
      std_msgs::msg::String msg;
      msg.data = direct_command_payload(cmd);
      if (!opt_.dry_run && command_pub_) command_pub_->publish(msg);
    } else {
      mavros_msgs::msg::OverrideRCIn msg;
      msg.channels = rc_channels(cmd, !opt_.no_invert_heave_rc, !opt_.no_invert_yaw_rc, opt_.rc_pwm_span);
      if (!opt_.dry_run && rc_pub_) rc_pub_->publish(msg);
    }
  }

  static std::string direct_command_payload(const Command &cmd) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "{\"direct_cmd\":{\"forward\":" << clamp(cmd.forward, -1.0, 1.0)
        << ",\"sway\":" << clamp(cmd.sway, -1.0, 1.0)
        << ",\"heave\":" << clamp(cmd.heave, -1.0, 1.0)
        << ",\"yaw\":" << clamp(cmd.yaw, -1.0, 1.0)
        << ",\"pitch\":" << clamp(cmd.pitch, -1.0, 1.0) << "}}";
    return out.str();
  }

  void open_csv() {
    std::string path = opt_.mission_log;
    const auto normalized = lower(trim(path));
    if (normalized == "off" || normalized == "none" || normalized == "false" || normalized == "0") return;
    if (path == "auto") path = auto_log_path();
    ensure_parent(path);
    csv_.open(path);
    if (!csv_) return;
    const auto header = mission_.csv_header();
    for (size_t i = 0; i < header.size(); ++i) {
      if (i) csv_ << ",";
      csv_ << header[i];
    }
    csv_ << "\n";
    RCLCPP_INFO(get_logger(), "mission CSV log: %s", path.c_str());
  }

  void write_csv(const Step &step, double now) {
    if (!csv_ || !pose_) return;
    const auto row = mission_.csv_row(step, *pose_, now);
    for (size_t i = 0; i < row.size(); ++i) {
      if (i) csv_ << ",";
      csv_ << row[i];
    }
    csv_ << "\n";
    csv_.flush();
  }

  void write_status(const std::optional<Step> &step, double now, bool waiting_pose, bool waiting_arm) {
    if (opt_.status_json.empty()) return;
    ensure_parent(opt_.status_json);
    const std::string payload = mission_.status_json(step, pose_, now, waiting_pose, armed_, waiting_arm);
    const std::string tmp = opt_.status_json + ".tmp";
    {
      std::ofstream out(tmp);
      out << payload;
    }
    std::rename(tmp.c_str(), opt_.status_json.c_str());
  }

  Options opt_;
  MissionController mission_;
  std::optional<Pose> pose_;
  std::optional<bool> armed_;
  double last_log_s_{0.0};
  size_t mission_target_count_{0};
  std::string first_target_name_{"-"};
  std::ofstream csv_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr buoy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif

}  // namespace

#ifndef MISSION_FSM_CORE_ONLY
int main(int argc, char **argv) {
  try {
    Options opt = parse_args(argc, argv);
    Vec3 start = load_body_positions(opt.scene)["base_link"];
    auto targets = plan_targets(load_targets(opt), opt, start);
    if (targets.empty()) {
      std::cerr << "no buoy targets loaded after C++ FSM planning\n";
      return 2;
    }
    const size_t target_count = targets.size();
    const std::string first = targets.front().name;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionNode>(opt, std::move(targets), target_count, first);
    rclcpp::spin(node);
    node->release_rc();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &exc) {
    std::cerr << "ground_truth_buoy_fsm error: " << exc.what() << "\n";
    return 1;
  }
}
#endif
