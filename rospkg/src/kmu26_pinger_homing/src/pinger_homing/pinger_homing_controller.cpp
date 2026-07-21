#include <algorithm>
// Canonical deployed controller for the standalone kmu26_pinger_homing package.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "pinger_homing_math.hpp"

namespace {

using kmu26::pinger_homing::SourceEstimate;
using kmu26::pinger_homing::clamp;
using kmu26::pinger_homing::confirm_no_odom_bearing_pair;
using kmu26::pinger_homing::no_odom_probe_scale_from_pwm_override;
using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

constexpr double kPi = 3.14159265358979323846;
constexpr int kRcNeutral = 1500;
constexpr std::size_t kPrimaryRcChannelCount = 8;
constexpr std::size_t kChHeave = 2;
constexpr std::size_t kChYaw = 3;
constexpr std::size_t kChForward = 4;
constexpr std::size_t kChSway = 5;

struct Command {
  double forward{0.0};
  double lateral{0.0};
  double heave{0.0};
  double yaw{0.0};
};

struct RangeSample {
  double wall_s{0.0};
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  double cumulative_change_m{0.0};
  double amplitude_range_m{std::numeric_limits<double>::quiet_NaN()};
};

struct NoOdomPhaseSample {
  double probe_s{0.0};
  Eigen::Vector3d command_body{Eigen::Vector3d::Zero()};
  double delta_range_m{0.0};
};

// A Phase delta is a displacement increment between adjacent demodulation
// windows.  During a straight approach it is the only new acoustic
// observation available without deliberately exciting another body axis.
// Keep its timestamp with the value so an innovation window is based on time,
// rather than an assumed audio publication rate.
struct NoOdomApproachSample {
  SteadyTime stamp{};
  double delta_range_m{0.0};
};

double seconds_since(const SteadyTime &stamp, const SteadyTime &now) {
  if (stamp == SteadyTime{}) return 1.0e9;
  // transition() records a fresh steady-clock timestamp a few microseconds
  // after control_tick() captured `now`.  Treat that same-tick ordering as
  // zero elapsed time.  Returning the stale sentinel here skipped the entire
  // first probe immediately after arming (PROBE -> REPROBE in one callback).
  if (now < stamp) return 0.0;
  return std::chrono::duration<double>(now - stamp).count();
}

double seconds_from_epoch(const SteadyTime &stamp) {
  return std::chrono::duration<double>(stamp.time_since_epoch()).count();
}

double radians(double degrees) {
  return degrees * kPi / 180.0;
}

std::string bool_text(bool value) {
  return value ? "true" : "false";
}

std::string json_number(const std::optional<double> &value, int precision = 4) {
  if (!value || !std::isfinite(*value)) return "null";
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << *value;
  return out.str();
}

std::string json_vector(const std::optional<Eigen::Vector3d> &value, int precision = 4) {
  if (!value || !value->allFinite()) return "null";
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision)
      << "[" << value->x() << "," << value->y() << "," << value->z() << "]";
  return out.str();
}

int axis_pwm(double value, double span, double deadzone_pwm) {
  const double command = clamp(value, -1.0, 1.0);
  if (std::abs(command) <= 1.0e-9) return kRcNeutral;

  // ArduSub discards RC input inside RCx_DZ.  probe_pwm_delta and
  // approach_pwm_delta intentionally describe useful authority *outside* that
  // deadzone so the same launch values have the same meaning on SITL and the
  // physical Pixhawk.  Keep neutral exactly neutral and cap the compensated
  // result at the configured RC span.
  const double effective_delta = std::abs(command) * span;
  const double raw_delta = std::min(span, effective_delta + std::max(0.0, deadzone_pwm));
  return static_cast<int>(std::llround(
      kRcNeutral + std::copysign(raw_delta, command)));
}

}  // namespace

class PingerHomingController : public rclcpp::Node {
 public:
  PingerHomingController()
      : Node("pinger_homing_controller") {
    state_started_ = SteadyClock::now();
    last_range_progress_ = state_started_;
    controller_mode_ = declare_parameter<std::string>("controller_mode", "active_range");
    navigation_mode_ = declare_parameter<std::string>("navigation_mode", "odometry");
    if (navigation_mode_ != "odometry" && navigation_mode_ != "no_odom_phase") {
      throw std::invalid_argument(
          "navigation_mode must be 'odometry' or 'no_odom_phase'");
    }
    acoustic_estimator_mode_ =
        declare_parameter<std::string>("acoustic_estimator_mode", "phase");
    if (navigation_mode_ == "no_odom_phase" && acoustic_estimator_mode_ != "phase") {
      throw std::invalid_argument(
          "navigation_mode=no_odom_phase requires acoustic_estimator_mode=phase");
    }
    controller_profile_ =
        declare_parameter<std::string>("controller_profile", "real");
    const bool sim_fast_profile = controller_profile_ == "sim_fast";
    if (controller_profile_ != "real" && !sim_fast_profile) {
      throw std::invalid_argument("controller_profile must be 'real' or 'sim_fast'");
    }
    legacy_python_sequence_ = declare_parameter<bool>(
        "legacy_python_sequence", acoustic_estimator_mode_ == "phase");
    // Phase must preserve the deployed Python RC state machine.  In
    // particular, its neutral-separated X/Y/Z legs are what make the
    // four-state Phase EKF observable; a simultaneous helical move cannot
    // separate vertical direction from the range-rate bias.
    // The Phase estimator still needs the original neutral-separated probe,
    // but its post-fit RC motion may use the simulator profile.  Conflating
    // those two choices previously made sim_fast silently fall back to the
    // real-vehicle yaw/forward gains for Phase.
    const bool fast_motion_profile = sim_fast_profile;
    const bool fast_probe_profile = sim_fast_profile && !legacy_python_sequence_;
    transport_ = declare_parameter<std::string>("transport", "rc_override");

    const auto legacy_direction_topic =
        declare_parameter<std::string>("direction_topic", "/homing/direction");
    direction_topic_ = declare_parameter<std::string>(
        "direction_input_topic", legacy_direction_topic);
    direction_frame_ = declare_parameter<std::string>(
        "direction_frame", acoustic_estimator_mode_ == "snr" ? "body" : "world");
    if (direction_frame_ != "body" && direction_frame_ != "world") {
      throw std::invalid_argument("direction_frame must be 'body' or 'world'");
    }
    const auto legacy_odom_topic =
        declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    odometry_topic_ = declare_parameter<std::string>(
        "odometry_topic", legacy_odom_topic);
    // This is deliberately separate from the optional localization input.
    // In no_odom_phase it consumes *only* twist.x/twist.y magnitude to adapt
    // the Phase leg timing. Pose, velocity direction, and this timestamp
    // never enter the Phase ABBA regression or the bearing controller.
    motion_response_enabled_ = declare_parameter<bool>(
        "motion_response_enabled", false);
    motion_response_velocity_topic_ = declare_parameter<std::string>(
        "motion_response_velocity_topic", odometry_topic_);
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/mavros/imu/data");
    depth_pose_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    const auto legacy_state_topic =
        declare_parameter<std::string>("state_topic", "/mavros/state");
    vehicle_state_topic_ = declare_parameter<std::string>(
        "vehicle_state_topic", legacy_state_topic);
    delta_range_topic_ = declare_parameter<std::string>(
        "delta_range_topic", "/audio_phase_estimator/delta_range_m");
    iq_magnitude_topic_ = declare_parameter<std::string>(
        "iq_magnitude_topic", "/audio_phase_estimator/iq_magnitude");
    audio_quality_topic_ = declare_parameter<std::string>(
        "audio_quality_topic", "/audio_phase_estimator/iq_snr_ratio");
    bootstrap_probe_on_audio_quality_ = declare_parameter<bool>(
        "bootstrap_probe_on_audio_quality", true);
    audio_quality_timeout_s_ = std::max(
        0.5, declare_parameter<double>("audio_quality_timeout_s", 3.0));
    direction_output_topic_ = declare_parameter<std::string>(
        "direction_output_topic", "/pinger_homing/direction_body");
    control_direction_output_topic_ = declare_parameter<std::string>(
        "control_direction_output_topic", "/pinger_homing/control_direction_body");
    status_topic_ = declare_parameter<std::string>("status_topic", "/pinger_homing/status");
    const auto legacy_rc_topic =
        declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
    rc_output_topic_ = declare_parameter<std::string>("rc_output_topic", legacy_rc_topic);
    command_override_topic_ = declare_parameter<std::string>(
        "command_override_topic", "/uuv_mujoco/sitl/command_override");

    dry_run_ = declare_parameter<bool>("dry_run", true);
    rate_hz_ = clamp(declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
    control_period_s_ = 1.0 / rate_hz_;
    const double legacy_forward = clamp(
        declare_parameter<double>("forward_fast", fast_motion_profile ? 0.78 : 0.48),
        0.1, 0.8);
    forward_max_ = clamp(declare_parameter<double>("forward_max", legacy_forward), 0.1, 0.8);
    max_runtime_s_ = std::max(0.0, declare_parameter<double>("max_runtime_s", 0.0));
    terminal_exit_delay_s_ = std::max(
        0.0, declare_parameter<double>("terminal_exit_delay_s", 3.0));
    yaw_gain_ = clamp(
        declare_parameter<double>("yaw_gain", fast_motion_profile ? 1.15 : 0.85),
        0.1, 2.0);
    yaw_rate_damping_ = clamp(
        declare_parameter<double>(
            "yaw_rate_damping", fast_motion_profile ? 0.16 : 0.18),
        0.0, 1.0);
    const double legacy_yaw_limit = clamp(
        declare_parameter<double>("yaw_limit", fast_motion_profile ? 0.62 : 0.42),
        0.10, 0.70);
    yaw_command_limit_ = clamp(
        declare_parameter<double>("yaw_command_limit", legacy_yaw_limit), 0.10, 0.70);
    yaw_deadband_rad_ = radians(clamp(
        declare_parameter<double>("yaw_deadband_deg", 2.5), 0.0, 10.0));
    yaw_slew_rate_ = clamp(
        declare_parameter<double>("yaw_slew_rate", fast_motion_profile ? 2.2 : 0.90),
        0.1, 4.0);
    yaw_brake_gain_ = clamp(
        declare_parameter<double>("yaw_brake_gain", fast_motion_profile ? 0.75 : 0.55),
        0.0, 2.0);
    yaw_brake_slew_rate_ = clamp(
        declare_parameter<double>(
            "yaw_brake_slew_rate", 2.2),
        0.1, 8.0);
    yaw_brake_horizon_s_ = clamp(
        declare_parameter<double>(
            "yaw_brake_horizon_s", fast_motion_profile ? 0.55 : 0.45),
        0.0, 1.5);
    yaw_settle_rate_rad_s_ = clamp(
        declare_parameter<double>(
            "yaw_settle_rate_rad_s", fast_motion_profile ? 0.12 : 0.10),
        0.01, 1.0);
    yaw_settle_hold_s_ = clamp(
        declare_parameter<double>(
            "yaw_settle_hold_s", 0.20),
        0.0, 2.0);
    direction_input_filter_alpha_ = clamp(
        declare_parameter<double>(
            "direction_input_filter_alpha", fast_motion_profile ? 0.35 : 0.12),
        0.05, 1.0);
    direction_filter_alpha_ = clamp(
        declare_parameter<double>(
            "direction_filter_alpha",
            fast_motion_profile ? (acoustic_estimator_mode_ == "snr" ? 0.34 : 0.30) : 0.22),
        0.05, 1.0);
    align_enter_rad_ = radians(clamp(
        declare_parameter<double>("align_enter_deg", fast_motion_profile ? 36.0 : 28.0),
        12.0, 60.0));
    align_exit_rad_ = radians(clamp(
        declare_parameter<double>("align_exit_deg", fast_motion_profile ? 14.0 : 10.0),
        2.0, 20.0));
    align_exit_rad_ = std::min(align_exit_rad_, align_enter_rad_ - radians(2.0));
    probe_scale_ = clamp(declare_parameter<double>("probe_scale", 0.28), 0.02, 0.30);
    odometry_timeout_s_ = std::max(
        0.2, declare_parameter<double>("odometry_timeout_s", 2.0));
    imu_timeout_s_ = std::max(
        0.1, declare_parameter<double>("imu_timeout_s", 1.0));
    depth_pose_timeout_s_ = std::max(
        0.1, declare_parameter<double>("depth_pose_timeout_s", 1.0));
    audio_timeout_s_ = std::max(0.5, declare_parameter<double>("audio_timeout_s", 3.0));
    vehicle_disconnect_grace_s_ = std::max(
        0.0, declare_parameter<double>("vehicle_disconnect_grace_s", 0.0));
    vehicle_state_timeout_s_ = std::max(
        0.2, declare_parameter<double>("vehicle_state_timeout_s", 3.5));
    direction_timeout_s_ = std::max(
        0.1, declare_parameter<double>(
            "direction_timeout_s", 3.0));
    no_odom_probe_scale_ = clamp(
        declare_parameter<double>("no_odom_probe_scale", 0.22), 0.08, 0.35);
    no_odom_probe_heave_scale_ = clamp(
        declare_parameter<double>("no_odom_probe_heave_scale", 0.12), 0.05, 0.25);
    no_odom_probe_leg_s_ = clamp(
        declare_parameter<double>("no_odom_probe_leg_s", 1.5), 0.5, 4.0);
    no_odom_probe_neutral_s_ = clamp(
        declare_parameter<double>("no_odom_probe_neutral_s", 0.50), 0.2, 2.0);
    no_odom_probe_settle_s_ = clamp(
        declare_parameter<double>("no_odom_probe_settle_s", 0.80), 0.2, 3.0);
    // RC demand is not vehicle velocity.  In STABILIZE the first part of a
    // leg contains thruster spool-up and the previous leg's hydrodynamic
    // momentum, so it must not be interpreted as a Phase gradient sample.
    const double default_probe_sample_delay_s =
        std::min(0.45, 0.35 * no_odom_probe_leg_s_);
    no_odom_probe_sample_delay_s_ = clamp(
        declare_parameter<double>(
            "no_odom_probe_sample_delay_s", default_probe_sample_delay_s),
        0.0, 0.80 * no_odom_probe_leg_s_);
    no_odom_probe_huber_k_ = clamp(
        declare_parameter<double>("no_odom_probe_huber_k", 1.5), 1.0, 3.0);
    no_odom_initial_confirmation_probes_ = std::max(
        1, std::min(3, static_cast<int>(declare_parameter<int>(
            "no_odom_initial_confirmation_probes", 1))));
    no_odom_initial_confirmation_dot_min_ = clamp(
        declare_parameter<double>("no_odom_initial_confirmation_dot_min", 0.75),
        -1.0, 1.0);
    no_odom_min_samples_per_leg_ = std::max(
        2, static_cast<int>(declare_parameter<int>(
            "no_odom_min_samples_per_leg", 3)));
    no_odom_min_horizontal_signal_ = std::max(
        1.0e-6, declare_parameter<double>("no_odom_min_horizontal_signal", 1.0e-4));
    no_odom_forward_command_ = clamp(
        declare_parameter<double>("no_odom_forward_command", 0.30), 0.05, 0.50);
    no_odom_reestimate_policy_ = declare_parameter<std::string>(
        "no_odom_reestimate_policy", "adaptive");
    if (no_odom_reestimate_policy_ != "adaptive" &&
        no_odom_reestimate_policy_ != "fixed") {
      throw std::invalid_argument(
          "no_odom_reestimate_policy must be 'adaptive' or 'fixed'");
    }
    // ``forward_duration`` is retained only for an explicit legacy fixed
    // cadence.  Adaptive Phase runs re-probe from an acoustic innovation or
    // vehicle-response event; this watchdog prevents an indefinitely stale
    // bearing if neither feedback stream is usable.
    no_odom_forward_duration_s_ = clamp(
        declare_parameter<double>("no_odom_forward_duration_s", 4.0), 0.5, 40.0);
    no_odom_approach_min_s_ = clamp(
        declare_parameter<double>("no_odom_approach_min_s", 2.5), 0.0, 30.0);
    no_odom_approach_max_s_ = clamp(
        declare_parameter<double>("no_odom_approach_max_s", 25.0), 0.0, 120.0);
    no_odom_innovation_enabled_ = declare_parameter<bool>(
        "no_odom_innovation_enabled", true);
    no_odom_innovation_window_s_ = clamp(
        declare_parameter<double>("no_odom_innovation_window_s", 0.70), 0.10, 5.0);
    no_odom_innovation_noise_floor_m_ = clamp(
        declare_parameter<double>("no_odom_innovation_noise_floor_m", 0.0005),
        1.0e-6, 0.02);
    no_odom_innovation_limit_ = clamp(
        declare_parameter<double>("no_odom_innovation_limit", 1.50), 0.25, 10.0);
    no_odom_innovation_hold_s_ = clamp(
        declare_parameter<double>("no_odom_innovation_hold_s", 1.20), 0.10, 10.0);
    no_odom_innovation_min_expected_delta_m_ = clamp(
        declare_parameter<double>("no_odom_innovation_min_expected_delta_m", 0.0002),
        1.0e-6, 0.02);
    no_odom_terminal_brake_enabled_ = declare_parameter<bool>(
        "no_odom_terminal_brake_enabled", true);
    no_odom_terminal_brake_command_ = clamp(
        declare_parameter<double>("no_odom_terminal_brake_command", 0.22),
        0.0, kmu26::pinger_homing::kMaxNoOdomTerminalBrakeCommand);
    no_odom_terminal_brake_duration_s_ = clamp(
        declare_parameter<double>("no_odom_terminal_brake_duration_s", 0.90),
        0.0, 3.0);
    no_odom_vertical_control_enabled_ = declare_parameter<bool>(
        "no_odom_vertical_control_enabled", true);
    // In a shallow test tank ALT_HOLD owns Z.  This optional profile keeps
    // the proven ABBA/Huber estimator but fits only the commanded XY legs.
    no_odom_horizontal_only_ = declare_parameter<bool>(
        "no_odom_horizontal_only", false);
    no_odom_heave_limit_ = clamp(
        declare_parameter<double>("no_odom_heave_limit", 0.18), 0.0, 0.30);
    // Requested RC magnitude is not a speed setpoint.  Fresh velocity only
    // decides when a commanded Phase leg has moved enough to be sampled. A
    // slow leg is extended (not discarded), and a blocked forward segment
    // returns to a neutral ABBA re-estimate instead of declaring failure.
    motion_response_min_command_ = clamp(
        declare_parameter<double>("motion_response_min_command", 0.05), 0.01, 0.80);
    motion_response_min_speed_mps_ = clamp(
        declare_parameter<double>("motion_response_min_speed_mps", 0.03), 0.001, 1.0);
    motion_response_probe_extension_s_ = clamp(
        declare_parameter<double>("motion_response_probe_extension_s", 0.30), 0.05, 2.0);
    motion_response_probe_max_extension_s_ = clamp(
        declare_parameter<double>("motion_response_probe_max_extension_s", 1.20), 0.0, 6.0);
    motion_response_approach_grace_s_ = clamp(
        declare_parameter<double>("motion_response_approach_grace_s", 0.80), 0.0, 5.0);
    motion_response_approach_hold_s_ = clamp(
        declare_parameter<double>("motion_response_approach_hold_s", 0.80), 0.10, 10.0);
    motion_response_feedback_timeout_s_ = clamp(
        declare_parameter<double>("motion_response_feedback_timeout_s", 0.75), 0.10, 5.0);
    require_source_lock_ = declare_parameter<bool>(
        "require_source_lock", legacy_python_sequence_);
    prefer_direction_control_ = declare_parameter<bool>(
        "prefer_direction_control",
        !legacy_python_sequence_ &&
            (sim_fast_profile || acoustic_estimator_mode_ == "snr"));
    phase_yaw_guidance_enabled_ = declare_parameter<bool>(
        "phase_yaw_guidance_enabled", acoustic_estimator_mode_ == "phase");
    phase_yaw_min_range_m_ = std::max(
        0.0, declare_parameter<double>("phase_yaw_min_range_m", 6.0));
    phase_vertical_disambiguation_enabled_ = declare_parameter<bool>(
        "phase_vertical_disambiguation_enabled", false);
    phase_bearing_follow_after_probe_s_ = clamp(
        declare_parameter<double>("phase_bearing_follow_after_probe_s", 2.5),
        0.5, 8.0);
    phase_bearing_follow_min_updates_ = std::max(
        1, static_cast<int>(
            declare_parameter<int>("phase_bearing_follow_min_updates", 3)));
    direction_follow_after_probe_s_ = clamp(
        declare_parameter<double>(
            "direction_follow_after_probe_s",
            fast_motion_profile
                ? 5.0
                : phase_bearing_follow_after_probe_s_),
        0.2, 12.0);
    direction_follow_min_updates_ = std::max(
        1, static_cast<int>(declare_parameter<int>(
            "direction_follow_min_updates", phase_bearing_follow_min_updates_)));
    source_phase_alignment_min_ = clamp(
        declare_parameter<double>(
            "source_phase_alignment_min", legacy_python_sequence_ ? 0.20 : 0.82),
        0.0, 1.0);
    source_fit_period_s_ = clamp(
        declare_parameter<double>(
            "source_fit_period_s", legacy_python_sequence_ ? 0.45 : 1.20),
        0.40, 5.0);
    source_range_consistency_ratio_ = clamp(
        declare_parameter<double>("source_range_consistency_ratio", 0.60), 0.1, 0.95);
    source_range_consistency_margin_m_ = std::max(
        0.0, declare_parameter<double>("source_range_consistency_margin_m", 0.75));
    source_absolute_median_limit_m_ = std::max(
        0.05, declare_parameter<double>("source_absolute_median_limit_m", 0.90));
    source_absolute_rms_limit_m_ = std::max(
        source_absolute_median_limit_m_,
        declare_parameter<double>("source_absolute_rms_limit_m", 1.50));
    source_absolute_latest_margin_m_ = std::max(
        0.05, declare_parameter<double>("source_absolute_latest_margin_m", 0.75));
    source_absolute_latest_ratio_ = clamp(
        declare_parameter<double>("source_absolute_latest_ratio", 0.15), 0.01, 0.75);

    tank_max_depth_m_ = std::max(
        0.0, declare_parameter<double>("tank_max_depth_m", 0.0));
    auto_source_depth_ = tank_max_depth_m_ > 0.0;
    const double legacy_pinger_depth = std::max(
        0.0, declare_parameter<double>("pinger_depth_m", 8.85));
    const double legacy_vehicle_target = std::max(
        0.0, declare_parameter<double>("pinger_contact_depth_m", 8.50));
    const double expected_depth = declare_parameter<double>("pinger_expected_depth_m", -1.0);
    const double target_depth = declare_parameter<double>("vehicle_target_depth_m", -1.0);
    // Match the deployed Python Phase controller: when the operator provides
    // only the tank maximum depth, that value is a 3-D solver/safety bound,
    // not a hidden pinger-depth assumption.  Fixing Z to tank_depth * 0.8 was
    // the cause of the initial blind descent and prevented the vertical probe
    // from estimating the pinger depth.  Keep the established derived-depth
    // policy for the independent SNR profile until it is validated against
    // the real SNR direction contract.
    const bool python_auto_depth = auto_source_depth_ && legacy_python_sequence_;
    pinger_expected_depth_m_ = python_auto_depth
        ? 0.0
        : (auto_source_depth_
            ? kmu26::pinger_homing::derive_pinger_depth_from_tank(tank_max_depth_m_)
            : (expected_depth > 0.0 ? expected_depth : legacy_pinger_depth));
    vehicle_target_depth_m_ = python_auto_depth
        ? 0.0
        : (auto_source_depth_
            ? kmu26::pinger_homing::derive_vehicle_target_depth_from_tank(tank_max_depth_m_)
            : (target_depth > 0.0 ? target_depth : legacy_vehicle_target));
    const double explicit_max_vehicle_depth = std::max(
        0.0, declare_parameter<double>("max_vehicle_depth_m", 0.0));
    max_vehicle_depth_m_ = auto_source_depth_
        ? kmu26::pinger_homing::derive_vehicle_depth_limit(tank_max_depth_m_)
        : explicit_max_vehicle_depth;
    depth_target_control_ = declare_parameter<bool>(
        "depth_target_control", auto_source_depth_ && !legacy_python_sequence_);
    depth_soft_margin_m_ = std::max(
        0.0, declare_parameter<double>("depth_soft_margin_m", 0.15));
    depth_recovery_heave_ = clamp(
        declare_parameter<double>("depth_recovery_heave", 0.12), 0.0, 0.4);
    probe_heave_ = clamp(declare_parameter<double>("probe_heave", -0.18), -0.4, 0.4);
    auto_probe_heave_magnitude_ = auto_source_depth_
        ? (tank_max_depth_m_ <= 2.0 ? 0.10
           : 0.10 + (std::min(tank_max_depth_m_, 11.0) - 2.0) * (0.10 / 9.0))
        : 0.10;
    const double default_probe_duration_scale = tank_max_depth_m_ <= 2.0 ? 1.0 : 1.5;
    probe_duration_scale_ = clamp(declare_parameter<double>(
        "probe_duration_scale",
        fast_probe_profile ? 0.55 : default_probe_duration_scale),
        0.25, 3.0);
    // A deep tank needs the mirrored second leg even in sim_fast.  A live
    // full-physics run with one leg reached 5.25 m quickly but produced a
    // weak depth fit, reversed heave, and correctly fell back to REPROBE.
    // Keep test-tank operation at one leg and deep-tank/real operation at two.
    const int default_probe_legs = tank_max_depth_m_ <= 2.0 ? 1 : 2;
    minimum_probe_legs_ = std::clamp(
        static_cast<int>(declare_parameter<int>(
            "minimum_probe_legs", fast_probe_profile ? 1 : default_probe_legs)),
        1, 4);
    fast_probe_enabled_ = declare_parameter<bool>(
        "fast_probe_enabled", fast_probe_profile);
    fast_probe_duration_s_ = clamp(
        declare_parameter<double>(
            "fast_probe_duration_s", fast_probe_profile ? 4.5 : 6.0),
        2.0, 15.0);
    fast_probe_forward_ = clamp(
        declare_parameter<double>("fast_probe_forward", 0.30), 0.0, 0.45);
    fast_probe_lateral_ = clamp(
        declare_parameter<double>("fast_probe_lateral", 0.08), 0.0, 0.30);
    fast_probe_heave_ = clamp(
        declare_parameter<double>("fast_probe_heave", -0.12), -0.35, 0.35);
    fast_probe_yaw_ = clamp(
        declare_parameter<double>("fast_probe_yaw", 0.28), 0.0, 0.50);
    unknown_range_m_ = clamp(
        declare_parameter<double>("unknown_range_m", 12.0), 2.0, 40.0);
    forward_mid_ = clamp(declare_parameter<double>(
        "forward_mid", fast_motion_profile ? 0.52 : 0.34), 0.05, 0.8);
    forward_near_ = clamp(declare_parameter<double>(
        "forward_near", fast_motion_profile ? 0.30 : 0.20), 0.05, 0.6);
    forward_contact_ = clamp(declare_parameter<double>(
        "forward_contact", fast_motion_profile ? 0.16 : 0.13), 0.0, 0.4);
    align_forward_ = clamp(declare_parameter<double>(
        "align_forward", fast_motion_profile ? 0.16 : 0.0), 0.0, 0.35);
    heave_gain_ = clamp(declare_parameter<double>(
        "heave_gain", fast_motion_profile ? 1.05 : 0.75), 0.1, 2.0);
    heave_limit_ = clamp(declare_parameter<double>(
        "heave_limit", fast_motion_profile ? 0.62 : 0.38), 0.05, 0.8);
    range_progress_delta_m_ = clamp(declare_parameter<double>(
        "range_progress_delta_m", 0.12), 0.02, 1.0);
    range_progress_timeout_s_ = clamp(declare_parameter<double>(
        "range_progress_timeout_s", fast_motion_profile ? 6.0 : 8.0), 2.0, 30.0);
    range_gradient_enabled_ = declare_parameter<bool>(
        "range_gradient_enabled", fast_motion_profile);
    continuous_range_excitation_enabled_ = declare_parameter<bool>(
        "continuous_range_excitation_enabled", fast_motion_profile);
    range_guidance_require_two_axis_ = declare_parameter<bool>(
        "range_guidance_require_two_axis", !fast_motion_profile);
    direction_vectoring_enabled_ = declare_parameter<bool>(
        "direction_vectoring_enabled", fast_motion_profile);
    range_gradient_takeover_m_ = clamp(declare_parameter<double>(
        "range_gradient_takeover_m", fast_motion_profile ? 30.0 : 9.0), 1.0, 40.0);
    range_regression_margin_m_ = clamp(declare_parameter<double>(
        "range_regression_margin_m", 0.60), 0.10, 2.0);
    range_regression_hold_s_ = clamp(declare_parameter<double>(
        "range_regression_hold_s", 1.20), 0.10, 3.0);
    range_gradient_window_s_ = clamp(declare_parameter<double>(
        "range_gradient_window_s", 7.0), 2.0, 15.0);
    range_gradient_min_span_m_ = clamp(declare_parameter<double>(
        "range_gradient_min_span_m", 0.20), 0.08, 1.0);
    range_gradient_max_rms_m_ = clamp(declare_parameter<double>(
        "range_gradient_max_rms_m", fast_motion_profile ? 0.30 : 0.45), 0.05, 2.0);
    range_gradient_max_age_s_ = clamp(declare_parameter<double>(
        "range_gradient_max_age_s", 12.0), 1.0, 20.0);
    range_feedback_probe_duration_s_ = clamp(declare_parameter<double>(
        "range_feedback_probe_duration_s", 4.5), 3.0, 10.0);
    range_gradient_forward_ = clamp(declare_parameter<double>(
        "range_gradient_forward", fast_motion_profile ? 0.80 : 0.34), 0.10, 0.8);
    range_guidance_filter_alpha_ = clamp(declare_parameter<double>(
        "range_guidance_filter_alpha", fast_motion_profile ? 0.18 : 0.30), 0.02, 1.0);
    range_guidance_max_step_rad_ = radians(clamp(declare_parameter<double>(
        "range_guidance_max_step_deg", fast_motion_profile ? 16.0 : 30.0), 2.0, 90.0));
    range_excitation_sway_ = clamp(declare_parameter<double>(
        "range_excitation_sway", fast_motion_profile ? 0.14 : 0.0), 0.0, 0.35);
    range_excitation_period_s_ = clamp(declare_parameter<double>(
        "range_excitation_period_s", fast_motion_profile ? 3.6 : 5.0), 1.5, 12.0);
    acoustic_position_enabled_ = declare_parameter<bool>(
        "acoustic_position_enabled", fast_motion_profile);
    acoustic_position_control_enabled_ = declare_parameter<bool>(
        "acoustic_position_control_enabled", false);
    acoustic_position_max_condition_ = clamp(declare_parameter<double>(
        "acoustic_position_max_condition", 50.0), 2.0, 200.0);
    acoustic_position_max_median_residual_m_ = clamp(declare_parameter<double>(
        "acoustic_position_max_median_residual_m", 0.35), 0.05, 1.0);
    acoustic_position_max_rms_residual_m_ = clamp(declare_parameter<double>(
        "acoustic_position_max_rms_residual_m", 0.85), 0.10, 2.0);
    acoustic_position_max_age_s_ = clamp(declare_parameter<double>(
        "acoustic_position_max_age_s", 20.0), 2.0, 60.0);
    acoustic_position_consistency_margin_m_ = clamp(declare_parameter<double>(
        "acoustic_position_consistency_margin_m", 2.5), 0.5, 8.0);
    acoustic_position_consistency_ratio_ = clamp(declare_parameter<double>(
        "acoustic_position_consistency_ratio", 0.35), 0.05, 0.8);
    acoustic_position_sway_gain_ = clamp(declare_parameter<double>(
        "acoustic_position_sway_gain", fast_motion_profile ? 0.85 : 0.0), 0.0, 1.0);
    acoustic_position_sway_limit_ = clamp(declare_parameter<double>(
        "acoustic_position_sway_limit", fast_motion_profile ? 0.60 : 0.0), 0.0, 0.8);
    success_range_m_ = std::max(
        0.0, declare_parameter<double>("success_range_m", 0.0));
    success_hold_s_ = std::max(
        0.1, declare_parameter<double>("success_hold_s", 0.8));
    arrival_radius_m_ = std::max(
        0.0, declare_parameter<double>("arrival_radius_m", 0.8));
    arrival_hold_s_ = std::max(
        0.1, declare_parameter<double>("arrival_hold_s", 1.0));
    first_lock_confirmation_radius_m_ = std::max(
        arrival_radius_m_,
        declare_parameter<double>("first_lock_confirmation_radius_m", 3.0));
    amplitude_range_constant_ = std::max(
        0.0, declare_parameter<double>("amplitude_range_constant", 0.325));
    // /odometry/filtered is a startup-local frame. Course-map XY limits are
    // meaningful only when an operator deliberately supplies map-aligned
    // odometry, so the real-vehicle default validates depth/range/residuals
    // without assuming a global XY origin or yaw.
    source_xy_bounds_enabled_ = declare_parameter<bool>(
        "source_xy_bounds_enabled", false);
    source_min_x_m_ = declare_parameter<double>("source_min_x_m", -16.5);
    source_max_x_m_ = declare_parameter<double>("source_max_x_m", 16.5);
    source_min_y_m_ = declare_parameter<double>("source_min_y_m", -14.0);
    source_max_y_m_ = declare_parameter<double>("source_max_y_m", 14.0);
    max_source_z_world_ = declare_parameter<double>("max_source_z_world", -0.5);
    pinger_min_submergence_m_ = std::max(
        0.05, declare_parameter<double>("pinger_min_submergence_m", 0.20));
    if (auto_source_depth_) max_source_z_world_ = -pinger_min_submergence_m_;
    if (!auto_source_depth_ && max_vehicle_depth_m_ > 0.0 &&
        vehicle_target_depth_m_ > max_vehicle_depth_m_) {
      vehicle_target_depth_m_ = max_vehicle_depth_m_;
    }

    rc_pwm_span_ = clamp(declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);
    rc_deadzone_compensation_enabled_ = declare_parameter<bool>(
        "rc_deadzone_compensation_enabled", true);
    rc_xy_deadzone_pwm_ = clamp(
        declare_parameter<double>("rc_xy_deadzone_pwm", 30.0), 0.0, 200.0);
    rc_yaw_deadzone_pwm_ = clamp(
        declare_parameter<double>("rc_yaw_deadzone_pwm", 40.0), 0.0, 200.0);
    rc_heave_deadzone_pwm_ = clamp(
        declare_parameter<double>("rc_heave_deadzone_pwm", 30.0), 0.0, 200.0);
    const int probe_pwm_override = declare_parameter<int>("probe_pwm_delta", 0);
    const int approach_pwm_override = declare_parameter<int>("approach_pwm_delta", 0);
    const int no_odom_probe_pwm_override = declare_parameter<int>(
        "no_odom_probe_pwm_delta", 0);
    const int no_odom_approach_pwm_override = declare_parameter<int>(
        "no_odom_approach_pwm_delta", 0);
    const int no_odom_terminal_brake_pwm_override = declare_parameter<int>(
        "no_odom_terminal_brake_pwm_delta", 0);
    // The physical launch exposes PWM deltas, not normalized commands. Apply
    // them to the odometry/legacy path as well as to the no-odometry fallback;
    // previously a displayed 20/25 profile silently drove 112/192 PWM in the
    // odometry controller through its old 0.28/0.48 defaults.
    if (probe_pwm_override > 0) {
      probe_scale_ = clamp(
          static_cast<double>(probe_pwm_override) / rc_pwm_span_, 0.02, 0.30);
    }
    if (approach_pwm_override > 0) {
      forward_max_ = clamp(
          static_cast<double>(approach_pwm_override) / rc_pwm_span_, 0.03, 0.80);
    }
    if (no_odom_probe_pwm_override > 0) {
      no_odom_probe_scale_ = no_odom_probe_scale_from_pwm_override(
          no_odom_probe_pwm_override, rc_pwm_span_, no_odom_probe_scale_);
    }
    if (no_odom_approach_pwm_override > 0) {
      no_odom_forward_command_ = clamp(
          static_cast<double>(no_odom_approach_pwm_override) / rc_pwm_span_,
          0.05,
          0.50);
    }
    if (no_odom_terminal_brake_pwm_override > 0) {
      no_odom_terminal_brake_command_ = clamp(
          static_cast<double>(no_odom_terminal_brake_pwm_override) / rc_pwm_span_,
          0.0,
          kmu26::pinger_homing::kMaxNoOdomTerminalBrakeCommand);
    }
    no_odom_probe_pwm_delta_ = static_cast<int>(
        std::lround(no_odom_probe_scale_ * rc_pwm_span_));
    no_odom_approach_pwm_delta_ = static_cast<int>(
        std::lround(no_odom_forward_command_ * rc_pwm_span_));
    no_odom_terminal_brake_pwm_delta_ = static_cast<int>(
        std::lround(no_odom_terminal_brake_command_ * rc_pwm_span_));
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    auto_mode_ = declare_parameter<bool>("auto_mode", false);
    mode_ = declare_parameter<std::string>("mode", "STABILIZE");
    if (mode_ != "STABILIZE" && mode_ != "ALT_HOLD") {
      throw std::invalid_argument(
          "pinger homing live RC requires STABILIZE or ALT_HOLD mode");
    }
    if (legacy_python_sequence_) {
      // These are invariants of the validated Python controller, not tuning
      // suggestions. Keep experimental simulator shortcuts from silently
      // changing the Phase state machine when controller_profile=sim_fast.
      require_source_lock_ = true;
      prefer_direction_control_ = false;
      // Once the metric source is locked, the validated Python controller
      // steers exclusively from source-position minus odometry.  Replacing
      // that yaw with the travelling Phase EMA caused repeated 180-degree
      // ALIGN/APPROACH hunting in full physics.
      // This is part of the Phase algorithm, not a real-vs-sim tuning knob.
      // The original Python controller follows the locked metric source after
      // probing; mixing the continuously changing phase-bearing into yaw made
      // the C++ simulator path hunt and repeatedly reprobe.
      phase_yaw_guidance_enabled_ = false;
      if (controller_profile_ == "real") {
        yaw_gain_ = std::min(yaw_gain_, 0.85);
        yaw_command_limit_ = std::min(yaw_command_limit_, 0.42);
        yaw_slew_rate_ = std::min(yaw_slew_rate_, 0.90);
      }
      fast_probe_enabled_ = false;
      range_gradient_enabled_ = false;
      continuous_range_excitation_enabled_ = false;
      direction_vectoring_enabled_ = false;
      acoustic_position_enabled_ = false;
      acoustic_position_control_enabled_ = false;
      align_forward_ = 0.0;
      depth_target_control_ = false;
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    if (navigation_mode_ == "odometry") {
      odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odometry_topic_, sensor_qos,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odometry(*msg); });
    } else if (motion_response_enabled_ && !motion_response_velocity_topic_.empty()) {
      motion_response_velocity_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          motion_response_velocity_topic_, sensor_qos,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            on_motion_response_velocity(*msg);
          });
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, sensor_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); });
    depth_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        depth_pose_topic_, sensor_qos,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          on_depth_pose(*msg);
        });
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        vehicle_state_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](const mavros_msgs::msg::State::SharedPtr msg) { on_vehicle_state(*msg); });
    delta_range_sub_ = create_subscription<std_msgs::msg::Float64>(
        delta_range_topic_, rclcpp::QoS(50),
        [this](const std_msgs::msg::Float64::SharedPtr msg) { on_delta_range(msg->data); });
    iq_magnitude_sub_ = create_subscription<std_msgs::msg::Float64>(
        iq_magnitude_topic_, rclcpp::QoS(50),
        [this](const std_msgs::msg::Float64::SharedPtr msg) { on_iq_magnitude(msg->data); });
    audio_quality_sub_ = create_subscription<std_msgs::msg::Float64>(
        audio_quality_topic_, rclcpp::QoS(50),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (!std::isfinite(msg->data)) return;
          last_audio_quality_value_ = msg->data;
          last_audio_quality_ = SteadyClock::now();
        });
    if (navigation_mode_ == "odometry") {
      direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
          direction_topic_, rclcpp::QoS(10),
          [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
            on_direction(*msg);
          });
    }
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_output_topic_, rclcpp::QoS(10));
    direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        direction_output_topic_, rclcpp::QoS(10));
    control_direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        control_direction_output_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));
    direct_pub_ = create_publisher<std_msgs::msg::String>(
        command_override_topic_, rclcpp::QoS(10));
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_period_s_)),
        [this]() { control_tick(); });
    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(200), [this]() { publish_status(); });

    RCLCPP_INFO(
        get_logger(),
        "C++ pinger homing ready mode=%s dry_run=%s odom=%s imu=%s depth=%s "
        "audio=%s,%s direction=%s rc=%s tank_depth=%.3fm max_vehicle_depth=%.3fm "
        "legacy_python_sequence=%s navigation_mode=%s motion_response=%s velocity=%s",
        controller_mode_.c_str(), dry_run_ ? "true" : "false", odometry_topic_.c_str(),
        imu_topic_.c_str(), depth_pose_topic_.c_str(), delta_range_topic_.c_str(),
        iq_magnitude_topic_.c_str(), direction_topic_.c_str(), rc_output_topic_.c_str(),
        tank_max_depth_m_, max_vehicle_depth_m_, legacy_python_sequence_ ? "true" : "false",
        navigation_mode_.c_str(), motion_response_enabled_ ? "true" : "false",
        motion_response_velocity_topic_.c_str());
  }

  ~PingerHomingController() override {
    publish_release();
  }

 private:
  void on_odometry(const nav_msgs::msg::Odometry &msg) {
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    const Eigen::Vector3d position(p.x, p.y, p.z);
    if (!position.allFinite() || !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w)) {
      return;
    }
    position_ = position;
    qx_ = q.x;
    qy_ = q.y;
    qz_ = q.z;
    qw_ = q.w;
    yaw_rate_rad_s_ = std::isfinite(msg.twist.twist.angular.z)
        ? msg.twist.twist.angular.z : 0.0;
    last_odometry_ = SteadyClock::now();
    on_motion_response_velocity(msg);
  }

  void on_motion_response_velocity(const nav_msgs::msg::Odometry &msg) {
    const double vx = msg.twist.twist.linear.x;
    const double vy = msg.twist.twist.linear.y;
    if (!std::isfinite(vx) || !std::isfinite(vy)) return;
    // The frame of twist may be body or odom, but the XY norm is invariant to
    // yaw.  Z is intentionally excluded because ALT_HOLD/depth settling is
    // not evidence of horizontal collision or progress.
    motion_response_horizontal_speed_mps_ = std::hypot(vx, vy);
    last_motion_response_velocity_ = SteadyClock::now();
  }

  void on_imu(const sensor_msgs::msg::Imu &msg) {
    const auto &q = msg.orientation;
    Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() <= 1.0e-9) return;
    imu_orientation_ = quaternion.normalized();
    imu_yaw_rate_rad_s_ = std::isfinite(msg.angular_velocity.z)
        ? msg.angular_velocity.z : 0.0;
    last_imu_ = SteadyClock::now();
  }

  void on_depth_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &msg) {
    const double z = msg.pose.pose.position.z;
    if (!std::isfinite(z)) return;
    depth_pose_z_ = z;
    last_depth_pose_ = SteadyClock::now();
  }

  void on_vehicle_state(const mavros_msgs::msg::State &msg) {
    const auto now = SteadyClock::now();
    connected_ = msg.connected;
    armed_ = msg.armed;
    actual_vehicle_mode_ = msg.mode;
    last_vehicle_state_ = now;
    if (connected_) last_connected_ = now;
    if (armed_) last_armed_ = now;
  }

  void on_direction(const geometry_msgs::msg::Vector3Stamped &msg) {
    Eigen::Vector3d direction(msg.vector.x, msg.vector.y, msg.vector.z);
    if (!direction.allFinite() || direction.norm() <= 1.0e-6) return;
    direction.normalize();
    if (direction_frame_ == "body") {
      if (!input_direction_body_) {
        input_direction_body_ = direction;
      } else {
        const Eigen::Vector3d filtered =
            (1.0 - direction_input_filter_alpha_) * *input_direction_body_ +
            direction_input_filter_alpha_ * direction;
        if (filtered.norm() > 1.0e-6) input_direction_body_ = filtered.normalized();
      }
      if (const auto quaternion = attitude_quaternion(SteadyClock::now())) {
        direction = quaternion->toRotationMatrix() * direction;
      } else {
        last_direction_ = SteadyClock::now();
        ++direction_update_count_;
        return;
      }
    }
    if (!phase_direction_world_) {
      phase_direction_world_ = direction;
    } else {
      const Eigen::Vector3d filtered =
          (1.0 - direction_input_filter_alpha_) * *phase_direction_world_ +
          direction_input_filter_alpha_ * direction;
      if (filtered.norm() > 1.0e-6) phase_direction_world_ = filtered.normalized();
    }
    last_direction_ = SteadyClock::now();
    ++direction_update_count_;
  }

  void on_delta_range(double delta) {
    if (!std::isfinite(delta)) return;
    const auto now = SteadyClock::now();
    last_audio_ = now;
    const double bounded_delta = clamp(delta, -0.06, 0.06);
    if (navigation_mode_ == "no_odom_phase" && state_ == "NO_ODOM_PHASE_PROBE" &&
        no_odom_probe_sample_enabled_) {
      // On the vehicle, regress against the depth-limited command that was
      // actually sent.  Dry-run deliberately releases RC, so its deterministic
      // runtime test uses the bounded request instead.
      const Command &sampled_command = dry_run_ ? last_requested_command_ : last_command_;
      NoOdomPhaseSample sample;
      sample.probe_s = seconds_since(state_started_, now);
      // ArduSub RC6 high commands vehicle-right, while this estimator's body
      // frame is ROS FLU (+Y left).  Regress Phase against realized FLU axes,
      // not raw RC stick signs.  A live STABILIZE ±90-PWM probe measured
      // RC6 high -> negative FLU y and RC6 low -> positive FLU y.
      sample.command_body = Eigen::Vector3d(
          sampled_command.forward, -sampled_command.lateral, sampled_command.heave);
      sample.delta_range_m = bounded_delta;
      no_odom_phase_samples_.push_back(sample);
      if (no_odom_probe_axis_ == 0) {
        no_odom_neutral_delta_sum_ += bounded_delta;
        ++no_odom_neutral_delta_count_;
      } else {
        const std::size_t index = static_cast<std::size_t>(no_odom_probe_axis_ + 3);
        if (index < no_odom_probe_delta_sum_.size()) {
          no_odom_probe_delta_sum_[index] += bounded_delta;
          ++no_odom_probe_delta_count_[index];
        }
      }
    }
    raw_delta_history_.push_back(bounded_delta);
    while (raw_delta_history_.size() > 3U) raw_delta_history_.pop_front();
    record_no_odom_approach_delta(now, bounded_delta);
    const std::vector<double> values(raw_delta_history_.begin(), raw_delta_history_.end());
    cumulative_range_change_m_ += kmu26::pinger_homing::median(values);
    // A fresh Phase/direction stream is sufficient for the no-XY fallback.
    // Metric source fitting still requires measured odometry; never invent a
    // pseudo position from RC commands when localization is unavailable.
    if (!position_ || seconds_since(last_odometry_, now) >= odometry_timeout_s_) return;
    RangeSample sample;
    sample.wall_s = seconds_from_epoch(now);
    sample.position_world = *position_;
    sample.cumulative_change_m = cumulative_range_change_m_;
    if (const auto range = amplitude_range()) sample.amplitude_range_m = *range;
    samples_.push_back(sample);
    while (samples_.size() > 5000U) samples_.pop_front();
    range_feedback_samples_.push_back(sample);
    while (range_feedback_samples_.size() > 5000U) range_feedback_samples_.pop_front();
  }

  void on_iq_magnitude(double magnitude) {
    if (amplitude_range_constant_ <= 0.0 || !std::isfinite(magnitude) ||
        magnitude <= 1.0e-6) {
      return;
    }
    const double range = std::pow(amplitude_range_constant_ / magnitude, 2.0);
    if (range > 0.1 && range < 80.0) {
      amplitude_range_history_.push_back(range);
      while (amplitude_range_history_.size() > 9U) amplitude_range_history_.pop_front();
    }
  }

  void control_tick() {
    const auto now = SteadyClock::now();
    maybe_request_arm_mode(now);
    if (state_ == "TERMINAL_BRAKE") {
      terminal_brake_tick(now);
      return;
    }
    if (range_complete_ || state_ == "COMPLETE") {
      publish_command(Command{});
      maybe_shutdown_after_terminal(now);
      return;
    }
    if (state_.rfind("FAILED_", 0) == 0) {
      publish_command(Command{});
      maybe_shutdown_after_terminal(now);
      return;
    }
    if (max_runtime_s_ > 0.0 && active_started_ &&
        seconds_since(*active_started_, now) > max_runtime_s_) {
      transition("FAILED_TIMEOUT");
      publish_command(Command{});
      return;
    }
    // Explicit no-odometry Phase navigation is intentionally isolated from
    // the metric source-fit path below.  It consumes only coherent Phase
    // delta-range samples during known body-axis RC legs, IMU attitude/rate,
    // Bar30 depth and MAVROS state. A separately subscribed odometry twist
    // may only adapt probe timing/re-estimation; position and velocity never
    // affect this mode's Phase regression or bearing.
    if (navigation_mode_ == "no_odom_phase") {
      no_odom_phase_tick(now);
      return;
    }
    // Keep the public acoustic direction contract stable across the entire
    // state machine.  Previously this topic silently changed from the Phase
    // estimator vector during PROBE to the range-localizer steering vector
    // after source lock, which made the red viewer arrow appear to jump to a
    // wrong bearing.  Steering is published on its own topic below.
    if (position_ && direction_usable(now) &&
        seconds_since(last_odometry_, now) < odometry_timeout_s_) {
      publish_phase_direction_if_available(now);
    }
    if (controller_mode_ == "direction") {
      direct_direction_tick(now);
      return;
    }
    if (!ready(now)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "pinger probe blocked: connected=%s armed=%s actual_mode=%s required_mode=%s "
          "odom=%s phase_delta=%s iq_quality=%s",
          connected_ ? "true" : "false", armed_ ? "true" : "false",
          actual_vehicle_mode_.empty() ? "unavailable" : actual_vehicle_mode_.c_str(),
          mode_.c_str(),
          (position_ && seconds_since(last_odometry_, now) < odometry_timeout_s_)
              ? "fresh" : "stale",
          phase_delta_fresh(now) ? "fresh" : "stale",
          audio_quality_fresh(now) ? "fresh" : "stale");
      // Match the deployed Python safety behavior: stale required input stops
      // motion and invalidates the in-progress state timer.  Once odometry and
      // audio recover, WAIT_VEHICLE always starts a fresh PROBE sequence.
      transition("WAIT_VEHICLE");
      publish_command(Command{});
      return;
    }
    if (state_ == "WAIT_VEHICLE") {
      // Match the Python controller: the runtime budget starts when the
      // vehicle is actually ready and the first probe begins, not while
      // MAVROS is still connecting/arming.
      if (!active_started_) {
        // Delta/range callbacks intentionally remain alive while disarmed so
        // input freshness can be established.  None of those stationary
        // pre-arm samples may participate in localization, however: they make
        // the first range sphere dominate the bounded fit.  Start the mission
        // with a clean localization session while preserving the Python probe
        // state/timeline itself.
        reset_localization_for_initial_probe(now);
        active_started_ = now;
      }
      last_range_progress_ = now;
      transition("PROBE");
    }

    // The SNR mode owns its bearing contract.  Its phase estimator remains
    // active only to provide calibrated IQ range, never as a hidden position
    // localizer that can override the SNR direction.
    if (acoustic_estimator_mode_ == "phase") maybe_fit_source(now, false);
    if (maybe_complete(now)) {
      publish_success_terminal_command(now);
      return;
    }
    if (state_ == "PROBE" || state_ == "REPROBE") {
      const bool short_probe_allowed =
          state_ == "PROBE" || (state_ == "REPROBE" && !force_full_reprobe_);
      if (!legacy_python_sequence_ && short_probe_allowed && !require_source_lock_ &&
          direction_update_count_ >=
              static_cast<std::size_t>(direction_follow_min_updates_) &&
          direction_usable(now) &&
          seconds_since(state_started_, now) >= direction_follow_after_probe_s_) {
        RCLCPP_INFO(
            get_logger(),
            "fresh %s bearing available after short probe; source lock remains optional",
            acoustic_estimator_mode_.c_str());
        probe_completed_ = true;
        transition("ALIGN");
        approach_command(now);
        return;
      }
      publish_command(probe_command(now, state_ == "REPROBE"));
      return;
    }
    approach_command(now);
  }

  std::optional<Eigen::Quaterniond> attitude_quaternion(const SteadyTime &now) const {
    if (navigation_mode_ == "odometry" && position_ &&
        seconds_since(last_odometry_, now) < odometry_timeout_s_) {
      Eigen::Quaterniond quaternion(qw_, qx_, qy_, qz_);
      if (quaternion.coeffs().allFinite() && quaternion.norm() > 1.0e-9) {
        return quaternion.normalized();
      }
    }
    if (imu_orientation_ && seconds_since(last_imu_, now) < imu_timeout_s_) {
      return imu_orientation_;
    }
    return std::nullopt;
  }

  double current_yaw_rate(const SteadyTime &now) const {
    if (navigation_mode_ == "odometry" && position_ &&
        seconds_since(last_odometry_, now) < odometry_timeout_s_) {
      return yaw_rate_rad_s_;
    }
    return seconds_since(last_imu_, now) < imu_timeout_s_
        ? imu_yaw_rate_rad_s_ : 0.0;
  }

  std::optional<double> current_position_z(const SteadyTime &now) const {
    if (navigation_mode_ == "odometry" && position_ &&
        seconds_since(last_odometry_, now) < odometry_timeout_s_) {
      return position_->z();
    }
    if (depth_pose_z_ && seconds_since(last_depth_pose_, now) < depth_pose_timeout_s_) {
      return depth_pose_z_;
    }
    return std::nullopt;
  }

  bool phase_delta_fresh(const SteadyTime &now) const {
    return seconds_since(last_audio_, now) < audio_timeout_s_;
  }

  bool audio_quality_fresh(const SteadyTime &now) const {
    return last_audio_quality_ != SteadyTime{} &&
           seconds_since(last_audio_quality_, now) < audio_quality_timeout_s_;
  }

  bool audio_ready_for_probe(const SteadyTime &now) const {
    // A weak carrier may be rejected by the Phase quality gate before it can
    // publish delta_range. The small, bounded probe is what creates the
    // spatial diversity needed to improve that estimate, so allow probing on
    // the estimator's quality heartbeat. Source lock and APPROACH still
    // require real delta-range samples and can never be authorized by this
    // heartbeat alone.
    return phase_delta_fresh(now) ||
           (bootstrap_probe_on_audio_quality_ && audio_quality_fresh(now));
  }

  bool no_odom_phase_ready(const SteadyTime &now) const {
    if ((!dry_run_ && !live_control_ready(now)) ||
        !audio_ready_for_probe(now) ||
        !attitude_quaternion(now)) {
      return false;
    }
    // The physical XY-only profile leaves all vertical authority to
    // ArduSub ALT_HOLD.  In that configuration depth is still exposed in the
    // status/preflight surface, but it must not gate Phase bearing probes:
    // the vehicle has no controller-issued heave command for a floor limiter
    // to constrain.  Three-axis/vertical profiles keep the strict Bar30
    // requirement whenever a tank depth limit is configured.
    const bool passive_vertical_alt_hold =
        no_odom_horizontal_only_ && !no_odom_vertical_control_enabled_;
    return passive_vertical_alt_hold || max_vehicle_depth_m_ <= 0.0 ||
           current_position_z(now).has_value();
  }

  bool motion_response_feedback_fresh(const SteadyTime &now) const {
    return motion_response_horizontal_speed_mps_.has_value() &&
        last_motion_response_velocity_ != SteadyTime{} &&
        seconds_since(last_motion_response_velocity_, now) < motion_response_feedback_timeout_s_;
  }

  bool motion_response_moving(const SteadyTime &now) const {
    return !motion_response_enabled_ || !motion_response_feedback_fresh(now) ||
        *motion_response_horizontal_speed_mps_ >= motion_response_min_speed_mps_;
  }

  void reset_motion_response() {
    motion_response_command_started_.reset();
    motion_response_low_speed_started_.reset();
  }

  void reset_no_odom_approach_innovation() {
    no_odom_approach_samples_.clear();
    no_odom_innovation_bad_started_.reset();
    no_odom_innovation_ratio_.reset();
    no_odom_observed_delta_m_.reset();
  }

  void record_no_odom_approach_delta(const SteadyTime &now, double delta_range_m) {
    if (navigation_mode_ != "no_odom_phase" || state_ != "APPROACH" ||
        !no_odom_forward_started_) {
      return;
    }
    no_odom_approach_samples_.push_back({now, delta_range_m});
    while (!no_odom_approach_samples_.empty() &&
           seconds_since(no_odom_approach_samples_.front().stamp, now) >
               no_odom_innovation_window_s_) {
      no_odom_approach_samples_.pop_front();
    }
  }

  bool handle_no_odom_phase_innovation(const SteadyTime &now) {
    if (no_odom_reestimate_policy_ != "adaptive" ||
        !no_odom_innovation_enabled_ || !no_odom_forward_started_) {
      return false;
    }
    const double elapsed_s = seconds_since(*no_odom_forward_started_, now);
    if (elapsed_s < no_odom_approach_min_s_ ||
        no_odom_approach_samples_.size() < 3U ||
        !std::isfinite(no_odom_forward_expected_delta_m_) ||
        no_odom_forward_expected_delta_m_ >=
            -no_odom_innovation_min_expected_delta_m_) {
      no_odom_innovation_bad_started_.reset();
      return false;
    }

    std::vector<double> observed;
    observed.reserve(no_odom_approach_samples_.size());
    for (const auto &sample : no_odom_approach_samples_) {
      observed.push_back(sample.delta_range_m);
    }
    const double observed_median = kmu26::pinger_homing::median(observed);
    const double innovation_m = observed_median - no_odom_forward_expected_delta_m_;
    const double sigma_m = std::max(
        no_odom_innovation_noise_floor_m_, no_odom_probe_fit_rms_m_);
    const double innovation_ratio = innovation_m / sigma_m;
    no_odom_observed_delta_m_ = observed_median;
    no_odom_innovation_ratio_ = innovation_ratio;

    // A positive innovation means that the actual range closes less than the
    // ABBA bearing predicted (or is opening).  This is the Phase equivalent
    // of a normalized innovation/error gate: do not pretend it is a lateral
    // position error, because a single straight-line scalar observation
    // cannot distinguish left from right.  Instead neutral and collect a new
    // two-axis observable ABBA sample only after the mismatch persists.
    if (innovation_ratio <= no_odom_innovation_limit_) {
      no_odom_innovation_bad_started_.reset();
      return false;
    }
    if (!no_odom_innovation_bad_started_) {
      no_odom_innovation_bad_started_ = now;
      return false;
    }
    if (seconds_since(*no_odom_innovation_bad_started_, now) <
        no_odom_innovation_hold_s_) {
      return false;
    }

    ++no_odom_innovation_reestimate_count_;
    RCLCPP_WARN(
        get_logger(),
        "Phase innovation %.2f exceeds %.2f for %.2fs: observed delta=%+.6g "
        "expected=%+.6g m; neutral then adaptive ABBA re-estimate #%d",
        innovation_ratio, no_odom_innovation_limit_,
        seconds_since(*no_odom_innovation_bad_started_, now), observed_median,
        no_odom_forward_expected_delta_m_, no_odom_innovation_reestimate_count_);
    publish_command(Command{});
    start_no_odom_probe(now);
    return true;
  }

  bool handle_no_odom_approach_watchdog(const SteadyTime &now) {
    if (no_odom_reestimate_policy_ != "adaptive" ||
        no_odom_approach_max_s_ <= 0.0 || !no_odom_forward_started_ ||
        seconds_since(*no_odom_forward_started_, now) < no_odom_approach_max_s_) {
      return false;
    }
    ++no_odom_watchdog_reestimate_count_;
    RCLCPP_WARN(
        get_logger(),
        "adaptive Phase approach watchdog %.1fs reached; neutral then ABBA re-estimate #%d",
        no_odom_approach_max_s_, no_odom_watchdog_reestimate_count_);
    publish_command(Command{});
    start_no_odom_probe(now);
    return true;
  }

  bool handle_no_odom_motion_response(
      const SteadyTime &now, const Command &requested) {
    const double requested_xy = std::hypot(requested.forward, requested.lateral);
    motion_response_requested_xy_ = requested_xy;
    if (!motion_response_enabled_ || state_ != "APPROACH" ||
        requested_xy < motion_response_min_command_ ||
        !motion_response_feedback_fresh(now)) {
      reset_motion_response();
      return false;
    }

    if (!motion_response_command_started_) {
      motion_response_command_started_ = now;
      motion_response_low_speed_started_.reset();
      return false;
    }
    if (seconds_since(*motion_response_command_started_, now) <
        motion_response_approach_grace_s_) {
      return false;
    }
    if (motion_response_moving(now)) {
      // Measured horizontal motion recovered.  Start a new low-speed window
      // only if the vehicle becomes stuck again; do not accumulate separated
      // brief slowdowns such as a yaw correction.
      motion_response_low_speed_started_.reset();
      return false;
    }
    if (!motion_response_low_speed_started_) {
      motion_response_low_speed_started_ = now;
      return false;
    }
    if (seconds_since(*motion_response_low_speed_started_, now) <
        motion_response_approach_hold_s_) {
      return false;
    }

    ++motion_response_reestimate_count_;
    const double low_speed_for_s =
        seconds_since(*motion_response_low_speed_started_, now);
    const double measured_speed_mps = *motion_response_horizontal_speed_mps_;
    // A blocked forward segment is not a mission failure. Neutral first and
    // repeat the complete ABBA sequence from the current physical position;
    // the refreshed Phase slope can select a different yaw/approach direction.
    // start_no_odom_probe() adds its own neutral settle interval before any
    // new excitation, so no command is carried into the obstacle.
    publish_command(Command{});
    reset_motion_response();
    RCLCPP_WARN(
        get_logger(),
        "motion response low: requested XY=%.3f, feedback=%.4f m/s for %.2fs; "
        "neutral then adaptive Phase re-estimate #%d",
        requested_xy, measured_speed_mps, low_speed_for_s,
        motion_response_reestimate_count_);
    start_no_odom_probe(now);
    return true;
  }

  Command no_odom_probe_command(double elapsed_s, bool &complete) {
    complete = false;
    no_odom_probe_axis_ = 0;
    no_odom_probe_sample_enabled_ = false;
    double remaining = std::max(0.0, elapsed_s);
    if (remaining < no_odom_probe_settle_s_) return {};
    remaining -= no_odom_probe_settle_s_;

    // ABBA is symmetric in time for every body axis.  Compared with one +/−
    // pair, +/−/−/+ cancels a first-order clock/current drift without assuming
    // that the vehicle reaches the same speed immediately after each reversal.
    const std::vector<int> axes = no_odom_horizontal_only_
        ? std::vector<int>{1, -1, -1, 1, 2, -2, -2, 2}
        : std::vector<int>{1, -1, -1, 1, 2, -2, -2, 2, 3, -3, -3, 3};
    for (std::size_t leg_index = 0; leg_index < axes.size(); ++leg_index) {
      const int axis = axes[leg_index];
      const double leg_duration = no_odom_probe_leg_s_ +
          no_odom_probe_leg_extensions_s_[leg_index];
      if (remaining < leg_duration) {
        no_odom_probe_axis_ = axis;
        no_odom_probe_active_leg_index_ = static_cast<int>(leg_index);
        no_odom_probe_active_leg_extension_s_ = no_odom_probe_leg_extensions_s_[leg_index];
        // Do not record a range slope while the requested leg has not yet
        // produced physical XY movement. The leg remains active so it can
        // become observable without relabeling the sample as another axis.
        no_odom_probe_sample_enabled_ =
            remaining >= no_odom_probe_sample_delay_s_ && motion_response_moving(SteadyClock::now());
        if (axis == 1) return {no_odom_probe_scale_, 0.0, 0.0, 0.0};
        if (axis == -1) return {-no_odom_probe_scale_, 0.0, 0.0, 0.0};
        if (axis == 2) return {0.0, no_odom_probe_scale_, 0.0, 0.0};
        if (axis == -2) return {0.0, -no_odom_probe_scale_, 0.0, 0.0};
        if (axis == 3) return {0.0, 0.0, no_odom_probe_heave_scale_, 0.0};
        return {0.0, 0.0, -no_odom_probe_heave_scale_, 0.0};
      }
      // Feedback says that the commanded leg stayed nearly stationary. Give
      // the current body axis more time instead of advancing to a nominally
      // different probe direction with no usable displacement. Stale/missing
      // feedback keeps the original validated wall-clock timing.
      if (motion_response_enabled_ && motion_response_feedback_fresh(SteadyClock::now()) &&
          !motion_response_moving(SteadyClock::now()) &&
          no_odom_probe_leg_extensions_s_[leg_index] <
              motion_response_probe_max_extension_s_) {
        const double extension = std::min(
            motion_response_probe_extension_s_,
            motion_response_probe_max_extension_s_ -
                no_odom_probe_leg_extensions_s_[leg_index]);
        no_odom_probe_leg_extensions_s_[leg_index] += extension;
        no_odom_probe_axis_ = axis;
        no_odom_probe_active_leg_index_ = static_cast<int>(leg_index);
        no_odom_probe_active_leg_extension_s_ = no_odom_probe_leg_extensions_s_[leg_index];
        no_odom_probe_sample_enabled_ = false;
        RCLCPP_DEBUG(
            get_logger(),
            "Phase probe leg %zu axis=%d has %.4f m/s XY response; extending %.2fs (total %.2fs)",
            leg_index, axis, *motion_response_horizontal_speed_mps_, extension,
            no_odom_probe_active_leg_extension_s_);
        if (axis == 1) return {no_odom_probe_scale_, 0.0, 0.0, 0.0};
        if (axis == -1) return {-no_odom_probe_scale_, 0.0, 0.0, 0.0};
        if (axis == 2) return {0.0, no_odom_probe_scale_, 0.0, 0.0};
        if (axis == -2) return {0.0, -no_odom_probe_scale_, 0.0, 0.0};
        if (axis == 3) return {0.0, 0.0, no_odom_probe_heave_scale_, 0.0};
        return {0.0, 0.0, -no_odom_probe_heave_scale_, 0.0};
      }
      remaining -= leg_duration;
      if (remaining < no_odom_probe_neutral_s_) {
        // The tail of each neutral gap supplies u=0 observations for the
        // robust clock/current drift fit.  Its initial momentum-decay portion
        // is discarded by the same bounded transient rule as active legs.
        const double neutral_delay = std::min(
            no_odom_probe_sample_delay_s_, 0.60 * no_odom_probe_neutral_s_);
        no_odom_probe_sample_enabled_ = remaining >= neutral_delay;
        return {};
      }
      remaining -= no_odom_probe_neutral_s_;
    }
    complete = true;
    return {};
  }

  void start_no_odom_probe(const SteadyTime &now) {
    no_odom_probe_delta_sum_.fill(0.0);
    no_odom_probe_delta_count_.fill(0U);
    no_odom_neutral_delta_sum_ = 0.0;
    no_odom_neutral_delta_count_ = 0U;
    no_odom_phase_samples_.clear();
    no_odom_probe_fit_rms_m_ = std::numeric_limits<double>::infinity();
    no_odom_probe_fit_inlier_ratio_ = 0.0;
    no_odom_probe_axis_ = 0;
    no_odom_probe_active_leg_index_ = -1;
    no_odom_probe_active_leg_extension_s_ = 0.0;
    no_odom_probe_leg_extensions_s_.fill(0.0);
    no_odom_probe_sample_enabled_ = false;
    no_odom_forward_started_.reset();
    no_odom_forward_expected_delta_m_ = std::numeric_limits<double>::quiet_NaN();
    reset_no_odom_approach_innovation();
    reset_motion_response();
    filtered_direction_body_.reset();
    probe_completed_ = false;
    if (state_ == "NO_ODOM_PHASE_PROBE") {
      state_started_ = now;
    } else {
      transition("NO_ODOM_PHASE_PROBE");
    }
  }

  bool finish_no_odom_probe(const SteadyTime &now) {
    const auto mean_for_axis = [this](int axis) -> std::optional<double> {
      const std::size_t index = static_cast<std::size_t>(axis + 3);
      if (index >= no_odom_probe_delta_sum_.size() ||
          no_odom_probe_delta_count_[index] <
              static_cast<std::size_t>(2 * no_odom_min_samples_per_leg_)) {
        return std::nullopt;
      }
      return no_odom_probe_delta_sum_[index] /
          static_cast<double>(no_odom_probe_delta_count_[index]);
    };
    const auto forward_plus = mean_for_axis(1);
    const auto forward_minus = mean_for_axis(-1);
    const auto lateral_plus = mean_for_axis(2);
    const auto lateral_minus = mean_for_axis(-2);
    const auto heave_plus = mean_for_axis(3);
    const auto heave_minus = mean_for_axis(-3);
    if (!forward_plus || !forward_minus || !lateral_plus || !lateral_minus ||
        (!no_odom_horizontal_only_ && (!heave_plus || !heave_minus))) {
      RCLCPP_WARN(
          get_logger(),
          "no-odom Phase probe rejected: insufficient delta-range samples per leg");
      return false;
    }

    // Fit delta_r = b0 + b1*t + beta_x*u_x + beta_y*u_y (+ beta_z*u_z).
    // The desired source score is -beta: travelling toward the source makes
    // range decrease.  ABBA makes the command columns orthogonal to a linear
    // drift in the ideal case; Huber IRLS then prevents an occasional Phase
    // unwrap/outlier from deciding the bearing.
    const std::size_t sample_count = no_odom_phase_samples_.size();
    if (sample_count < 12U) {
      RCLCPP_WARN(
          get_logger(),
          "no-odom Phase probe rejected: only %zu transient-filtered observations",
          sample_count);
      return false;
    }
    double mean_time = 0.0;
    double min_time = std::numeric_limits<double>::infinity();
    double max_time = -std::numeric_limits<double>::infinity();
    for (const auto &sample : no_odom_phase_samples_) {
      mean_time += sample.probe_s;
      min_time = std::min(min_time, sample.probe_s);
      max_time = std::max(max_time, sample.probe_s);
    }
    mean_time /= static_cast<double>(sample_count);
    const double time_scale = std::max(1.0, max_time - min_time);
    const Eigen::Index motion_axes = no_odom_horizontal_only_ ? 2 : 3;
    Eigen::MatrixXd design(static_cast<Eigen::Index>(sample_count), 2 + motion_axes);
    Eigen::VectorXd observation(static_cast<Eigen::Index>(sample_count));
    for (std::size_t i = 0; i < sample_count; ++i) {
      const auto &sample = no_odom_phase_samples_[i];
      design(static_cast<Eigen::Index>(i), 0) = 1.0;
      design(static_cast<Eigen::Index>(i), 1) =
          (sample.probe_s - mean_time) / time_scale;
      design(static_cast<Eigen::Index>(i), 2) = sample.command_body.x();
      design(static_cast<Eigen::Index>(i), 3) = sample.command_body.y();
      if (!no_odom_horizontal_only_) {
        design(static_cast<Eigen::Index>(i), 4) = sample.command_body.z();
      }
      observation(static_cast<Eigen::Index>(i)) = sample.delta_range_m;
    }
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> observability_qr(design);
    if (observability_qr.rank() < 2 + motion_axes) {
      RCLCPP_WARN(
          get_logger(),
          "no-odom Phase probe rejected: command/drift regression rank=%ld",
          static_cast<long>(observability_qr.rank()));
      return false;
    }

    Eigen::VectorXd weights = Eigen::VectorXd::Ones(
        static_cast<Eigen::Index>(sample_count));
    Eigen::VectorXd fit = observability_qr.solve(observation);
    for (int iteration = 0; iteration < 4; ++iteration) {
      const Eigen::VectorXd residual = observation - design * fit;
      std::vector<double> residual_values;
      residual_values.reserve(sample_count);
      for (Eigen::Index i = 0; i < residual.size(); ++i) {
        residual_values.push_back(residual(i));
      }
      const double residual_median = kmu26::pinger_homing::median(residual_values);
      std::vector<double> absolute_deviation;
      absolute_deviation.reserve(sample_count);
      for (const double value : residual_values) {
        absolute_deviation.push_back(std::abs(value - residual_median));
      }
      const double robust_sigma = std::max(
          1.0e-7,
          1.4826 * kmu26::pinger_homing::median(absolute_deviation));
      const double huber_threshold = no_odom_probe_huber_k_ * robust_sigma;
      Eigen::MatrixXd weighted_design = design;
      Eigen::VectorXd weighted_observation = observation;
      for (Eigen::Index i = 0; i < residual.size(); ++i) {
        const double centered_residual = std::abs(residual(i) - residual_median);
        weights(i) = centered_residual <= huber_threshold
            ? 1.0 : huber_threshold / std::max(centered_residual, 1.0e-12);
        const double root_weight = std::sqrt(weights(i));
        weighted_design.row(i) *= root_weight;
        weighted_observation(i) *= root_weight;
      }
      fit = weighted_design.colPivHouseholderQr().solve(weighted_observation);
    }
    if (!fit.allFinite()) {
      RCLCPP_WARN(get_logger(), "no-odom Phase probe rejected: non-finite robust fit");
      return false;
    }
    const Eigen::VectorXd residual = observation - design * fit;
    no_odom_probe_fit_rms_m_ = std::sqrt(residual.squaredNorm() /
        static_cast<double>(sample_count));
    no_odom_probe_fit_inlier_ratio_ =
        static_cast<double>((weights.array() >= 0.99).count()) /
        static_cast<double>(sample_count);
    Eigen::Vector3d score = Eigen::Vector3d::Zero();
    score.head(motion_axes) = -fit.segment(2, motion_axes);
    no_odom_probe_score_body_ = score;
    if (!score.allFinite() || score.head<2>().norm() < no_odom_min_horizontal_signal_) {
      RCLCPP_WARN(
          get_logger(),
          "no-odom Phase probe rejected: weak horizontal signal=(%.6g,%.6g,%.6g)",
          score.x(), score.y(), score.z());
      return false;
    }
    const auto quaternion = attitude_quaternion(now);
    if (!quaternion) return false;
    const Eigen::Vector3d body_direction = score.normalized();
    // After yaw has aligned this world bearing to the vehicle's X axis, the
    // expected per-window phase displacement for a forward command is the
    // horizontal ABBA score magnitude times that command.  Moving toward the
    // source is negative delta-range by this package's convention.
    no_odom_forward_expected_delta_m_ =
        -score.head<2>().norm() * no_odom_forward_command_;
    no_odom_direction_world_ =
        (quaternion->toRotationMatrix() * body_direction).normalized();
    last_control_direction_source_ = "no_odom_phase_probe";
    probe_completed_ = true;
    ++no_odom_probe_count_;
    RCLCPP_INFO(
        get_logger(),
        "no-odom Phase ABBA bearing ready body=(%.3f,%.3f,%.3f) "
        "samples=(%zu,%zu,%zu,%zu,%zu,%zu) horizontal_only=%s fit_n=%zu rms=%.6g inliers=%.2f",
        body_direction.x(), body_direction.y(), body_direction.z(),
        no_odom_probe_delta_count_[4], no_odom_probe_delta_count_[2],
        no_odom_probe_delta_count_[5], no_odom_probe_delta_count_[1],
        no_odom_probe_delta_count_[6], no_odom_probe_delta_count_[0],
        no_odom_horizontal_only_ ? "true" : "false", sample_count,
        no_odom_probe_fit_rms_m_, no_odom_probe_fit_inlier_ratio_);
    return true;
  }

  bool confirm_initial_no_odom_bearing(const SteadyTime &now) {
    if (no_odom_initial_bearing_confirmed_) return true;
    if (!no_odom_direction_world_) return false;

    const Eigen::Vector3d current = no_odom_direction_world_->normalized();
    if (no_odom_initial_confirmation_probes_ <= 1) {
      no_odom_initial_candidate_world_ = current;
      no_odom_initial_consistent_count_ = 1U;
      no_odom_initial_confirmation_dot_ = 1.0;
      no_odom_initial_bearing_confirmed_ = true;
      return true;
    }

    if (!no_odom_initial_candidate_world_) {
      no_odom_initial_candidate_world_ = current;
      no_odom_initial_consistent_count_ = 1U;
      no_odom_initial_confirmation_dot_ =
          std::numeric_limits<double>::quiet_NaN();
      RCLCPP_INFO(
          get_logger(),
          "no-odom initial bearing candidate stored (1/%d); vehicle remains neutral",
          no_odom_initial_confirmation_probes_);
      start_no_odom_probe(now);
      return false;
    }

    no_odom_initial_confirmation_dot_ =
        no_odom_initial_candidate_world_->normalized().dot(current);
    const auto consensus = confirm_no_odom_bearing_pair(
        *no_odom_initial_candidate_world_, current,
        no_odom_initial_confirmation_dot_min_);
    if (!consensus) {
      RCLCPP_WARN(
          get_logger(),
          "no-odom initial bearing rejected: dot=%.3f < %.3f; "
          "replacing candidate and repeating ABBA while neutral",
          no_odom_initial_confirmation_dot_,
          no_odom_initial_confirmation_dot_min_);
      no_odom_initial_candidate_world_ = current;
      no_odom_initial_consistent_count_ = 1U;
      start_no_odom_probe(now);
      return false;
    }

    no_odom_initial_candidate_world_ = *consensus;
    no_odom_direction_world_ = *consensus;
    ++no_odom_initial_consistent_count_;
    if (no_odom_initial_consistent_count_ <
        static_cast<std::size_t>(no_odom_initial_confirmation_probes_)) {
      RCLCPP_INFO(
          get_logger(),
          "no-odom initial bearing consistent dot=%.3f (%zu/%d); "
          "vehicle remains neutral for another ABBA probe",
          no_odom_initial_confirmation_dot_,
          no_odom_initial_consistent_count_,
          no_odom_initial_confirmation_probes_);
      start_no_odom_probe(now);
      return false;
    }

    no_odom_initial_bearing_confirmed_ = true;
    RCLCPP_INFO(
        get_logger(),
        "no-odom initial bearing confirmed dot=%.3f probes=%zu",
        no_odom_initial_confirmation_dot_,
        no_odom_initial_consistent_count_);
    return true;
  }

  void no_odom_phase_tick(const SteadyTime &now) {
    if (!no_odom_phase_ready(now)) {
      no_odom_probe_axis_ = 0;
      no_odom_direction_world_.reset();
      no_odom_forward_started_.reset();
      no_odom_initial_candidate_world_.reset();
      no_odom_initial_consistent_count_ = 0U;
      no_odom_initial_confirmation_dot_ =
          std::numeric_limits<double>::quiet_NaN();
      no_odom_initial_bearing_confirmed_ = false;
      reset_motion_response();
      transition("WAIT_VEHICLE");
      publish_command(Command{});
      return;
    }
    if (!active_started_) active_started_ = now;
    if (state_ == "WAIT_VEHICLE") start_no_odom_probe(now);

    if (state_ == "NO_ODOM_PHASE_PROBE") {
      bool complete = false;
      const Command command = no_odom_probe_command(
          seconds_since(state_started_, now), complete);
      if (!complete) {
        publish_command(command);
        return;
      }
      no_odom_probe_axis_ = 0;
      publish_command(Command{});
      if (!finish_no_odom_probe(now)) {
        start_no_odom_probe(now);
        return;
      }
      if (!confirm_initial_no_odom_bearing(now)) return;
      transition("ALIGN");
    }

    const auto quaternion = attitude_quaternion(now);
    if (!quaternion || !no_odom_direction_world_) {
      start_no_odom_probe(now);
      publish_command(Command{});
      return;
    }
    const Eigen::Vector3d body =
        (quaternion->inverse().toRotationMatrix() * *no_odom_direction_world_).normalized();
    publish_direction(body);
    publish_control_direction(body);
    last_control_direction_source_ = "no_odom_phase_probe";
    if (maybe_complete(now)) {
      publish_success_terminal_command(now);
      return;
    }

    const double bearing = std::atan2(body.y(), body.x());
    last_bearing_rad_ = bearing;
    double yaw = kmu26::pinger_homing::stabilized_yaw_command(
        bearing, current_yaw_rate(now), last_command_.yaw, control_period_s_,
        yaw_gain_, yaw_rate_damping_, yaw_deadband_rad_, yaw_command_limit_,
        yaw_slew_rate_);
    double heave = no_odom_vertical_control_enabled_
        ? clamp(heave_gain_ * body.z(), -no_odom_heave_limit_, no_odom_heave_limit_)
        : 0.0;
    double forward = 0.0;
    if (state_ == "ALIGN") {
      if (std::abs(bearing) <= align_exit_rad_) {
        transition("APPROACH");
        no_odom_forward_started_ = now;
        reset_no_odom_approach_innovation();
        forward = std::min(forward_max_, no_odom_forward_command_);
      }
    } else if (std::abs(bearing) > align_enter_rad_) {
      transition("ALIGN");
      no_odom_forward_started_.reset();
    } else {
      if (!no_odom_forward_started_) no_odom_forward_started_ = now;
      if (no_odom_reestimate_policy_ == "fixed" &&
          seconds_since(*no_odom_forward_started_, now) >= no_odom_forward_duration_s_) {
        // Compatibility mode for a deliberately fixed cadence.  The normal
        // profile uses the innovation/response gates below instead.
        start_no_odom_probe(now);
        publish_command(Command{});
        return;
      }
      transition("APPROACH");
      forward = std::min(forward_max_, no_odom_forward_command_);
    }
    const Command approach_command{forward, 0.0, heave, yaw};
    if (handle_no_odom_motion_response(now, approach_command)) return;
    if (handle_no_odom_phase_innovation(now)) return;
    if (handle_no_odom_approach_watchdog(now)) return;
    publish_command(approach_command);
  }

  void direct_direction_tick(const SteadyTime &now) {
    if ((!dry_run_ && !live_control_ready(now)) || !position_ ||
        seconds_since(last_odometry_, now) >= odometry_timeout_s_) {
      transition("WAIT_VEHICLE");
      publish_command(Command{});
      return;
    }
    if (!direction_usable(now)) {
      transition("WAIT_DIRECTION");
      publish_command(Command{});
      return;
    }
    transition("APPROACH");
    approach_command(now);
  }

  bool ready(const SteadyTime &now) const {
    return (dry_run_ || live_control_ready(now)) && position_ &&
           seconds_since(last_odometry_, now) < odometry_timeout_s_ &&
           audio_ready_for_probe(now);
  }

  bool vehicle_state_fresh(const SteadyTime &now) const {
    return last_vehicle_state_ != SteadyTime{} &&
           seconds_since(last_vehicle_state_, now) < vehicle_state_timeout_s_;
  }

  bool vehicle_mode_ready(const SteadyTime &now) const {
    return vehicle_state_fresh(now) && connected_ && vehicle_mode_matches_required();
  }

  bool vehicle_mode_matches_required() const {
    if (actual_vehicle_mode_ == mode_) return true;
    // MAVROS on the MuJoCo/SITL minimal ArduSub plugin list reports custom
    // mode 19 as the raw name below, while a real vehicle reports ALT_HOLD.
    // Treat this as a simulator display alias only; the requested flight mode
    // remains ALT_HOLD and no broader mode is accepted.
    return mode_ == "ALT_HOLD" && actual_vehicle_mode_ == "CMODE(19)";
  }

  bool live_control_ready(const SteadyTime &now) const {
    return vehicle_mode_ready(now) && armed_;
  }

  bool connection_grace_active(const SteadyTime &now) const {
    return vehicle_state_fresh(now) && vehicle_disconnect_grace_s_ > 0.0 && !connected_ &&
           seconds_since(last_connected_, now) <= vehicle_disconnect_grace_s_;
  }

  bool vehicle_armed_effective(const SteadyTime &now) const {
    if (!vehicle_state_fresh(now)) return false;
    if (connected_) return armed_;
    return connection_grace_active(now) &&
           seconds_since(last_armed_, now) <= vehicle_disconnect_grace_s_;
  }

  bool direction_usable(const SteadyTime &now) const {
    return phase_direction_world_ &&
           seconds_since(last_direction_, now) < direction_timeout_s_ &&
           phase_direction_world_->head<2>().norm() > 0.05;
  }

  Command probe_command(const SteadyTime &now, bool mirrored) {
    if (fast_probe_enabled_) {
      const double elapsed = seconds_since(state_started_, now);
      const double sign = mirrored ? -1.0 : 1.0;
      const double probe_duration = force_full_reprobe_
          ? range_feedback_probe_duration_s_ : fast_probe_duration_s_;
      if (elapsed < 0.6) return {};
      if (elapsed < probe_duration) {
        // A bounded helical excitation supplies X/Y/Z observability in one
        // motion instead of serial 12 s axes.  The acoustic estimators are
        // unchanged; this is purely the RC trajectory used to excite them.
        return {
            fast_probe_forward_, sign * fast_probe_lateral_, fast_probe_heave_,
            sign * fast_probe_yaw_};
      }
      const bool full_feedback_probe = force_full_reprobe_;
      const bool feedback_ready = update_range_guidance(now, full_feedback_probe);
      update_acoustic_position_guidance(now, true);
      force_full_reprobe_ = false;
      if (full_feedback_probe) range_feedback_probe_completed_ = true;
      if (full_feedback_probe && !feedback_ready) {
        range_gradient_recovery_active_ = false;
        RCLCPP_WARN(
            get_logger(),
            "full feedback probe ended without an observable 2-D amplitude gradient");
      }
      const bool fresh_phase_fit = acoustic_estimator_mode_ != "phase" ||
          maybe_fit_source(now, true);
      const bool estimator_ready = direction_usable(now) ||
          (acoustic_estimator_mode_ == "phase" && fresh_phase_fit && estimate_usable());
      if (estimator_ready) {
        probe_completed_ = true;
        transition("ALIGN");
      } else if (probe_attempt_ + 1 < minimum_probe_legs_) {
        ++probe_attempt_;
        transition(mirrored ? "PROBE" : "REPROBE");
      } else if (probe_attempt_ < 2) {
        ++probe_attempt_;
        transition(mirrored ? "PROBE" : "REPROBE");
      } else {
        transition("FAILED_ESTIMATE");
      }
      return {};
    }
    const auto probe = kmu26::pinger_homing::legacy_python_probe_command(
        seconds_since(state_started_, now), probe_duration_scale_, probe_scale_,
        probe_heave_for_state_, mirrored);
    if (!probe.complete) {
      return {probe.forward, probe.lateral, probe.heave, probe.yaw};
    }

    // Only the fit produced at this completed probe boundary may authorize a
    // lock.  A cached periodic estimate can remain usable after this force-fit
    // is rejected; promoting that stale value caused a rejected boundary fit
    // to transition directly into ALIGN/APPROACH.
    const bool force_fit_result = maybe_fit_source(now, true);
    update_range_guidance(now, force_full_reprobe_);
    update_acoustic_position_guidance(now, true);
    force_full_reprobe_ = false;
    const bool completed_fit_can_lock =
        kmu26::pinger_homing::completed_probe_fit_can_lock(
            force_fit_result, last_force_fit_accepted_, estimate_usable());
    if (completed_fit_can_lock) {
      // The validated Python Phase sequence always honors the configured
      // deep-tank leg count, including a near-field reprobe.  Reducing a
      // configured two-leg solve to one after a stall reintroduces the weak-Z
      // lock that the mirrored leg exists to remove.
      const int required_probe_legs = legacy_python_sequence_
          ? minimum_probe_legs_
          : kmu26::pinger_homing::effective_minimum_probe_legs(
                minimum_probe_legs_, near_reprobe_count_);
      if (probe_attempt_ + 1 < required_probe_legs) {
        ++probe_attempt_;
        transition(mirrored ? "PROBE" : "REPROBE");
        return {};
      }
      // A range-difference fit has no absolute-range observation.  With only
      // the first excitation it can occasionally select a mathematically
      // consistent near branch and declare arrival before any approach.  The
      // successful Python sequence already has a mirrored probe; request that
      // independent geometry only when an uncalibrated first fit claims the
      // source is unusually near.  Far, well-observed fits retain the one-leg
      // fast path.
      const std::optional<double> candidate_distance =
          position_ && source_smoothed_
              ? std::optional<double>((*source_smoothed_ - *position_).norm())
              : std::nullopt;
      const bool near_first_fit_needs_confirmation =
          legacy_python_sequence_ && success_range_m_ <= 0.0 &&
          probe_attempt_ == 0 && candidate_distance &&
          std::isfinite(*candidate_distance) &&
          *candidate_distance <= first_lock_confirmation_radius_m_;
      if (near_first_fit_needs_confirmation) {
        RCLCPP_WARN(
            get_logger(),
            "uncalibrated first source fit is near (%.3fm <= %.3fm); "
            "requiring mirrored confirmation before lock",
            *candidate_distance, first_lock_confirmation_radius_m_);
        ++probe_attempt_;
        source_estimate_.reset();
        source_smoothed_.reset();
        transition("REPROBE");
        return {};
      }
      if (!source_locked_ && source_smoothed_) {
        source_locked_ = source_smoothed_;
        const double fit_rms = source_estimate_
            ? source_estimate_->rms_residual_m : std::numeric_limits<double>::quiet_NaN();
        const double fit_condition = source_estimate_
            ? source_estimate_->condition_number : std::numeric_limits<double>::quiet_NaN();
        const double fit_initial_range = source_estimate_
            ? source_estimate_->initial_range_m : std::numeric_limits<double>::quiet_NaN();
        RCLCPP_INFO(
            get_logger(),
            "locked C++ pinger source at (%.3f, %.3f, %.3f) "
            "distance=%.3fm rms=%.6fm condition=%.3f initial_range=%.3fm samples=%zu",
            source_locked_->x(), source_locked_->y(), source_locked_->z(),
            candidate_distance.value_or(std::numeric_limits<double>::quiet_NaN()),
            fit_rms, fit_condition, fit_initial_range,
            source_estimate_ ? source_estimate_->sample_count : 0U);
      }
      probe_completed_ = true;
      transition("ALIGN");
    } else if (!legacy_python_sequence_ &&
               probe_attempt_ + 1 >= minimum_probe_legs_ && direction_usable(now)) {
      RCLCPP_WARN(
          get_logger(),
          "range position is not yet trustworthy; continuing with fresh hydrophone direction");
      probe_completed_ = true;
      transition("ALIGN");
    } else if (probe_attempt_ < 3) {
      ++probe_attempt_;
      transition(mirrored ? "PROBE" : "REPROBE");
    } else {
      transition("FAILED_ESTIMATE");
    }
    return {};
  }

  bool maybe_fit_source(const SteadyTime &now, bool force) {
    if (force) last_force_fit_accepted_ = false;
    if (!force && seconds_since(last_fit_, now) < source_fit_period_s_) return false;
    last_fit_ = now;
    if (samples_.size() < 30U) return false;

    std::vector<RangeSample> selected;
    const std::size_t maximum_fit_samples = legacy_python_sequence_ ? 360U : 180U;
    if (samples_.size() <= maximum_fit_samples) {
      selected.assign(samples_.begin(), samples_.end());
    } else {
      selected.reserve(maximum_fit_samples);
      for (std::size_t i = 0; i < maximum_fit_samples; ++i) {
        const std::size_t index =
            i * (samples_.size() - 1U) / (maximum_fit_samples - 1U);
        selected.push_back(samples_[index]);
      }
    }
    std::vector<Eigen::Vector3d> positions;
    std::vector<double> changes;
    std::vector<double> times;
    std::vector<double> ranges;
    positions.reserve(selected.size());
    changes.reserve(selected.size());
    times.reserve(selected.size());
    ranges.reserve(selected.size());
    for (const auto &sample : selected) {
      positions.push_back(sample.position_world);
      changes.push_back(sample.cumulative_change_m);
      times.push_back(sample.wall_s);
      ranges.push_back(sample.amplitude_range_m);
    }

    std::optional<Eigen::Vector3d> seed = source_smoothed_;
    if (!seed && phase_direction_world_ &&
        (!legacy_python_sequence_ || !auto_source_depth_)) {
      const double seed_range = amplitude_range().value_or(15.0);
      Eigen::Vector3d direction = *phase_direction_world_;
      if (!auto_source_depth_) {
        direction.z() = -std::max(std::abs(direction.z()), 0.45);
      }
      if (direction.norm() > 1.0e-9) {
        seed = positions.front() + seed_range * direction.normalized();
      }
    }
    const std::optional<double> min_z = auto_source_depth_
        ? std::optional<double>(-tank_max_depth_m_) : std::nullopt;
    const std::optional<double> max_z = auto_source_depth_
        ? std::optional<double>(max_source_z_world_) : std::nullopt;
    const std::optional<double> fixed_z = pinger_expected_depth_m_ > 0.0
        ? std::optional<double>(-pinger_expected_depth_m_) : std::nullopt;
    // Preserve the validated Python estimator order.  An explicitly known
    // pinger depth uses calibrated absolute ranges for XY first.  Tank-only
    // Phase mode leaves fixed_z empty and therefore uses the bounded 3-D
    // range-difference fit, exactly like the archived Python controller.
    std::optional<SourceEstimate> estimate;
    std::optional<SourceEstimate> gradient_seed;
    if (fixed_z) {
      gradient_seed = kmu26::pinger_homing::estimate_source_from_range_gradient(
          positions, ranges, *fixed_z);
      std::optional<Eigen::Vector3d> absolute_seed;
      if (gradient_seed) {
        absolute_seed = gradient_seed->source_world;
      } else if (!source_locked_ && seed) {
        absolute_seed = seed;
      }
      estimate = kmu26::pinger_homing::estimate_source_xy_from_absolute_ranges(
          positions, ranges, *fixed_z, absolute_seed);
    }
    if (!estimate) {
      estimate = kmu26::pinger_homing::estimate_source_from_range_differences(
          positions, changes, times, seed, ranges, min_z, max_z, fixed_z);
    }
    if (!estimate) {
      if (force) {
        RCLCPP_WARN(
            get_logger(),
            "C++ pinger source fit rejected samples=%zu reason=no_solution",
            selected.size());
      }
      return false;
    }
    const bool inside_bounds = source_inside_bounds(estimate->source_world);
    const bool quality_ok = estimate_quality(*estimate);
    if (!inside_bounds || !quality_ok) {
      if (force) {
        RCLCPP_WARN(
            get_logger(),
            "C++ pinger source fit rejected samples=%zu reason=%s%s "
            "source=(%.3f,%.3f,%.3f) phase_rms=%.6f condition=%.3f "
            "absolute_median=%.6f absolute_rms=%.6f latest_absolute_error=%.6f "
            "xy_bounds_enabled=%s",
            selected.size(), inside_bounds ? "" : "bounds",
            quality_ok ? "" : (inside_bounds ? "quality" : "+quality"),
            estimate->source_world.x(), estimate->source_world.y(),
            estimate->source_world.z(), estimate->rms_residual_m,
            estimate->condition_number, estimate->absolute_median_residual_m,
            estimate->absolute_rms_residual_m, estimate->latest_absolute_error_m,
            source_xy_bounds_enabled_ ? "true" : "false");
      }
      return false;
    }
    const bool phase_can_disambiguate = phase_direction_world_ &&
        (legacy_python_sequence_ ||
         (direction_update_count_ >= 3U && direction_usable(now)));
    if (!source_locked_ && phase_can_disambiguate) {
      const Eigen::Vector2d candidate =
          estimate->source_world.head<2>() -
          (legacy_python_sequence_ ? positions.front().head<2>() : positions.back().head<2>());
      const Eigen::Vector2d phase = phase_direction_world_->head<2>();
      if (candidate.norm() > 1.0e-6 && phase.norm() > 1.0e-6 &&
          candidate.normalized().dot(phase.normalized()) < source_phase_alignment_min_) {
        if (force) {
          RCLCPP_WARN(
              get_logger(),
              "C++ pinger source fit rejected reason=phase_xy_alignment "
              "source=(%.3f,%.3f,%.3f) alignment=%.3f minimum=%.3f",
              estimate->source_world.x(), estimate->source_world.y(),
              estimate->source_world.z(),
              candidate.normalized().dot(phase.normalized()),
              source_phase_alignment_min_);
        }
        return false;
      }
      // The horizontal Phase direction resolves the planar mirror branch.
      // When its vertical component is observable, also reject a candidate on
      // the opposite side of the vehicle (the recurrent near-surface mirror
      // for a pinger that the Phase estimator says is below the robot).
      if (phase_vertical_disambiguation_enabled_ && auto_source_depth_ &&
          std::abs(phase_direction_world_->z()) >= 0.08) {
        const double candidate_dz = estimate->source_world.z() - positions.front().z();
        if (std::abs(candidate_dz) >= 0.20 &&
            candidate_dz * phase_direction_world_->z() < 0.0) {
          if (force) {
            RCLCPP_WARN(
                get_logger(),
                "C++ pinger source fit rejected reason=phase_z_sign "
                "source_z=%.3f reference_z=%.3f phase_z=%.3f",
                estimate->source_world.z(), positions.front().z(),
                phase_direction_world_->z());
          }
          return false;
        }
      }
    }
    source_estimate_ = estimate;
    if (!source_smoothed_) {
      source_smoothed_ = estimate->source_world;
    } else {
      source_smoothed_ = 0.82 * *source_smoothed_ + 0.18 * estimate->source_world;
    }
    // The Python controller deliberately refines its locked source in the
    // near field.  Keep the source stationary while far away, then blend only
    // after calibrated amplitude range is below 6 m.
    const auto current_range = amplitude_range();
    if (legacy_python_sequence_ && source_locked_ && current_range &&
        *current_range < 6.0) {
      *source_locked_ = 0.65 * *source_locked_ + 0.35 * estimate->source_world;
    }
    if (force) last_force_fit_accepted_ = true;
    return true;
  }

  bool estimate_quality(const SourceEstimate &estimate) const {
    bool absolute_consistent = true;
    if (!legacy_python_sequence_ && estimate.absolute_sample_count >= 8U) {
      const double latest_limit = std::max(
          source_absolute_latest_margin_m_,
          source_absolute_latest_ratio_ * estimate.latest_absolute_range_m);
      absolute_consistent =
          estimate.absolute_median_residual_m <= source_absolute_median_limit_m_ &&
          estimate.absolute_rms_residual_m <= source_absolute_rms_limit_m_ &&
          estimate.latest_absolute_error_m <= latest_limit;
    }
    return absolute_consistent && estimate.sample_count >= 30U &&
           estimate.rms_residual_m < 0.30 &&
           estimate.condition_number < 2.0e5 && estimate.initial_range_m > 0.1 &&
           estimate.initial_range_m < 80.0;
  }

  bool estimate_usable() const {
    return source_estimate_ && source_smoothed_ && estimate_quality(*source_estimate_) &&
           source_inside_bounds(*source_smoothed_);
  }

  bool source_inside_bounds(const Eigen::Vector3d &source) const {
    const double boundary_margin = legacy_python_sequence_
        ? 0.0 : std::min(0.10, 0.1 * tank_max_depth_m_);
    const bool z_valid = !auto_source_depth_ ||
        (source.z() >= -tank_max_depth_m_ + boundary_margin &&
         source.z() <= max_source_z_world_ - boundary_margin);
    const bool xy_valid = !source_xy_bounds_enabled_ ||
        (source.x() >= source_min_x_m_ && source.x() <= source_max_x_m_ &&
         source.y() >= source_min_y_m_ && source.y() <= source_max_y_m_);
    return source.allFinite() && xy_valid && z_valid;
  }

  bool range_guidance_usable(const SteadyTime &now) const {
    return range_gradient_enabled_ && range_guidance_world_ &&
           range_guidance_world_->allFinite() &&
           range_guidance_world_->head<2>().norm() > 0.5 &&
           seconds_since(last_range_guidance_, now) < range_gradient_max_age_s_;
  }

  bool update_range_guidance(const SteadyTime &now, bool force) {
    if (!range_gradient_enabled_) return false;
    if (!force && seconds_since(last_range_gradient_fit_, now) < 0.40) {
      return range_guidance_usable(now);
    }
    last_range_gradient_fit_ = now;
    const double window_start = seconds_from_epoch(now) - range_gradient_window_s_;
    std::vector<const RangeSample *> candidates;
    candidates.reserve(range_feedback_samples_.size());
    for (const auto &sample : range_feedback_samples_) {
      if (sample.wall_s >= window_start && std::isfinite(sample.amplitude_range_m) &&
          sample.position_world.allFinite()) {
        candidates.push_back(&sample);
      }
    }
    if (candidates.size() < 8U) return range_guidance_usable(now);

    constexpr std::size_t kMaximumFeedbackSamples = 180U;
    const std::size_t count = std::min(candidates.size(), kMaximumFeedbackSamples);
    std::vector<Eigen::Vector3d> positions;
    std::vector<double> ranges;
    std::vector<double> times;
    positions.reserve(count);
    ranges.reserve(count);
    times.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t index = count == 1U
          ? 0U : i * (candidates.size() - 1U) / (count - 1U);
      positions.push_back(candidates[index]->position_world);
      ranges.push_back(candidates[index]->amplitude_range_m);
      times.push_back(candidates[index]->wall_s);
    }
    const std::optional<double> source_z = pinger_expected_depth_m_ > 0.0
        ? std::optional<double>(-pinger_expected_depth_m_) : std::nullopt;
    const auto estimate = kmu26::pinger_homing::estimate_amplitude_range_descent(
        positions, ranges, times, source_z, range_gradient_window_s_,
        range_gradient_min_span_m_);
    if (!estimate) return range_guidance_usable(now);

    range_gradient_slope_ = estimate->signed_slope_m_per_m;
    range_gradient_span_m_ = estimate->horizontal_span_m;
    range_gradient_rms_m_ = estimate->rms_residual_m;
    range_gradient_observability_ = estimate->observability_ratio;
    range_gradient_sample_count_ = estimate->sample_count;
    range_gradient_two_axis_ = estimate->two_axis_observable;
    if ((range_guidance_require_two_axis_ && !estimate->two_axis_observable) ||
        estimate->rms_residual_m > range_gradient_max_rms_m_) {
      return range_guidance_usable(now);
    }

    Eigen::Vector2d candidate = estimate->direction_world.head<2>();
    if (candidate.norm() <= 1.0e-9) return range_guidance_usable(now);
    candidate.normalize();
    if (range_guidance_world_) {
      Eigen::Vector2d current = range_guidance_world_->head<2>();
      if (current.norm() > 1.0e-9) {
        current.normalize();
        const double signed_angle = std::atan2(
            current.x() * candidate.y() - current.y() * candidate.x(),
            current.dot(candidate));
        const auto measured_range = amplitude_range();
        const bool regressing = measured_range && std::isfinite(best_amplitude_distance_) &&
            *measured_range > best_amplitude_distance_ + range_regression_margin_m_;
        const double step_limit = regressing
            ? std::max(range_guidance_max_step_rad_, radians(45.0))
            : range_guidance_max_step_rad_;
        const double step = clamp(signed_angle, -step_limit, step_limit);
        const double cs = std::cos(step);
        const double sn = std::sin(step);
        const Eigen::Vector2d slewed(
            cs * current.x() - sn * current.y(),
            sn * current.x() + cs * current.y());
        const double alpha = regressing
            ? std::max(range_guidance_filter_alpha_, 0.55)
            : range_guidance_filter_alpha_;
        candidate = ((1.0 - alpha) * current + alpha * slewed).normalized();
      }
    }
    range_guidance_world_ = Eigen::Vector3d(candidate.x(), candidate.y(), 0.0);
    last_range_guidance_ = now;
    if (force) range_gradient_recovery_active_ = true;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "amplitude range feedback ready direction=(%+.3f,%+.3f) span=%.3fm "
        "rms=%.3fm observability=%.3f samples=%zu",
        range_guidance_world_->x(), range_guidance_world_->y(),
        range_gradient_span_m_, range_gradient_rms_m_,
        range_gradient_observability_, range_gradient_sample_count_);
    return true;
  }

  bool acoustic_position_guidance_usable(const SteadyTime &now) const {
    if (!acoustic_position_enabled_ || !acoustic_source_xy_ || !position_ ||
        seconds_since(last_acoustic_position_fit_, now) > acoustic_position_max_age_s_) {
      return false;
    }
    if (const auto range = amplitude_range()) {
      const double dz = -pinger_expected_depth_m_ - position_->z();
      const double predicted = std::sqrt(
          (*acoustic_source_xy_ - position_->head<2>()).squaredNorm() + dz * dz);
      const double tolerance = std::max(
          acoustic_position_consistency_margin_m_,
          acoustic_position_consistency_ratio_ * *range);
      if (!std::isfinite(predicted) || std::abs(predicted - *range) > tolerance) {
        return false;
      }
    }
    return true;
  }

  bool update_acoustic_position_guidance(const SteadyTime &now, bool force) {
    if (!acoustic_position_enabled_) return false;
    if (!force && seconds_since(last_acoustic_position_attempt_, now) < 0.40) {
      return acoustic_position_guidance_usable(now);
    }
    last_acoustic_position_attempt_ = now;
    std::vector<Eigen::Vector3d> positions;
    std::vector<double> ranges;
    positions.reserve(range_feedback_samples_.size());
    ranges.reserve(range_feedback_samples_.size());
    for (const auto &sample : range_feedback_samples_) {
      if (!sample.position_world.allFinite() ||
          !std::isfinite(sample.amplitude_range_m)) {
        continue;
      }
      positions.push_back(sample.position_world);
      ranges.push_back(sample.amplitude_range_m);
    }
    const auto estimate = kmu26::pinger_homing::estimate_acoustic_source_xy(
        positions, ranges, -pinger_expected_depth_m_);
    if (!estimate) return acoustic_position_guidance_usable(now);
    acoustic_position_condition_ = estimate->condition_number;
    acoustic_position_median_residual_m_ = estimate->median_residual_m;
    acoustic_position_rms_residual_m_ = estimate->rms_residual_m;
    acoustic_position_sample_count_ = estimate->sample_count;
    const bool quality = estimate->condition_number <= acoustic_position_max_condition_ &&
        estimate->median_residual_m <= acoustic_position_max_median_residual_m_ &&
        estimate->rms_residual_m <= acoustic_position_max_rms_residual_m_;
    if (!quality) {
      acoustic_position_candidate_streak_ = 0;
      acoustic_position_candidate_xy_.reset();
      return acoustic_position_guidance_usable(now);
    }

    if (!acoustic_position_candidate_xy_ ||
        (estimate->source_xy_world - *acoustic_position_candidate_xy_).norm() > 0.75) {
      acoustic_position_candidate_xy_ = estimate->source_xy_world;
      acoustic_position_candidate_streak_ = 1;
      return acoustic_position_guidance_usable(now);
    }
    *acoustic_position_candidate_xy_ =
        0.60 * *acoustic_position_candidate_xy_ + 0.40 * estimate->source_xy_world;
    ++acoustic_position_candidate_streak_;
    if (acoustic_position_candidate_streak_ < 3) {
      return acoustic_position_guidance_usable(now);
    }
    acoustic_source_xy_ = acoustic_source_xy_
        ? 0.65 * *acoustic_source_xy_ + 0.35 * *acoustic_position_candidate_xy_
        : *acoustic_position_candidate_xy_;
    last_acoustic_position_fit_ = now;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "amplitude position feedback ready source=(%.3f,%.3f) condition=%.2f "
        "median=%.3fm rms=%.3fm samples=%zu",
        acoustic_source_xy_->x(), acoustic_source_xy_->y(),
        acoustic_position_condition_, acoustic_position_median_residual_m_,
        acoustic_position_rms_residual_m_, acoustic_position_sample_count_);
    return true;
  }

  void approach_command(const SteadyTime &now) {
    if (!position_) {
      publish_command(Command{});
      return;
    }
    const auto range = amplitude_range();
    bool position_source_consistent = true;
    if (!legacy_python_sequence_ && !prefer_direction_control_ && source_locked_ && range) {
      const double locked_distance = (*source_locked_ - *position_).norm();
      position_source_consistent =
          locked_distance + source_range_consistency_margin_m_ >=
          source_range_consistency_ratio_ * *range;
      if (!position_source_consistent && inconsistent_lock_reprobe_count_ < 2) {
        ++inconsistent_lock_reprobe_count_;
        RCLCPP_WARN(
            get_logger(),
            "discarding inconsistent source lock: fitted=%.3fm amplitude=%.3fm retry=%d/2",
            locked_distance, *range, inconsistent_lock_reprobe_count_);
        reset_localization_for_near_probe(now);
        publish_command(Command{});
        return;
      }
    }
    range_consistency_fallback_active_ = !position_source_consistent;
    update_range_guidance(now, false);
    update_acoustic_position_guidance(now, false);
    if (!continuous_range_excitation_enabled_ && range_gradient_enabled_ && range &&
        state_ == "APPROACH" &&
        *range <= range_gradient_takeover_m_ && !range_feedback_probe_completed_ &&
        !range_guidance_usable(now) && !acoustic_position_guidance_usable(now)) {
      range_feedback_probe_completed_ = true;
      force_full_reprobe_ = true;
      range_gradient_recovery_active_ = false;
      range_guidance_world_.reset();
      RCLCPP_INFO(
          get_logger(),
          "entered %.2fm near field at %.3fm; starting one bounded %.1fs 2-D "
          "range-feedback probe",
          range_gradient_takeover_m_, *range, range_feedback_probe_duration_s_);
      reset_localization_for_near_probe(now);
      publish_command(Command{});
      return;
    }
    if (legacy_python_sequence_ && range) {
      // Exact 83dafe8 Python progress/reprobe contract: only a 0.12 m
      // improvement resets the timer. After six seconds of APPROACH, eight
      // seconds without that progress discards the lock and repeats the
      // neutral-separated probe (at most twice).
      if (*range < best_amplitude_distance_ - range_progress_delta_m_) {
        best_amplitude_distance_ = *range;
        last_range_progress_ = now;
      } else if (state_ == "APPROACH" &&
                 seconds_since(state_started_, now) > 6.0 &&
                 seconds_since(last_range_progress_, now) > range_progress_timeout_s_ &&
                 near_reprobe_count_ < 2) {
        ++near_reprobe_count_;
        RCLCPP_WARN(
            get_logger(),
            "range progress stalled; stopping and probing again (%d/2)",
            near_reprobe_count_);
        reset_localization_for_near_probe(now);
        publish_command(Command{});
        return;
      }
    } else if (range) {
      if (!std::isfinite(best_amplitude_distance_) ||
          *range < best_amplitude_distance_ - 0.02) {
        if (!std::isfinite(best_amplitude_distance_) ||
            *range < best_amplitude_distance_ - range_progress_delta_m_) {
          last_range_progress_ = now;
        }
        best_amplitude_distance_ = *range;
        range_regression_started_.reset();
      } else if (!continuous_range_excitation_enabled_ &&
                 (state_ == "ALIGN" || state_ == "APPROACH" || state_ == "CONTACT") &&
                 best_amplitude_distance_ <= range_gradient_takeover_m_ &&
                 !acoustic_position_guidance_usable(now) &&
                 !range_guidance_usable(now) &&
                 *range > best_amplitude_distance_ + range_regression_margin_m_ &&
                 near_reprobe_count_ < 2) {
        if (!range_regression_started_) range_regression_started_ = now;
        if (seconds_since(*range_regression_started_, now) >= range_regression_hold_s_) {
          ++near_reprobe_count_;
          force_full_reprobe_ = true;
          range_gradient_recovery_active_ = false;
          range_guidance_world_.reset();
          RCLCPP_WARN(
              get_logger(),
              "amplitude range regressed from %.3fm to %.3fm for %.2fs; "
              "starting full 2-D feedback probe %d/2",
              best_amplitude_distance_, *range, range_regression_hold_s_,
              near_reprobe_count_);
          reset_localization_for_near_probe(now);
          publish_command(Command{});
          return;
        }
      } else if (*range <= best_amplitude_distance_ +
                              0.5 * range_regression_margin_m_) {
        range_regression_started_.reset();
      }
      if (!continuous_range_excitation_enabled_ && !range_regression_started_ &&
          state_ == "APPROACH" &&
          last_command_.forward > 0.08 &&
          best_amplitude_distance_ <= range_gradient_takeover_m_ &&
          !acoustic_position_guidance_usable(now) &&
          !range_guidance_usable(now) &&
          seconds_since(last_range_progress_, now) > range_progress_timeout_s_ &&
          near_reprobe_count_ < 2) {
        ++near_reprobe_count_;
        RCLCPP_WARN(
            get_logger(),
            "acoustic range made no %.2fm progress for %.1fs; starting bounded reprobe %d/2",
            range_progress_delta_m_, range_progress_timeout_s_, near_reprobe_count_);
        reset_localization_for_near_probe(now);
        publish_command(Command{});
        return;
      }
    }

    Eigen::Vector3d vector_world;
    std::string selected_control_source;
    const bool use_acoustic_position = acoustic_position_control_enabled_ &&
        acoustic_position_guidance_usable(now);
    const bool use_range_guidance = range && range_guidance_usable(now) &&
        (*range <= range_gradient_takeover_m_ || range_gradient_recovery_active_);
    if (use_range_guidance) {
      vector_world = *range_guidance_world_ * std::max(*range, 1.0);
      selected_control_source = "amplitude_gradient_guard";
    } else if (use_acoustic_position) {
      vector_world = Eigen::Vector3d(
          acoustic_source_xy_->x() - position_->x(),
          acoustic_source_xy_->y() - position_->y(),
          -pinger_expected_depth_m_ - position_->z());
      selected_control_source = "amplitude_multilateration_guard";
    } else if (prefer_direction_control_ && direction_usable(now)) {
      vector_world = phase_guidance_vector_world().value_or(
          phase_direction_world_->normalized() * range.value_or(unknown_range_m_));
      selected_control_source = acoustic_estimator_mode_ +
          (auto_source_depth_ ? "_direction+tank_depth" : "_direction");
    } else if (source_locked_ && position_source_consistent) {
      vector_world = *source_locked_ - *position_;
      selected_control_source = "range_localizer";
    } else if (direction_usable(now)) {
      vector_world = phase_guidance_vector_world().value_or(
          phase_direction_world_->normalized() *
          std::max(range.value_or(unknown_range_m_), 1.0));
      selected_control_source = acoustic_estimator_mode_ +
          (auto_source_depth_ ? "_direction+tank_depth" : "_direction");
    } else if (estimate_usable() && position_source_consistent) {
      vector_world = *source_smoothed_ - *position_;
      selected_control_source = "range_localizer";
    } else {
      if (seconds_since(state_started_, now) > 2.0 && controller_mode_ != "direction") {
        transition("REPROBE");
      }
      publish_command(Command{});
      return;
    }
    const std::optional<double> steering_range = amplitude_range();
    const bool phase_yaw_far_enough = !steering_range ||
        *steering_range > phase_yaw_min_range_m_;
    const bool apply_phase_yaw = phase_yaw_guidance_enabled_ &&
        phase_yaw_far_enough && direction_usable(now) &&
        selected_control_source == "range_localizer";
    if (apply_phase_yaw) selected_control_source = "range_localizer+phase_yaw";
    if (selected_control_source != last_control_direction_source_) {
      filtered_direction_body_.reset();
      last_control_direction_source_ = selected_control_source;
    }
    if (depth_target_control_ && vehicle_target_depth_m_ > 0.0) {
      vector_world.z() = -vehicle_target_depth_m_ - position_->z();
    }
    const double estimated_distance = vector_world.norm();
    const double distance = range.value_or(estimated_distance);
    if (!std::isfinite(distance) || distance > 80.0 || vector_world.norm() <= 1.0e-9) {
      if (controller_mode_ != "direction") transition("REPROBE");
      publish_command(Command{});
      return;
    }

    Eigen::Vector3d body_raw = kmu26::pinger_homing::world_vector_to_body_flu(
        vector_world, qx_, qy_, qz_, qw_).normalized();
    if (apply_phase_yaw) {
      const Eigen::Vector3d phase_body =
          kmu26::pinger_homing::world_vector_to_body_flu(
              phase_direction_world_->normalized(), qx_, qy_, qz_, qw_);
      body_raw = kmu26::pinger_homing::apply_acoustic_yaw_to_position_direction(
          body_raw, phase_body);
    }
    const Eigen::Vector3d body = kmu26::pinger_homing::filter_unit_vector(
        filtered_direction_body_, body_raw, direction_filter_alpha_);
    filtered_direction_body_ = body;
    publish_control_direction(body);
    const double bearing = std::atan2(body.y(), body.x());
    last_bearing_rad_ = bearing;
    double yaw = kmu26::pinger_homing::stabilized_yaw_command(
        bearing, yaw_rate_rad_s_, last_command_.yaw, control_period_s_, yaw_gain_,
        yaw_rate_damping_, yaw_deadband_rad_, yaw_command_limit_, yaw_slew_rate_);
    double heave = clamp(heave_gain_ * body.z(), -heave_limit_, heave_limit_);
    double lateral = 0.0;

    double forward = 0.0;
    // Range-gradient guidance must still honor the configured motion policy.
    // Previously merely selecting amplitude_gradient_guard forced full 6-DOF
    // vectoring even when direction_vectoring_enabled=false, so a 165 degree
    // bearing could command reverse surge and large sway instead of the
    // requested stop/turn/forward sequence.  A trusted metric position guide
    // may retain vectoring; the direction/range guide only vectors when the
    // profile explicitly enables it.
    const bool guided_vectoring =
        use_acoustic_position || direction_vectoring_enabled_;
    if (legacy_python_sequence_) {
      // Preserve the validated Python controller's threshold-only state
      // machine.  ALIGN exits as soon as the filtered bearing crosses the
      // exit threshold and commands forward motion in that same control
      // sample; it has no yaw-rate brake, settle timer, or hold interval.
      const double align_threshold =
          state_ == "ALIGN" ? align_exit_rad_ : align_enter_rad_;
      yaw_settle_started_.reset();
      if (std::abs(bearing) > align_threshold) {
        transition("ALIGN");
        forward = 0.0;
      } else {
        if (distance > 0.75) {
          transition("APPROACH");
          if (distance > 6.0) forward = forward_max_;
          else if (distance > 2.0) forward = std::min(forward_max_, forward_mid_);
          else forward = std::min(forward_max_, forward_near_);
        } else {
          transition("CONTACT");
          forward = std::min(forward_max_, forward_contact_);
          yaw = clamp(yaw, -0.25, 0.25);
          heave = clamp(heave, -0.20, 0.20);
        }
      }
    } else if (guided_vectoring) {
      transition(distance <= 0.75 ? "CONTACT" : "APPROACH");
      const double vector_scale = distance > 2.0
          ? std::min(forward_max_, range_gradient_forward_)
          : (distance > 0.75
              ? std::min(forward_max_, forward_near_)
              : std::min(forward_max_, forward_contact_));
      // The vehicle is vectored 6-DOF, so a trusted range guide can translate
      // along body X/Y immediately while yaw converges in parallel.  This
      // removes the near-field stop/rotate/drive cycle that dominated the
      // one-minute benchmark without changing either acoustic estimator.
      forward = clamp(vector_scale * body.x(), -vector_scale, vector_scale);
      lateral = clamp(
          vector_scale * acoustic_position_sway_gain_ * body.y(),
          -acoustic_position_sway_limit_, acoustic_position_sway_limit_);
      if (distance <= 0.75) {
        yaw = clamp(yaw, -0.25, 0.25);
        heave = clamp(heave, -0.20, 0.20);
      }
    } else if (state_ == "ALIGN") {
      const bool rotating_toward_target = bearing * yaw_rate_rad_s_ > 0.0;
      const double dynamic_brake_threshold = std::min(
          align_enter_rad_,
          std::max(align_exit_rad_,
                   std::abs(yaw_rate_rad_s_) * yaw_brake_horizon_s_));
      if (rotating_toward_target &&
          std::abs(bearing) <= dynamic_brake_threshold) {
        yaw = kmu26::pinger_homing::braking_yaw_command(
            yaw_rate_rad_s_, last_command_.yaw, control_period_s_,
            yaw_brake_gain_, yaw_command_limit_, yaw_brake_slew_rate_);
      }
      const bool settled = kmu26::pinger_homing::yaw_alignment_sample_settled(
          bearing, yaw_rate_rad_s_, align_exit_rad_, yaw_settle_rate_rad_s_);
      if (!settled) {
        yaw_settle_started_.reset();
      } else if (!yaw_settle_started_) {
        yaw_settle_started_ = now;
      }
      if (yaw_settle_started_ &&
          seconds_since(*yaw_settle_started_, now) >= yaw_settle_hold_s_) {
        transition("APPROACH");
        if (distance > 6.0) forward = forward_max_;
        else if (distance > 2.0) forward = std::min(forward_max_, forward_mid_);
        else if (distance > 0.75) forward = std::min(forward_max_, forward_near_);
        else forward = std::min(forward_max_, forward_contact_);
      } else {
        forward = 0.0;
      }
    } else if (std::abs(bearing) > align_enter_rad_) {
      yaw_settle_started_.reset();
      transition("ALIGN");
    } else {
      yaw_settle_started_.reset();
      transition("APPROACH");
      if (distance > 6.0) forward = forward_max_;
      else if (distance > 2.0) forward = std::min(forward_max_, forward_mid_);
      else if (distance > 0.75) forward = std::min(forward_max_, forward_near_);
      else {
        transition("CONTACT");
        forward = std::min(forward_max_, forward_contact_);
        yaw = clamp(yaw, -0.25, 0.25);
        heave = clamp(heave, -0.20, 0.20);
      }
    }
    if (continuous_range_excitation_enabled_ &&
        distance > std::max(2.0, 1.35 * success_range_m_)) {
      const SteadyTime excitation_started = active_started_.value_or(state_started_);
      const double phase = 2.0 * std::acos(-1.0) *
          seconds_since(excitation_started, now) / range_excitation_period_s_;
      lateral = clamp(
          lateral + range_excitation_sway_ * std::sin(phase),
          -acoustic_position_sway_limit_, acoustic_position_sway_limit_);
    }
    publish_command(Command{forward, lateral, heave, yaw});
  }

  void reset_localization_for_near_probe(const SteadyTime &now) {
    source_estimate_.reset();
    source_smoothed_.reset();
    source_locked_.reset();
    phase_bearing_lock_world_.reset();
    last_force_fit_accepted_ = false;
    last_control_direction_source_ = "unavailable";
    filtered_direction_body_.reset();
    samples_.clear();
    raw_delta_history_.clear();
    cumulative_range_change_m_ = 0.0;
    best_amplitude_distance_ = std::numeric_limits<double>::infinity();
    last_range_progress_ = now;
    range_regression_started_.reset();
    force_full_reprobe_ = range_gradient_enabled_;
    probe_attempt_ = 0;
    transition("REPROBE");
  }

  void reset_localization_for_initial_probe(const SteadyTime &now) {
    samples_.clear();
    range_feedback_samples_.clear();
    raw_delta_history_.clear();
    cumulative_range_change_m_ = 0.0;

    source_estimate_.reset();
    source_smoothed_.reset();
    source_locked_.reset();
    phase_bearing_lock_world_.reset();
    last_force_fit_accepted_ = false;

    range_guidance_world_.reset();
    range_gradient_recovery_active_ = false;
    range_feedback_probe_completed_ = false;
    range_gradient_two_axis_ = false;
    range_gradient_slope_ = 0.0;
    range_gradient_span_m_ = 0.0;
    range_gradient_rms_m_ = 0.0;
    range_gradient_observability_ = 0.0;
    range_gradient_sample_count_ = 0U;

    acoustic_position_candidate_xy_.reset();
    acoustic_source_xy_.reset();
    acoustic_position_candidate_streak_ = 0;
    acoustic_position_condition_ = 0.0;
    acoustic_position_median_residual_m_ = 0.0;
    acoustic_position_rms_residual_m_ = 0.0;
    acoustic_position_sample_count_ = 0U;

    best_amplitude_distance_ = std::numeric_limits<double>::infinity();
    last_range_progress_ = now;
    range_regression_started_.reset();
    last_control_direction_source_ = "unavailable";
    RCLCPP_INFO(
        get_logger(),
        "cleared pre-arm localization history before first C++ pinger probe");
  }

  void begin_success_terminal_sequence() {
    terminal_brake_aborted_ = false;
    terminal_brake_entry_forward_ = last_command_.forward;
    terminal_brake_applied_ = kmu26::pinger_homing::should_start_no_odom_terminal_brake(
        true,
        navigation_mode_ == "no_odom_phase",
        no_odom_terminal_brake_enabled_,
        terminal_brake_entry_forward_,
        no_odom_terminal_brake_command_,
        no_odom_terminal_brake_duration_s_);
    if (terminal_brake_applied_) {
      RCLCPP_INFO(
          get_logger(),
          "no-odom terminal brake: entry_forward=%.3f reverse=%.3f duration=%.2fs",
          terminal_brake_entry_forward_, no_odom_terminal_brake_command_,
          no_odom_terminal_brake_duration_s_);
      transition("TERMINAL_BRAKE");
      return;
    }
    transition("COMPLETE");
  }

  void terminal_brake_tick(const SteadyTime &now) {
    if (!dry_run_ && !live_control_ready(now)) {
      terminal_brake_applied_ = false;
      terminal_brake_aborted_ = true;
      RCLCPP_WARN(
          get_logger(),
          "no-odom terminal brake aborted: state_fresh=%s connected=%s armed=%s "
          "actual_mode=%s",
          vehicle_state_fresh(now) ? "true" : "false",
          connected_ ? "true" : "false", armed_ ? "true" : "false",
          actual_vehicle_mode_.empty() ? "unavailable" : actual_vehicle_mode_.c_str());
      transition("COMPLETE");
      publish_command(Command{});
      return;
    }
    const double elapsed_s = seconds_since(state_started_, now);
    const double reverse =
        kmu26::pinger_homing::no_odom_terminal_brake_forward_command(
            elapsed_s, no_odom_terminal_brake_duration_s_,
            no_odom_terminal_brake_command_);
    if (reverse < 0.0) {
      // Fixed body-X braking only: STABILIZE retains attitude ownership and
      // the controller never chases a changing acoustic bearing after the
      // success decision has latched.
      publish_command(Command{reverse, 0.0, 0.0, 0.0});
      return;
    }
    transition("COMPLETE");
    publish_command(Command{});
  }

  void publish_success_terminal_command(const SteadyTime &now) {
    if (state_ == "TERMINAL_BRAKE") {
      terminal_brake_tick(now);
    } else {
      publish_command(Command{});
    }
  }

  bool maybe_complete(const SteadyTime &now) {
    const bool arrival_state = probe_completed_ &&
        (state_ == "ALIGN" || state_ == "APPROACH" || state_ == "CONTACT");
    if (!arrival_state) {
      range_success_started_.reset();
      arrival_success_started_.reset();
      return false;
    }

    const auto calibrated_distance = amplitude_range();
    if (success_range_m_ > 0.0 && calibrated_distance &&
        *calibrated_distance <= success_range_m_) {
      if (!range_success_started_) range_success_started_ = now;
      if (seconds_since(*range_success_started_, now) >= success_hold_s_) {
        range_complete_ = true;
        completion_reason_ = "calibrated_range";
        RCLCPP_INFO(
            get_logger(),
            "pinger homing calibrated-range success: range=%.3fm threshold=%.3fm",
            *calibrated_distance, success_range_m_);
        begin_success_terminal_sequence();
        return true;
      }
    } else {
      range_success_started_.reset();
    }

    // Match the Python estimated-arrival fallback, but only accept the
    // stationary source locked by a completed probe.  A transient smoothed
    // candidate must never terminate homing.
    std::optional<double> estimated_distance;
    // When a calibrated range success threshold is configured (the simulator
    // passes 1.2 m), that independent measurement is authoritative.  A local
    // source fit can become self-consistent near a pool wall while calibrated
    // amplitude still says the pinger is several metres away.  In that mode
    // keep approaching/reprobe on range progress and never terminate through
    // the estimated-arrival fallback.  The real Python-compatible default
    // has success_range_m=0 and retains its source-lock fallback unchanged.
    const bool estimated_arrival_fallback_enabled = success_range_m_ <= 0.0;
    const bool estimate_ready = estimated_arrival_fallback_enabled &&
        arrival_radius_m_ > 0.0 && source_locked_ && estimate_usable() && position_;
    if (estimate_ready) {
      const double distance = (*source_locked_ - *position_).norm();
      if (std::isfinite(distance) && distance >= 0.0) estimated_distance = distance;
    }
    if (!estimated_distance || *estimated_distance > arrival_radius_m_) {
      arrival_success_started_.reset();
      return false;
    }
    if (!arrival_success_started_) arrival_success_started_ = now;
    if (seconds_since(*arrival_success_started_, now) < arrival_hold_s_) return false;

    arrival_complete_ = true;
    completion_reason_ = "estimated_arrival";
    RCLCPP_INFO(
        get_logger(),
        "pinger homing arrival success: estimated_range=%.3fm radius=%.3fm",
        *estimated_distance, arrival_radius_m_);
    begin_success_terminal_sequence();
    return true;
  }

  std::optional<double> amplitude_range() const {
    if (amplitude_range_history_.empty()) return std::nullopt;
    const std::vector<double> values(
        amplitude_range_history_.begin(), amplitude_range_history_.end());
    const double value = kmu26::pinger_homing::median(values);
    return std::isfinite(value) && value > 0.1 && value < 80.0
        ? std::optional<double>(value) : std::nullopt;
  }

  std::optional<Eigen::Vector3d> phase_guidance_vector_world() const {
    if (!position_ || !phase_direction_world_ ||
        !phase_direction_world_->allFinite() ||
        phase_direction_world_->head<2>().norm() <= 1.0e-9) {
      return std::nullopt;
    }

    double target_depth_m = 0.0;
    if (auto_source_depth_ && tank_max_depth_m_ > 0.0) {
      target_depth_m =
          kmu26::pinger_homing::derive_pinger_depth_from_tank(tank_max_depth_m_);
    } else if (pinger_expected_depth_m_ > 0.0) {
      target_depth_m = pinger_expected_depth_m_;
    }
    if (target_depth_m <= 0.0) return std::nullopt;

    // A moving single hydrophone provides a useful Phase azimuth much sooner
    // than a trustworthy elevation.  Use only the configured tank depth to
    // resolve Z; this is controller guidance and never consumes simulator
    // ground truth.  The amplitude-derived range preserves the natural
    // horizontal/vertical ratio as the vehicle approaches the pinger.
    const Eigen::Vector3d bearing_world = phase_bearing_lock_world_
        ? phase_bearing_lock_world_->normalized()
        : phase_direction_world_->normalized();
    const Eigen::Vector2d phase_xy = bearing_world.head<2>().normalized();
    const double range_m = std::max(amplitude_range().value_or(unknown_range_m_), 1.0);
    const double dz = -target_depth_m - position_->z();
    const double horizontal_m = std::sqrt(std::max(
        0.05 * 0.05, range_m * range_m - dz * dz));
    Eigen::Vector3d guidance;
    guidance.head<2>() = horizontal_m * phase_xy;
    guidance.z() = dz;
    if (!guidance.allFinite() || guidance.norm() <= 1.0e-9) return std::nullopt;
    return guidance;
  }

  void publish_phase_direction_if_available(const SteadyTime &now) {
    if (!direction_usable(now)) return;
    const Eigen::Vector3d phase_body = kmu26::pinger_homing::world_vector_to_body_flu(
        phase_direction_world_->normalized(), qx_, qy_, qz_, qw_);
    if (!phase_body.allFinite() || phase_body.norm() <= 1.0e-9) return;

    // A moving single hydrophone observes Phase azimuth quickly, but its
    // elevation is weak until a dedicated vertical leg has completed and it
    // becomes stale after the vehicle passes the source.  Fuse the reliable
    // Phase azimuth with an acoustic range-fit elevation.  Before the first
    // fit, derive only a depth prior from tank_max_depth_m; no ground truth is
    // consumed.  In the near field the position fit owns the whole vector so
    // a lagging Phase bearing cannot keep driving past the pinger.
    std::optional<Eigen::Vector3d> position_direction_world;
    if (position_) {
      const std::optional<Eigen::Vector3d> source = source_locked_
          ? source_locked_
          : (require_source_lock_ && estimate_usable()
              ? source_smoothed_ : std::nullopt);
      if (source && (*source - *position_).norm() > 1.0e-9) {
        position_direction_world = (*source - *position_).normalized();
      } else if (const auto prior = phase_guidance_vector_world()) {
        position_direction_world = prior->normalized();
      }
    }

    Eigen::Vector3d output_body = phase_body.normalized();
    if (position_direction_world) {
      const Eigen::Vector3d position_body =
          kmu26::pinger_homing::world_vector_to_body_flu(
              *position_direction_world, qx_, qy_, qz_, qw_).normalized();
      const std::optional<double> range = amplitude_range();
      if (phase_bearing_lock_world_ && !source_locked_) {
        // The completed, neutral-separated probe is the calibrated Phase
        // bearing observation.  Do not replace its yaw with the travelling
        // EKF output; acoustic range still detects stalled progress and starts
        // a stopped reprobe when a new bearing is required.
        output_body = position_body;
      } else if (range && *range <= phase_yaw_min_range_m_) {
        output_body = position_body;
      } else {
        output_body =
            kmu26::pinger_homing::apply_acoustic_yaw_to_position_direction(
                position_body, phase_body);
      }
    }
    publish_direction(output_body.normalized());
  }

  void publish_direction(const Eigen::Vector3d &body) {
    publish_body_direction(direction_pub_, body);
  }

  void publish_control_direction(const Eigen::Vector3d &body) {
    publish_body_direction(control_direction_pub_, body);
  }

  void publish_body_direction(
      const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr &publisher,
      const Eigen::Vector3d &body) {
    if (!publisher) return;
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.vector.x = body.x();
    msg.vector.y = body.y();
    msg.vector.z = body.z();
    publisher->publish(msg);
  }

  void publish_command(const Command &requested) {
    last_requested_command_ = requested;
    const auto now = SteadyClock::now();

    // Run the depth limiter even when outputs are inhibited.  Dry-run is used
    // both by the simulator and by real-vehicle preflight, so its status must
    // expose the exact safety decision that would be applied after arming.
    const std::optional<double> position_z = current_position_z(now);
    const auto safety = kmu26::pinger_homing::limit_heave_by_vehicle_depth(
        requested.heave, position_z, max_vehicle_depth_m_, depth_soft_margin_m_,
        depth_recovery_heave_);
    depth_limit_active_ = safety.limit_active;
    depth_recovery_active_ = safety.recovery_active;
    vehicle_depth_m_ = safety.vehicle_depth_m;

    if (dry_run_) {
      last_command_ = {};
      publish_release();
      if (transport_ != "rc_override") publish_direct(Command{});
      return;
    }

    if (!live_control_ready(now)) {
      // Never emit a non-neutral override until MAVROS has freshly confirmed
      // connected + armed + STABILIZE.  A fresh armed vehicle in a different
      // mode receives neutral immediately; missing/disconnected/disarmed state
      // releases the override so a stale command cannot survive a state outage.
      last_command_ = {};
      if (transport_ == "rc_override") {
        if (vehicle_state_fresh(now) && connected_ && armed_) {
          publish_rc(Command{});
        } else {
          publish_release();
        }
      } else {
        publish_direct(Command{});
      }
      return;
    }

    Command command = requested;
    command.heave = safety.command_heave;
    last_command_ = command;
    if (transport_ == "rc_override") publish_rc(command);
    else publish_direct(command);
  }

  void publish_rc(const Command &command) {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    for (std::size_t index = 0; index < kPrimaryRcChannelCount; ++index) {
      msg.channels[index] = kRcNeutral;
    }
    const double xy_deadzone =
        rc_deadzone_compensation_enabled_ ? rc_xy_deadzone_pwm_ : 0.0;
    const double yaw_deadzone =
        rc_deadzone_compensation_enabled_ ? rc_yaw_deadzone_pwm_ : 0.0;
    const double heave_deadzone =
        rc_deadzone_compensation_enabled_ ? rc_heave_deadzone_pwm_ : 0.0;
    msg.channels[kChHeave] = static_cast<uint16_t>(
        axis_pwm(command.heave, rc_pwm_span_, heave_deadzone));
    msg.channels[kChYaw] = static_cast<uint16_t>(
        axis_pwm(-command.yaw, rc_pwm_span_, yaw_deadzone));
    msg.channels[kChForward] = static_cast<uint16_t>(
        axis_pwm(command.forward, rc_pwm_span_, xy_deadzone));
    msg.channels[kChSway] = static_cast<uint16_t>(
        axis_pwm(command.lateral, rc_pwm_span_, xy_deadzone));
    rc_pub_->publish(msg);
  }

  void publish_release() {
    if (!rc_pub_) return;
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    rc_pub_->publish(msg);
  }

  void publish_direct(const Command &command) {
    if (!direct_pub_) return;
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "{\"source\":\"pinger_homing_cpp\",\"phase\":\"" << state_
        << "\",\"direct_cmd\":{\"forward\":" << command.forward
        << ",\"sway\":" << command.lateral << ",\"heave\":" << command.heave
        << ",\"yaw\":" << command.yaw << ",\"pitch\":0.0}}";
    msg.data = out.str();
    direct_pub_->publish(msg);
  }

  void maybe_request_arm_mode(const SteadyTime &now) {
    // Automatic arming/mode selection belongs to an active homing session
    // only.  The node intentionally remains alive after a terminal result so
    // its final status can still be observed; without this guard that idle
    // node kept re-arming the vehicle and forcing its requested mode once per second.
    const bool terminal_braking = state_ == "TERMINAL_BRAKE";
    if ((range_complete_ && !terminal_braking) || state_ == "COMPLETE" ||
        state_.rfind("FAILED_", 0) == 0) {
      return;
    }
    // A terminal brake may only continue in an already-confirmed STABILIZE
    // state.  Do not issue a late mode/arm request that could take effect after
    // the brake has been aborted and ownership has ended.
    if (terminal_braking) return;
    if ((!auto_arm_ && !auto_mode_) || seconds_since(last_service_request_, now) < 1.0) return;
    if (!vehicle_state_fresh(now) || !connected_) return;
    last_service_request_ = now;
    if (!vehicle_mode_matches_required()) {
      if (auto_mode_ && mode_client_->service_is_ready()) {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode_;
        mode_client_->async_send_request(request);
      }
      // Never arm in MANUAL (or any other mode).  Wait for a later fresh
      // /mavros/state sample to confirm STABILIZE before taking arm ownership.
      return;
    }
    if (auto_arm_ && !armed_ && arm_client_->service_is_ready()) {
      auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      request->value = true;
      arm_client_->async_send_request(request);
    }
  }

  void maybe_shutdown_after_terminal(const SteadyTime &now) {
    if (terminal_shutdown_requested_ || terminal_exit_delay_s_ <= 0.0 ||
        seconds_since(state_started_, now) < terminal_exit_delay_s_) {
      return;
    }
    terminal_shutdown_requested_ = true;
    // Publish release once more before ending this node. The launch contract
    // observes the controller exit and shuts down the RC mux/estimators too.
    publish_release();
    RCLCPP_INFO(
        get_logger(),
        "pinger homing terminal state %s held %.2fs; shutting down launch owner",
        state_.c_str(), terminal_exit_delay_s_);
    rclcpp::shutdown();
  }

  void transition(const std::string &next) {
    if (next == state_) return;
    RCLCPP_INFO(get_logger(), "C++ homing state: %s -> %s", state_.c_str(), next.c_str());
    state_ = next;
    state_started_ = SteadyClock::now();
    if (state_ == "ALIGN") yaw_settle_started_.reset();
    if (state_ == "WAIT_VEHICLE" || state_ == "PROBE" || state_ == "REPROBE") {
      probe_completed_ = false;
      range_success_started_.reset();
      arrival_success_started_.reset();
    }
    if (state_ == "PROBE" || state_ == "REPROBE") {
      const auto position_z = current_position_z(SteadyClock::now());
      const std::optional<double> depth = position_z
          ? std::optional<double>(std::max(0.0, -*position_z)) : std::nullopt;
      probe_heave_for_state_ = auto_source_depth_
          ? kmu26::pinger_homing::select_auto_probe_heave(
                depth, max_vehicle_depth_m_, 0.45, auto_probe_heave_magnitude_)
          : probe_heave_;
      RCLCPP_INFO(
          get_logger(), "C++ vertical probe heave=%+.3f depth=%s limit=%.3fm",
          probe_heave_for_state_, json_number(depth, 3).c_str(), max_vehicle_depth_m_);
    }
  }

  void publish_status() {
    if (!status_pub_) return;
    const auto now = SteadyClock::now();
    const bool odometry_fresh = position_ &&
        seconds_since(last_odometry_, now) < odometry_timeout_s_;
    const bool imu_fresh = imu_orientation_ &&
        seconds_since(last_imu_, now) < imu_timeout_s_;
    const bool depth_fresh = current_position_z(now).has_value();
    const bool phase_measurement_fresh = phase_delta_fresh(now);
    const bool quality_fresh = audio_quality_fresh(now);
    const bool audio_fresh = audio_ready_for_probe(now);
    const bool state_fresh = vehicle_state_fresh(now);
    const bool mode_ready = vehicle_mode_ready(now);
    const bool live_ready = live_control_ready(now);
    const bool direction_fresh = navigation_mode_ == "no_odom_phase"
        ? no_odom_direction_world_.has_value() : direction_usable(now);
    const bool inputs_ready = (dry_run_ || live_ready) && audio_fresh &&
        (navigation_mode_ == "no_odom_phase"
            ? (imu_fresh && (max_vehicle_depth_m_ <= 0.0 || depth_fresh))
            : odometry_fresh);
    const std::optional<Eigen::Vector3d> control_source =
        source_locked_ ? source_locked_ : source_smoothed_;
    std::optional<double> estimated_distance;
    if (control_source && position_) estimated_distance = (*control_source - *position_).norm();
    const std::optional<double> range_guidance_age = range_guidance_world_
        ? std::optional<double>(seconds_since(last_range_guidance_, now)) : std::nullopt;
    const std::optional<Eigen::Vector3d> acoustic_source_world = acoustic_source_xy_
        ? std::optional<Eigen::Vector3d>(Eigen::Vector3d(
              acoustic_source_xy_->x(), acoustic_source_xy_->y(),
              -pinger_expected_depth_m_))
        : std::nullopt;
    const std::optional<double> acoustic_source_age = acoustic_source_xy_
        ? std::optional<double>(seconds_since(last_acoustic_position_fit_, now))
        : std::nullopt;
    const bool output_active = !dry_run_ && live_ready;
    const bool motion_response_feedback_is_fresh = motion_response_feedback_fresh(now);
    const std::optional<double> motion_response_speed = motion_response_feedback_is_fresh
        ? motion_response_horizontal_speed_mps_ : std::nullopt;
    const std::optional<double> motion_response_low_speed_elapsed =
        motion_response_low_speed_started_
            ? std::optional<double>(seconds_since(*motion_response_low_speed_started_, now))
            : std::nullopt;
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << std::fixed << std::setprecision(4)
        << "{\"running\":"
        << bool_text(state_ != "COMPLETE" && state_.rfind("FAILED_", 0) != 0)
        << ",\"state\":\"" << state_ << "\",\"phase\":\"" << state_
        << "\",\"implementation\":\"cpp\",\"controller_mode\":\"" << controller_mode_
        << "\",\"navigation_mode\":\"" << navigation_mode_
        << "\",\"odometry_required\":" << bool_text(navigation_mode_ == "odometry")
        << ",\"acoustic_estimator_mode\":\"" << acoustic_estimator_mode_
        << "\",\"controller_profile\":\"" << controller_profile_
        << "\",\"legacy_python_sequence\":" << bool_text(legacy_python_sequence_)
        << ",\"direction_frame\":\"" << direction_frame_
        << "\",\"time_base\":\"steady_monotonic\""
        << ",\"estimator_direction_topic\":\"" << direction_output_topic_ << "\""
        << ",\"control_direction_topic\":\""
        << control_direction_output_topic_ << "\""
        << ",\"source_xy_bounds_enabled\":"
        << bool_text(source_xy_bounds_enabled_)
        << ",\"dry_run\":" << bool_text(dry_run_)
        << ",\"control_output_active\":" << bool_text(output_active)
        << ",\"connected\":"
        << bool_text(state_fresh && (connected_ || connection_grace_active(now)))
        << ",\"armed\":" << bool_text(vehicle_armed_effective(now))
        << ",\"raw_connected\":" << bool_text(connected_)
        << ",\"raw_armed\":" << bool_text(armed_)
        << ",\"vehicle_state_fresh\":" << bool_text(state_fresh)
        << ",\"vehicle_state_age_s\":" << seconds_since(last_vehicle_state_, now)
        << ",\"vehicle_state_timeout_s\":" << vehicle_state_timeout_s_
        << ",\"actual_vehicle_mode\":\"" << actual_vehicle_mode_ << "\""
        << ",\"required_vehicle_mode\":\"" << mode_ << "\""
        << ",\"vehicle_mode_ready\":" << bool_text(mode_ready)
        << ",\"audio_fresh\":" << bool_text(audio_fresh)
        << ",\"phase_measurement_fresh\":" << bool_text(phase_measurement_fresh)
        << ",\"audio_quality_fresh\":" << bool_text(quality_fresh)
        << ",\"audio_bootstrap_active\":"
        << bool_text(!phase_measurement_fresh && bootstrap_probe_on_audio_quality_ && quality_fresh)
        << ",\"audio_quality_value\":" << json_number(last_audio_quality_value_)
        << ",\"direction_fresh\":" << bool_text(direction_fresh)
        << ",\"odometry_fresh\":" << bool_text(odometry_fresh)
        << ",\"imu_fresh\":" << bool_text(imu_fresh)
        << ",\"depth_fresh\":" << bool_text(depth_fresh)
        << ",\"inputs_ready\":" << bool_text(inputs_ready)
        << ",\"sample_count\":" << samples_.size()
        << ",\"probe_attempt\":" << probe_attempt_
        << ",\"minimum_probe_legs\":" << minimum_probe_legs_
        << ",\"legacy_probe_pwm_delta\":"
        << static_cast<int>(std::lround(probe_scale_ * rc_pwm_span_))
        << ",\"legacy_approach_pwm_delta\":"
        << static_cast<int>(std::lround(forward_max_ * rc_pwm_span_))
        << ",\"rc_deadzone_compensation_enabled\":"
        << bool_text(rc_deadzone_compensation_enabled_)
        << ",\"rc_xy_deadzone_pwm\":" << rc_xy_deadzone_pwm_
        << ",\"rc_yaw_deadzone_pwm\":" << rc_yaw_deadzone_pwm_
        << ",\"rc_heave_deadzone_pwm\":" << rc_heave_deadzone_pwm_
        << ",\"legacy_probe_duration_scale\":" << probe_duration_scale_
        << ",\"probe_completed\":" << bool_text(probe_completed_)
        << ",\"require_source_lock\":" << bool_text(require_source_lock_)
        << ",\"prefer_direction_control\":" << bool_text(prefer_direction_control_)
        << ",\"phase_yaw_guidance_enabled\":"
        << bool_text(phase_yaw_guidance_enabled_)
        << ",\"phase_yaw_min_range_m\":" << phase_yaw_min_range_m_
        << ",\"yaw_control\":{\"gain\":" << yaw_gain_
        << ",\"command_limit\":" << yaw_command_limit_
        << ",\"slew_rate\":" << yaw_slew_rate_
        << ",\"brake_gain\":" << yaw_brake_gain_
        << ",\"brake_slew_rate\":" << yaw_brake_slew_rate_
        << ",\"brake_horizon_s\":" << yaw_brake_horizon_s_
        << ",\"settle_rate_rad_s\":" << yaw_settle_rate_rad_s_
        << ",\"settle_hold_s\":" << yaw_settle_hold_s_
        << ",\"settle_active\":" << bool_text(yaw_settle_started_.has_value())
        << "}"
        << ",\"phase_vertical_disambiguation_enabled\":"
        << bool_text(phase_vertical_disambiguation_enabled_)
        << ",\"direction_age_s\":" << seconds_since(last_direction_, now)
        << ",\"odometry_age_s\":" << seconds_since(last_odometry_, now)
        << ",\"imu_age_s\":" << seconds_since(last_imu_, now)
        << ",\"depth_pose_age_s\":" << seconds_since(last_depth_pose_, now)
        << ",\"audio_age_s\":" << seconds_since(last_audio_, now)
        << ",\"estimated_source_world\":" << json_vector(control_source)
        << ",\"source_locked\":" << bool_text(source_locked_.has_value())
        << ",\"estimated_distance_m\":" << json_number(estimated_distance)
        << ",\"amplitude_distance_m\":" << json_number(amplitude_range())
        << ",\"rms_residual_m\":"
        << json_number(source_estimate_
            ? std::optional<double>(source_estimate_->rms_residual_m) : std::nullopt)
        << ",\"condition_number\":"
        << json_number(source_estimate_
            ? std::optional<double>(source_estimate_->condition_number) : std::nullopt)
        << ",\"absolute_range_fit\":{\"sample_count\":"
        << (source_estimate_ ? source_estimate_->absolute_sample_count : 0U)
        << ",\"median_residual_m\":"
        << json_number(source_estimate_
            ? std::optional<double>(source_estimate_->absolute_median_residual_m)
            : std::nullopt)
        << ",\"rms_residual_m\":"
        << json_number(source_estimate_
            ? std::optional<double>(source_estimate_->absolute_rms_residual_m)
            : std::nullopt)
        << ",\"latest_error_m\":"
        << json_number(source_estimate_
            ? std::optional<double>(source_estimate_->latest_absolute_error_m)
            : std::nullopt)
        << ",\"latest_range_m\":"
        << json_number(source_estimate_
            ? std::optional<double>(source_estimate_->latest_absolute_range_m)
            : std::nullopt)
        << ",\"last_force_fit_accepted\":" << bool_text(last_force_fit_accepted_)
        << "}"
        << ",\"source_fit_period_s\":" << source_fit_period_s_
        << ",\"phase_direction_world\":" << json_vector(phase_direction_world_)
        << ",\"no_odom_phase_direction_world\":" << json_vector(no_odom_direction_world_)
        << ",\"phase_bearing_lock_world\":"
        << json_vector(phase_bearing_lock_world_)
        << ",\"control_direction_source\":\"" << last_control_direction_source_ << "\""
        << ",\"no_odom_phase\":{\"probe_axis\":" << no_odom_probe_axis_
        << ",\"probe_count\":" << no_odom_probe_count_
        << ",\"score_body\":" << json_vector(no_odom_probe_score_body_)
        << ",\"neutral_samples\":" << no_odom_neutral_delta_count_
        << ",\"probe_scheme\":\"abba_huber_drift\""
        << ",\"horizontal_only\":"
        << bool_text(no_odom_horizontal_only_)
        << ",\"initial_confirmation_required\":"
        << no_odom_initial_confirmation_probes_
        << ",\"initial_confirmation_count\":"
        << no_odom_initial_consistent_count_
        << ",\"initial_confirmation_dot\":" << json_number(
            std::isfinite(no_odom_initial_confirmation_dot_)
                ? std::optional<double>(no_odom_initial_confirmation_dot_)
                : std::nullopt, 4)
        << ",\"initial_confirmed\":"
        << bool_text(no_odom_initial_bearing_confirmed_)
        << ",\"sample_delay_s\":" << no_odom_probe_sample_delay_s_
        << ",\"fit_samples\":" << no_odom_phase_samples_.size()
        << ",\"fit_rms_m\":" << json_number(
            std::isfinite(no_odom_probe_fit_rms_m_)
                ? std::optional<double>(no_odom_probe_fit_rms_m_)
                : std::nullopt, 7)
        << ",\"fit_inlier_ratio\":" << no_odom_probe_fit_inlier_ratio_
        << ",\"probe_pwm_delta\":" << no_odom_probe_pwm_delta_
        << ",\"approach_pwm_delta\":" << no_odom_approach_pwm_delta_
        << ",\"reestimate_policy\":\"" << no_odom_reestimate_policy_ << "\""
        << ",\"legacy_fixed_duration_s\":" << no_odom_forward_duration_s_
        << ",\"approach_min_s\":" << no_odom_approach_min_s_
        << ",\"approach_max_s\":" << no_odom_approach_max_s_
        << ",\"approach_elapsed_s\":" << json_number(
            no_odom_forward_started_
                ? std::optional<double>(seconds_since(*no_odom_forward_started_, now))
                : std::nullopt)
        << ",\"expected_delta_m\":" << json_number(
            std::isfinite(no_odom_forward_expected_delta_m_)
                ? std::optional<double>(no_odom_forward_expected_delta_m_)
                : std::nullopt, 7)
        << ",\"observed_delta_m\":" << json_number(no_odom_observed_delta_m_, 7)
        << ",\"innovation_ratio\":" << json_number(no_odom_innovation_ratio_, 4)
        << ",\"innovation_limit\":" << no_odom_innovation_limit_
        << ",\"innovation_hold_s\":" << no_odom_innovation_hold_s_
        << ",\"innovation_reestimate_count\":" << no_odom_innovation_reestimate_count_
        << ",\"watchdog_reestimate_count\":" << no_odom_watchdog_reestimate_count_
        << ",\"terminal_brake_enabled\":"
        << bool_text(no_odom_terminal_brake_enabled_)
        << ",\"terminal_brake_active\":"
        << bool_text(state_ == "TERMINAL_BRAKE")
        << ",\"terminal_brake_applied\":" << bool_text(terminal_brake_applied_)
        << ",\"terminal_brake_aborted\":" << bool_text(terminal_brake_aborted_)
        << ",\"terminal_brake_command\":" << no_odom_terminal_brake_command_
        << ",\"terminal_brake_pwm_delta\":" << no_odom_terminal_brake_pwm_delta_
        << ",\"terminal_brake_duration_s\":" << no_odom_terminal_brake_duration_s_
        << ",\"terminal_brake_entry_forward\":" << terminal_brake_entry_forward_
        << "}"
        << ",\"motion_response\":{\"enabled\":"
        << bool_text(motion_response_enabled_)
        << ",\"uses_for_bearing\":false"
        << ",\"role\":\"probe_timing_and_reestimate\""
        << ",\"velocity_topic\":\"" << motion_response_velocity_topic_ << "\""
        << ",\"feedback_fresh\":" << bool_text(motion_response_feedback_is_fresh)
        << ",\"feedback_age_s\":"
        << seconds_since(last_motion_response_velocity_, now)
        << ",\"horizontal_speed_mps\":" << json_number(motion_response_speed)
        << ",\"requested_xy\":" << motion_response_requested_xy_
        << ",\"min_command\":" << motion_response_min_command_
        << ",\"min_speed_mps\":" << motion_response_min_speed_mps_
        << ",\"probe_extension_s\":" << no_odom_probe_active_leg_extension_s_
        << ",\"probe_max_extension_s\":" << motion_response_probe_max_extension_s_
        << ",\"approach_grace_s\":" << motion_response_approach_grace_s_
        << ",\"approach_hold_s\":" << motion_response_approach_hold_s_
        << ",\"low_speed_elapsed_s\":" << json_number(motion_response_low_speed_elapsed)
        << ",\"reestimate_count\":" << motion_response_reestimate_count_ << "}"
        << ",\"range_feedback\":{\"enabled\":" << bool_text(range_gradient_enabled_)
        << ",\"continuous_excitation\":"
        << bool_text(continuous_range_excitation_enabled_)
        << ",\"require_two_axis\":" << bool_text(range_guidance_require_two_axis_)
        << ",\"direction_vectoring\":" << bool_text(direction_vectoring_enabled_)
        << ",\"guidance_world\":" << json_vector(range_guidance_world_)
        << ",\"guidance_age_s\":" << json_number(range_guidance_age)
        << ",\"recovery_active\":" << bool_text(range_gradient_recovery_active_)
        << ",\"force_full_reprobe\":" << bool_text(force_full_reprobe_)
        << ",\"near_probe_completed\":" << bool_text(range_feedback_probe_completed_)
        << ",\"takeover_range_m\":" << range_gradient_takeover_m_
        << ",\"excitation_sway\":" << range_excitation_sway_
        << ",\"excitation_period_s\":" << range_excitation_period_s_
        << ",\"probe_duration_s\":" << range_feedback_probe_duration_s_
        << ",\"two_axis_observable\":" << bool_text(range_gradient_two_axis_)
        << ",\"slope_m_per_m\":" << range_gradient_slope_
        << ",\"span_m\":" << range_gradient_span_m_
        << ",\"rms_residual_m\":" << range_gradient_rms_m_
        << ",\"observability_ratio\":" << range_gradient_observability_
        << ",\"sample_count\":" << range_gradient_sample_count_ << "}"
        << ",\"acoustic_position_feedback\":{\"enabled\":"
        << bool_text(acoustic_position_enabled_)
        << ",\"control_enabled\":" << bool_text(acoustic_position_control_enabled_)
        << ",\"source_world\":" << json_vector(acoustic_source_world)
        << ",\"age_s\":" << json_number(acoustic_source_age)
        << ",\"candidate_streak\":" << acoustic_position_candidate_streak_
        << ",\"condition_number\":" << acoustic_position_condition_
        << ",\"median_residual_m\":" << acoustic_position_median_residual_m_
        << ",\"rms_residual_m\":" << acoustic_position_rms_residual_m_
        << ",\"sample_count\":" << acoustic_position_sample_count_ << "}"
        << ",\"command\":{\"forward\":" << last_command_.forward
        << ",\"lateral\":" << last_command_.lateral
        << ",\"heave\":" << last_command_.heave
        << ",\"yaw\":" << last_command_.yaw << "}"
        << ",\"requested_command\":{\"forward\":" << last_requested_command_.forward
        << ",\"lateral\":" << last_requested_command_.lateral
        << ",\"heave\":" << last_requested_command_.heave
        << ",\"yaw\":" << last_requested_command_.yaw << "}"
        << ",\"depth_safety\":{\"vehicle_depth_m\":" << json_number(vehicle_depth_m_)
        << ",\"tank_max_depth_m\":" << tank_max_depth_m_
        << ",\"auto_source_depth\":" << bool_text(auto_source_depth_)
        << ",\"pinger_expected_depth_m\":" << pinger_expected_depth_m_
        << ",\"vehicle_target_depth_m\":" << vehicle_target_depth_m_
        << ",\"max_vehicle_depth_m\":" << max_vehicle_depth_m_
        << ",\"soft_margin_m\":" << depth_soft_margin_m_
        << ",\"probe_heave\":" << probe_heave_for_state_
        << ",\"limit_active\":" << bool_text(depth_limit_active_)
        << ",\"recovery_active\":" << bool_text(depth_recovery_active_) << "}"
        << ",\"bearing_error_deg\":" << (last_bearing_rad_ * 180.0 / kPi)
        << ",\"yaw_rate_rad_s\":" << current_yaw_rate(now)
        // Retain this legacy status key so older status consumers can read
        // the standalone controller without a schema exception. Pinger homing no
        // longer subscribes to mission/collector state, so it is always false.
        << ",\"capture_confirmed\":false"
        << ",\"range_complete\":" << bool_text(range_complete_)
        << ",\"arrival_complete\":" << bool_text(arrival_complete_)
        << ",\"completion_reason\":\"" << completion_reason_ << "\""
        << ",\"success_range_m\":" << success_range_m_
        << ",\"success_hold_s\":" << success_hold_s_
        << ",\"estimated_arrival_fallback_enabled\":"
        << bool_text(success_range_m_ <= 0.0)
        << ",\"arrival_radius_m\":" << arrival_radius_m_
        << ",\"arrival_hold_s\":" << arrival_hold_s_
        << ",\"first_lock_confirmation_radius_m\":"
        << first_lock_confirmation_radius_m_
        << ",\"amplitude_range_constant\":" << amplitude_range_constant_ << "}";
    msg.data = out.str();
    status_pub_->publish(msg);
  }

  std::string controller_mode_;
  std::string navigation_mode_;
  std::string acoustic_estimator_mode_;
  std::string controller_profile_;
  std::string transport_;
  std::string direction_topic_;
  std::string direction_frame_;
  std::string odometry_topic_;
  std::string motion_response_velocity_topic_;
  std::string imu_topic_;
  std::string depth_pose_topic_;
  std::string vehicle_state_topic_;
  std::string delta_range_topic_;
  std::string iq_magnitude_topic_;
  std::string audio_quality_topic_;
  std::string direction_output_topic_;
  std::string control_direction_output_topic_;
  std::string status_topic_;
  std::string rc_output_topic_;
  std::string command_override_topic_;
  std::string mode_;

  bool dry_run_{true};
  bool legacy_python_sequence_{false};
  bool auto_arm_{false};
  bool auto_mode_{false};
  double rate_hz_{30.0};
  double control_period_s_{1.0 / 30.0};
  double forward_max_{0.48};
  double max_runtime_s_{0.0};
  double terminal_exit_delay_s_{3.0};
  double yaw_gain_{0.85};
  double yaw_rate_damping_{0.18};
  double yaw_command_limit_{0.42};
  double yaw_deadband_rad_{radians(2.5)};
  double yaw_slew_rate_{0.90};
  double yaw_brake_gain_{0.55};
  double yaw_brake_slew_rate_{2.2};
  double yaw_brake_horizon_s_{0.45};
  double yaw_settle_rate_rad_s_{0.10};
  double yaw_settle_hold_s_{0.20};
  double direction_input_filter_alpha_{0.12};
  double direction_filter_alpha_{0.22};
  double align_enter_rad_{radians(28.0)};
  double align_exit_rad_{radians(10.0)};
  double probe_scale_{0.28};
  double odometry_timeout_s_{2.0};
  double imu_timeout_s_{1.0};
  double depth_pose_timeout_s_{1.0};
  double audio_timeout_s_{3.0};
  double audio_quality_timeout_s_{3.0};
  bool bootstrap_probe_on_audio_quality_{true};
  double vehicle_disconnect_grace_s_{0.0};
  double vehicle_state_timeout_s_{3.5};
  double direction_timeout_s_{1.0};
  double no_odom_probe_scale_{0.22};
  double no_odom_probe_heave_scale_{0.12};
  double no_odom_probe_leg_s_{1.5};
  double no_odom_probe_neutral_s_{0.50};
  double no_odom_probe_settle_s_{0.80};
  double no_odom_probe_sample_delay_s_{0.45};
  double no_odom_probe_huber_k_{1.5};
  int no_odom_initial_confirmation_probes_{1};
  double no_odom_initial_confirmation_dot_min_{0.75};
  int no_odom_min_samples_per_leg_{3};
  double no_odom_min_horizontal_signal_{1.0e-4};
  double no_odom_forward_command_{0.30};
  std::string no_odom_reestimate_policy_{"adaptive"};
  double no_odom_forward_duration_s_{4.0};
  double no_odom_approach_min_s_{2.5};
  double no_odom_approach_max_s_{25.0};
  bool no_odom_innovation_enabled_{true};
  double no_odom_innovation_window_s_{0.70};
  double no_odom_innovation_noise_floor_m_{0.0005};
  double no_odom_innovation_limit_{1.50};
  double no_odom_innovation_hold_s_{1.20};
  double no_odom_innovation_min_expected_delta_m_{0.0002};
  bool no_odom_terminal_brake_enabled_{true};
  double no_odom_terminal_brake_command_{0.22};
  double no_odom_terminal_brake_duration_s_{0.90};
  int no_odom_probe_pwm_delta_{88};
  int no_odom_approach_pwm_delta_{120};
  int no_odom_terminal_brake_pwm_delta_{88};
  bool no_odom_vertical_control_enabled_{true};
  bool no_odom_horizontal_only_{false};
  double no_odom_heave_limit_{0.18};
  bool motion_response_enabled_{false};
  double motion_response_min_command_{0.05};
  double motion_response_min_speed_mps_{0.03};
  double motion_response_probe_extension_s_{0.30};
  double motion_response_probe_max_extension_s_{1.20};
  double motion_response_approach_grace_s_{0.80};
  double motion_response_approach_hold_s_{0.80};
  double motion_response_feedback_timeout_s_{0.75};
  bool require_source_lock_{false};
  bool prefer_direction_control_{false};
  bool phase_yaw_guidance_enabled_{true};
  double phase_yaw_min_range_m_{6.0};
  bool phase_vertical_disambiguation_enabled_{false};
  double phase_bearing_follow_after_probe_s_{2.5};
  int phase_bearing_follow_min_updates_{3};
  double direction_follow_after_probe_s_{2.5};
  int direction_follow_min_updates_{3};
  double source_phase_alignment_min_{0.82};
  double source_fit_period_s_{1.20};
  double source_range_consistency_ratio_{0.60};
  double source_range_consistency_margin_m_{0.75};
  double source_absolute_median_limit_m_{0.90};
  double source_absolute_rms_limit_m_{1.50};
  double source_absolute_latest_margin_m_{0.75};
  double source_absolute_latest_ratio_{0.15};
  double tank_max_depth_m_{0.0};
  bool auto_source_depth_{false};
  bool depth_target_control_{false};
  double pinger_expected_depth_m_{0.0};
  double vehicle_target_depth_m_{0.0};
  double max_vehicle_depth_m_{0.0};
  double depth_soft_margin_m_{0.15};
  double depth_recovery_heave_{0.12};
  double probe_heave_{-0.18};
  double probe_heave_for_state_{0.0};
  double auto_probe_heave_magnitude_{0.10};
  double probe_duration_scale_{1.0};
  int minimum_probe_legs_{1};
  bool fast_probe_enabled_{false};
  double fast_probe_duration_s_{6.0};
  double fast_probe_forward_{0.30};
  double fast_probe_lateral_{0.08};
  double fast_probe_heave_{-0.12};
  double fast_probe_yaw_{0.28};
  double unknown_range_m_{12.0};
  double forward_mid_{0.34};
  double forward_near_{0.20};
  double forward_contact_{0.13};
  double align_forward_{0.0};
  double heave_gain_{0.75};
  double heave_limit_{0.38};
  double range_progress_delta_m_{0.12};
  double range_progress_timeout_s_{8.0};
  bool range_gradient_enabled_{false};
  bool continuous_range_excitation_enabled_{false};
  bool range_guidance_require_two_axis_{true};
  bool direction_vectoring_enabled_{false};
  double range_gradient_takeover_m_{9.0};
  double range_regression_margin_m_{0.60};
  double range_regression_hold_s_{1.20};
  double range_gradient_window_s_{7.0};
  double range_gradient_min_span_m_{0.20};
  double range_gradient_max_rms_m_{0.45};
  double range_gradient_max_age_s_{12.0};
  double range_feedback_probe_duration_s_{4.5};
  double range_gradient_forward_{0.80};
  double range_guidance_filter_alpha_{0.30};
  double range_guidance_max_step_rad_{radians(30.0)};
  double range_excitation_sway_{0.0};
  double range_excitation_period_s_{5.0};
  bool acoustic_position_enabled_{false};
  bool acoustic_position_control_enabled_{false};
  double acoustic_position_max_condition_{50.0};
  double acoustic_position_max_median_residual_m_{0.35};
  double acoustic_position_max_rms_residual_m_{0.85};
  double acoustic_position_max_age_s_{20.0};
  double acoustic_position_consistency_margin_m_{2.5};
  double acoustic_position_consistency_ratio_{0.35};
  double acoustic_position_sway_gain_{0.85};
  double acoustic_position_sway_limit_{0.60};
  double success_range_m_{0.0};
  double success_hold_s_{0.8};
  double arrival_radius_m_{0.8};
  double arrival_hold_s_{1.0};
  double first_lock_confirmation_radius_m_{3.0};
  double amplitude_range_constant_{0.325};
  bool source_xy_bounds_enabled_{false};
  double source_min_x_m_{-16.5};
  double source_max_x_m_{16.5};
  double source_min_y_m_{-14.0};
  double source_max_y_m_{14.0};
  double max_source_z_world_{-0.5};
  double pinger_min_submergence_m_{0.20};
  double rc_pwm_span_{400.0};
  bool rc_deadzone_compensation_enabled_{true};
  double rc_xy_deadzone_pwm_{30.0};
  double rc_yaw_deadzone_pwm_{40.0};
  double rc_heave_deadzone_pwm_{30.0};

  std::optional<Eigen::Vector3d> position_;
  std::optional<Eigen::Quaterniond> imu_orientation_;
  std::optional<double> depth_pose_z_;
  double qx_{0.0};
  double qy_{0.0};
  double qz_{0.0};
  double qw_{1.0};
  double yaw_rate_rad_s_{0.0};
  double imu_yaw_rate_rad_s_{0.0};
  std::optional<SteadyTime> yaw_settle_started_;
  bool connected_{false};
  bool armed_{false};
  std::string actual_vehicle_mode_;
  SteadyTime last_vehicle_state_{};
  SteadyTime last_connected_{};
  SteadyTime last_armed_{};
  SteadyTime last_odometry_{};
  SteadyTime last_motion_response_velocity_{};
  SteadyTime last_imu_{};
  SteadyTime last_depth_pose_{};
  SteadyTime last_audio_{};
  SteadyTime last_audio_quality_{};
  std::optional<double> last_audio_quality_value_;
  SteadyTime last_direction_{};
  SteadyTime last_fit_{};
  SteadyTime last_service_request_{};
  std::optional<Eigen::Vector3d> phase_direction_world_;
  std::optional<Eigen::Vector3d> input_direction_body_;
  std::optional<Eigen::Vector3d> no_odom_direction_world_;
  std::optional<Eigen::Vector3d> no_odom_probe_score_body_;
  std::array<double, 7> no_odom_probe_delta_sum_{};
  std::array<std::size_t, 7> no_odom_probe_delta_count_{};
  double no_odom_neutral_delta_sum_{0.0};
  std::size_t no_odom_neutral_delta_count_{0U};
  std::vector<NoOdomPhaseSample> no_odom_phase_samples_;
  double no_odom_probe_fit_rms_m_{std::numeric_limits<double>::infinity()};
  double no_odom_probe_fit_inlier_ratio_{0.0};
  int no_odom_probe_axis_{0};
  int no_odom_probe_active_leg_index_{-1};
  std::array<double, 12> no_odom_probe_leg_extensions_s_{};
  double no_odom_probe_active_leg_extension_s_{0.0};
  bool no_odom_probe_sample_enabled_{false};
  std::size_t no_odom_probe_count_{0U};
  std::optional<Eigen::Vector3d> no_odom_initial_candidate_world_;
  std::size_t no_odom_initial_consistent_count_{0U};
  double no_odom_initial_confirmation_dot_{
      std::numeric_limits<double>::quiet_NaN()};
  bool no_odom_initial_bearing_confirmed_{false};
  std::optional<SteadyTime> no_odom_forward_started_;
  double no_odom_forward_expected_delta_m_{std::numeric_limits<double>::quiet_NaN()};
  std::deque<NoOdomApproachSample> no_odom_approach_samples_;
  std::optional<SteadyTime> no_odom_innovation_bad_started_;
  std::optional<double> no_odom_innovation_ratio_;
  std::optional<double> no_odom_observed_delta_m_;
  int no_odom_innovation_reestimate_count_{0};
  int no_odom_watchdog_reestimate_count_{0};
  std::optional<double> motion_response_horizontal_speed_mps_;
  std::optional<SteadyTime> motion_response_command_started_;
  std::optional<SteadyTime> motion_response_low_speed_started_;
  double motion_response_requested_xy_{0.0};
  int motion_response_reestimate_count_{0};
  std::optional<Eigen::Vector3d> phase_bearing_lock_world_;
  std::size_t direction_update_count_{0U};
  std::optional<Eigen::Vector3d> filtered_direction_body_;
  std::deque<double> raw_delta_history_;
  std::deque<double> amplitude_range_history_;
  std::deque<RangeSample> samples_;
  std::deque<RangeSample> range_feedback_samples_;
  double cumulative_range_change_m_{0.0};
  std::optional<SourceEstimate> source_estimate_;
  std::optional<Eigen::Vector3d> source_smoothed_;
  std::optional<Eigen::Vector3d> source_locked_;
  bool last_force_fit_accepted_{false};
  std::string state_{"WAIT_VEHICLE"};
  SteadyTime state_started_{};
  std::optional<SteadyTime> active_started_;
  int probe_attempt_{0};
  int near_reprobe_count_{0};
  int inconsistent_lock_reprobe_count_{0};
  bool range_consistency_fallback_active_{false};
  bool probe_completed_{false};
  bool range_complete_{false};
  bool arrival_complete_{false};
  bool terminal_shutdown_requested_{false};
  bool terminal_brake_applied_{false};
  bool terminal_brake_aborted_{false};
  double terminal_brake_entry_forward_{0.0};
  std::string completion_reason_;
  std::optional<SteadyTime> range_success_started_;
  std::optional<SteadyTime> arrival_success_started_;
  Command last_requested_command_{};
  Command last_command_{};
  bool depth_limit_active_{false};
  bool depth_recovery_active_{false};
  std::optional<double> vehicle_depth_m_;
  double best_amplitude_distance_{std::numeric_limits<double>::infinity()};
  SteadyTime last_range_progress_{};
  std::optional<SteadyTime> range_regression_started_;
  std::optional<Eigen::Vector3d> range_guidance_world_;
  SteadyTime last_range_guidance_{};
  SteadyTime last_range_gradient_fit_{};
  bool range_gradient_recovery_active_{false};
  bool force_full_reprobe_{false};
  bool range_feedback_probe_completed_{false};
  bool range_gradient_two_axis_{false};
  double range_gradient_slope_{0.0};
  double range_gradient_span_m_{0.0};
  double range_gradient_rms_m_{0.0};
  double range_gradient_observability_{0.0};
  std::size_t range_gradient_sample_count_{0U};
  std::optional<Eigen::Vector2d> acoustic_position_candidate_xy_;
  std::optional<Eigen::Vector2d> acoustic_source_xy_;
  int acoustic_position_candidate_streak_{0};
  SteadyTime last_acoustic_position_attempt_{};
  SteadyTime last_acoustic_position_fit_{};
  double acoustic_position_condition_{0.0};
  double acoustic_position_median_residual_m_{0.0};
  double acoustic_position_rms_residual_m_{0.0};
  std::size_t acoustic_position_sample_count_{0U};
  std::string last_control_direction_source_{"unavailable"};
  double last_bearing_rad_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr motion_response_velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      depth_pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr delta_range_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr iq_magnitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr audio_quality_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr control_direction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direct_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingerHomingController>());
  rclcpp::shutdown();
  return 0;
}
