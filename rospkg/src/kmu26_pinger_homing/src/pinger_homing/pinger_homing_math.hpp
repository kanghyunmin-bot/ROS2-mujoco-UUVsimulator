#pragma once
// C++ range-difference fitting used by the deployed pinger controller.

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace kmu26::pinger_homing {

struct SourceEstimate {
  Eigen::Vector3d source_world{Eigen::Vector3d::Zero()};
  double initial_range_m{0.0};
  double bias_range_rate_mps{0.0};
  double rms_residual_m{std::numeric_limits<double>::infinity()};
  // Absolute amplitude-range residuals are deliberately reported separately
  // from the coherent phase/range-difference residual above.  A small phase
  // residual alone does not prove that the selected 3-D mirror branch is at
  // the measured distance.
  double absolute_median_residual_m{std::numeric_limits<double>::infinity()};
  double absolute_rms_residual_m{std::numeric_limits<double>::infinity()};
  double latest_absolute_error_m{std::numeric_limits<double>::infinity()};
  double latest_absolute_range_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t absolute_sample_count{0};
  double condition_number{std::numeric_limits<double>::infinity()};
  std::size_t sample_count{0};
};

struct DepthSafetyResult {
  double requested_heave{0.0};
  double command_heave{0.0};
  std::optional<double> vehicle_depth_m;
  bool limit_active{false};
  bool recovery_active{false};
};

struct LegacyProbeCommand {
  double forward{0.0};
  double lateral{0.0};
  double heave{0.0};
  double yaw{0.0};
  bool complete{false};
};

// A terminal brake is controller-only motion used after a successful
// no-odometry approach.  It does not change the acoustic estimator or its
// completion decision.  Keep the reverse demand deliberately smaller than a
// normal approach command so a bad duration cannot launch the vehicle back
// away from the pinger.
inline constexpr double kMaxNoOdomTerminalBrakeCommand = 0.35;

inline bool should_start_no_odom_terminal_brake(
    bool success,
    bool no_odom_navigation,
    bool enabled,
    double last_forward_command,
    double reverse_command,
    double duration_s) {
  return success && no_odom_navigation && enabled &&
         std::isfinite(last_forward_command) && last_forward_command > 1.0e-6 &&
         std::isfinite(reverse_command) && reverse_command > 0.0 &&
         std::isfinite(duration_s) && duration_s > 0.0;
}

inline double no_odom_terminal_brake_forward_command(
    double elapsed_s,
    double duration_s,
    double reverse_command) {
  if (!std::isfinite(elapsed_s) || !std::isfinite(duration_s) ||
      !std::isfinite(reverse_command) || elapsed_s < 0.0 ||
      duration_s <= 0.0 || elapsed_s >= duration_s || reverse_command <= 0.0) {
    return 0.0;
  }
  return -std::max(
      0.0, std::min(reverse_command, kMaxNoOdomTerminalBrakeCommand));
}

// A completed probe may promote only the fit produced by that completion
// callback.  `estimate_usable` can still describe a periodically accepted,
// cached estimate after the final force-fit has failed, so it is not a
// sufficient lock gate by itself.
inline bool completed_probe_fit_can_lock(
    bool force_fit_result,
    bool last_force_fit_accepted,
    bool estimate_usable) {
  return force_fit_result && last_force_fit_accepted && estimate_usable;
}

// Exact RC excitation timeline shipped by the Python controller in Git
// commit 83dafe8.  Neutral gaps are intentional: they separate the Phase
// EKF's direction components from its range-rate bias.
inline LegacyProbeCommand legacy_python_probe_command(
    double elapsed_s,
    double duration_scale,
    double probe_scale,
    double probe_heave,
    bool mirrored) {
  const double elapsed = std::max(0.0, elapsed_s) /
      std::max(duration_scale, 1.0e-9);
  const double lateral_sign = mirrored ? -1.0 : 1.0;
  if (elapsed < 1.2) return {};
  if (elapsed < 4.2) return {probe_scale, 0.0, 0.0, 0.0, false};
  if (elapsed < 4.8) return {};
  if (elapsed < 7.8) {
    return {0.0, lateral_sign * probe_scale, 0.0, 0.0, false};
  }
  if (elapsed < 8.4) return {};
  if (elapsed < 10.9) return {0.0, 0.0, probe_heave, 0.0, false};
  if (elapsed < 11.9) return {};
  return {0.0, 0.0, 0.0, 0.0, true};
}

enum class RangeTravelTrend {
  kApproaching,
  kReceding,
  kFlat,
};

// Controller-side range feedback.  This is deliberately independent from the
// hydrophone Phase/SNR estimators: those nodes still own the acoustic bearing,
// while this small fit only prevents the RC controller from continuing along a
// bearing after the measured amplitude range has started increasing.
struct AmplitudeDescentEstimate {
  Eigen::Vector3d direction_world{Eigen::Vector3d::Zero()};
  RangeTravelTrend trend{RangeTravelTrend::kFlat};
  double signed_slope_m_per_m{0.0};
  double horizontal_span_m{0.0};
  double rms_residual_m{std::numeric_limits<double>::infinity()};
  double observability_ratio{0.0};
  std::size_t sample_count{0};
  bool two_axis_observable{false};
};

struct AcousticPositionEstimate {
  Eigen::Vector2d source_xy_world{Eigen::Vector2d::Zero()};
  double condition_number{std::numeric_limits<double>::infinity()};
  double median_residual_m{std::numeric_limits<double>::infinity()};
  double rms_residual_m{std::numeric_limits<double>::infinity()};
  std::size_t sample_count{0};
};

inline double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(hi, value));
}

// An explicit neutral-centred PWM request is an operator/vehicle contract,
// not the normalized probe-scale tuning path. Preserve it down to one PWM
// count instead of applying the legacy 0.08 normalized lower bound (32 counts
// for the normal 400-count span). A zero/negative override deliberately keeps
// the existing normalized fallback, while the upper bound retains the
// controller's historical probe safety cap. Thus the real-vehicle 15--25
// count range and the simulator's explicit 90-count excitation are both exact.
inline double no_odom_probe_scale_from_pwm_override(
    int pwm_delta,
    double rc_pwm_span,
    double fallback_scale,
    double max_scale = 0.35) {
  if (pwm_delta <= 0 || !std::isfinite(rc_pwm_span) || rc_pwm_span <= 0.0) {
    return fallback_scale;
  }
  const double one_count_scale = 1.0 / rc_pwm_span;
  return clamp(
      static_cast<double>(pwm_delta) / rc_pwm_span,
      one_count_scale,
      max_scale);
}

// A no-odometry vehicle cannot use map position to reject a mirrored or
// startup-transient Phase bearing.  Before the first forward leg, require two
// independently completed ABBA probes to agree and follow their normalized
// mean.  Returning nullopt keeps the vehicle neutral and requests another
// probe; the acoustic estimator itself remains unchanged.
inline std::optional<Eigen::Vector3d> confirm_no_odom_bearing_pair(
    const Eigen::Vector3d &previous_world,
    const Eigen::Vector3d &current_world,
    double minimum_dot) {
  if (!previous_world.allFinite() || !current_world.allFinite() ||
      previous_world.norm() <= 1.0e-9 || current_world.norm() <= 1.0e-9) {
    return std::nullopt;
  }
  const Eigen::Vector3d previous = previous_world.normalized();
  const Eigen::Vector3d current = current_world.normalized();
  if (previous.dot(current) < clamp(minimum_dot, -1.0, 1.0)) {
    return std::nullopt;
  }
  const Eigen::Vector3d mean = previous + current;
  if (!mean.allFinite() || mean.norm() <= 1.0e-9) return std::nullopt;
  return mean.normalized();
}

inline double median(std::vector<double> values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  if ((values.size() % 2U) != 0U) return *middle;
  const double upper = *middle;
  const double lower = *std::max_element(values.begin(), middle);
  return 0.5 * (lower + upper);
}

inline double derive_vehicle_depth_limit(
    double tank_max_depth_m,
    double vehicle_bottom_extent_m = 0.17,
    double floor_clearance_m = 0.10) {
  if (!std::isfinite(tank_max_depth_m) || tank_max_depth_m <= 0.0) return 0.0;
  const double reserved = std::max(vehicle_bottom_extent_m, 0.0) +
                          std::max(floor_clearance_m, 0.0);
  return std::max(0.0, tank_max_depth_m - reserved);
}

inline double derive_pinger_depth_from_tank(
    double tank_max_depth_m,
    double pinger_depth_fraction = 0.80) {
  if (!std::isfinite(tank_max_depth_m) || tank_max_depth_m <= 0.0) return 0.0;
  return tank_max_depth_m * clamp(pinger_depth_fraction, 0.10, 0.98);
}

inline double derive_vehicle_target_depth_from_tank(
    double tank_max_depth_m,
    double vehicle_depth_fraction = 0.78) {
  if (!std::isfinite(tank_max_depth_m) || tank_max_depth_m <= 0.0) return 0.0;
  return std::min(
      derive_vehicle_depth_limit(tank_max_depth_m),
      tank_max_depth_m * clamp(vehicle_depth_fraction, 0.10, 0.98));
}

inline int effective_minimum_probe_legs(
    int configured_minimum_probe_legs,
    int completed_near_reprobes) {
  // The initial far-field solve keeps the configured mirrored legs.  Once a
  // range-stall has deliberately moved the vehicle close to the source, one
  // complete neutral-separated X/Y/Z leg is sufficient when its metric fit
  // passes every quality gate.  Repeating the initial deep-tank requirement
  // here wastes the remaining approach budget without adding a new axis.
  return completed_near_reprobes > 0
      ? 1 : std::max(1, configured_minimum_probe_legs);
}

inline double select_auto_probe_heave(
    const std::optional<double> &vehicle_depth_m,
    double max_vehicle_depth_m,
    double shallow_vehicle_depth_m = 0.45,
    double magnitude = 0.10) {
  if (!vehicle_depth_m || !std::isfinite(*vehicle_depth_m) ||
      !std::isfinite(max_vehicle_depth_m) || max_vehicle_depth_m <= 0.0) {
    return 0.0;
  }
  const double depth = std::max(0.0, *vehicle_depth_m);
  const double upward_room = std::max(0.0, depth - std::max(shallow_vehicle_depth_m, 0.0));
  const double downward_room = std::max(0.0, max_vehicle_depth_m - depth);
  const double command = downward_room >= upward_room ? -std::abs(magnitude) : std::abs(magnitude);
  return clamp(command, -0.4, 0.4);
}

inline DepthSafetyResult limit_heave_by_vehicle_depth(
    double requested_heave,
    const std::optional<double> &vehicle_z_world,
    double max_vehicle_depth_m,
    double soft_margin_m,
    double recovery_heave) {
  DepthSafetyResult result;
  result.requested_heave = clamp(requested_heave, -1.0, 1.0);
  result.command_heave = result.requested_heave;
  if (!std::isfinite(max_vehicle_depth_m) || max_vehicle_depth_m <= 0.0) return result;

  if (!vehicle_z_world || !std::isfinite(*vehicle_z_world)) {
    result.command_heave = std::max(result.requested_heave, 0.0);
    result.limit_active = result.command_heave != result.requested_heave;
    return result;
  }

  const double depth = std::max(0.0, -*vehicle_z_world);
  result.vehicle_depth_m = depth;
  const double recovery = clamp(recovery_heave, 0.0, 1.0);
  if (depth >= max_vehicle_depth_m) {
    result.command_heave = std::max(result.requested_heave, recovery);
    result.limit_active = true;
    result.recovery_active = recovery > 0.0;
    return result;
  }

  const double margin = std::max(soft_margin_m, 0.0);
  const double remaining = max_vehicle_depth_m - depth;
  if (result.requested_heave < 0.0 && margin > 0.0 && remaining < margin) {
    result.command_heave = result.requested_heave * clamp(remaining / margin, 0.0, 1.0);
    result.limit_active = result.command_heave != result.requested_heave;
  }
  return result;
}

inline Eigen::Vector3d filter_unit_vector(
    const std::optional<Eigen::Vector3d> &previous,
    const Eigen::Vector3d &current,
    double alpha) {
  if (!current.allFinite() || current.norm() <= 1.0e-9) {
    throw std::invalid_argument("current direction must be finite and non-zero");
  }
  const Eigen::Vector3d value = current.normalized();
  if (!previous || !previous->allFinite() || previous->norm() <= 1.0e-9) return value;
  const Eigen::Vector3d filtered =
      (1.0 - clamp(alpha, 0.0, 1.0)) * previous->normalized() +
      clamp(alpha, 0.0, 1.0) * value;
  return filtered.norm() <= 1.0e-9 ? value : filtered.normalized();
}

// Preserve the position/range localizer's vertical command while steering the
// vehicle's horizontal nose toward the fresh acoustic bearing.  The Phase
// estimator is substantially more accurate in azimuth than a weakly
// observable 3-D range fit, whereas the range fit is the safer depth source.
inline Eigen::Vector3d apply_acoustic_yaw_to_position_direction(
    const Eigen::Vector3d &position_direction_body,
    const Eigen::Vector3d &acoustic_direction_body,
    double minimum_acoustic_horizontal_norm = 0.05) {
  if (!position_direction_body.allFinite() ||
      position_direction_body.norm() <= 1.0e-9) {
    throw std::invalid_argument("position direction must be finite and non-zero");
  }
  const Eigen::Vector3d position = position_direction_body.normalized();
  if (!acoustic_direction_body.allFinite()) return position;
  const Eigen::Vector2d acoustic_xy = acoustic_direction_body.head<2>();
  if (acoustic_xy.norm() < std::max(0.0, minimum_acoustic_horizontal_norm)) {
    return position;
  }
  const double horizontal_magnitude =
      std::sqrt(std::max(0.0, 1.0 - position.z() * position.z()));
  if (horizontal_magnitude <= 1.0e-9) return position;
  Eigen::Vector3d result;
  result.head<2>() = horizontal_magnitude * acoustic_xy.normalized();
  result.z() = position.z();
  return result.normalized();
}

inline std::optional<AmplitudeDescentEstimate> estimate_amplitude_range_descent(
    const std::vector<Eigen::Vector3d> &positions_world,
    const std::vector<double> &ranges_m,
    const std::vector<double> &times_s,
    const std::optional<double> &source_z_world = std::nullopt,
    double recent_window_s = 7.0,
    double min_horizontal_span_m = 0.20,
    double min_abs_slope_m_per_m = 0.04,
    double ridge_ratio = 0.02) {
  if (positions_world.size() != ranges_m.size() ||
      positions_world.size() != times_s.size() || positions_world.size() < 8U) {
    return std::nullopt;
  }

  double latest_time = -std::numeric_limits<double>::infinity();
  double previous_time = -std::numeric_limits<double>::infinity();
  for (double time : times_s) {
    if (!std::isfinite(time) || time < previous_time) return std::nullopt;
    previous_time = time;
    latest_time = time;
  }
  if (!std::isfinite(latest_time)) return std::nullopt;

  std::vector<Eigen::Vector2d> xy;
  std::vector<double> horizontal_ranges;
  xy.reserve(positions_world.size());
  horizontal_ranges.reserve(positions_world.size());
  const double window_start = latest_time - std::max(recent_window_s, 0.0);
  for (std::size_t i = 0; i < positions_world.size(); ++i) {
    const auto &position = positions_world[i];
    const double measured_range = ranges_m[i];
    if (times_s[i] < window_start || !position.allFinite() ||
        !std::isfinite(measured_range) || measured_range <= 0.1 ||
        measured_range >= 80.0) {
      continue;
    }
    double horizontal_range = measured_range;
    if (source_z_world && std::isfinite(*source_z_world)) {
      const double vertical = *source_z_world - position.z();
      const double squared = measured_range * measured_range - vertical * vertical;
      if (squared <= 0.01) continue;
      horizontal_range = std::sqrt(squared);
    }
    xy.push_back(position.head<2>());
    horizontal_ranges.push_back(horizontal_range);
  }
  if (xy.size() < 8U) return std::nullopt;

  Eigen::Vector2d mean_xy = Eigen::Vector2d::Zero();
  double mean_range = 0.0;
  for (std::size_t i = 0; i < xy.size(); ++i) {
    mean_xy += xy[i];
    mean_range += horizontal_ranges[i];
  }
  mean_xy /= static_cast<double>(xy.size());
  mean_range /= static_cast<double>(xy.size());

  Eigen::MatrixXd design(static_cast<Eigen::Index>(xy.size()), 2);
  Eigen::VectorXd response(static_cast<Eigen::Index>(xy.size()));
  double horizontal_span = 0.0;
  for (std::size_t i = 0; i < xy.size(); ++i) {
    const Eigen::Vector2d centered = xy[i] - mean_xy;
    design.row(static_cast<Eigen::Index>(i)) = centered.transpose();
    response(static_cast<Eigen::Index>(i)) = horizontal_ranges[i] - mean_range;
    horizontal_span = std::max(horizontal_span, centered.norm());
  }
  if (!std::isfinite(horizontal_span) || horizontal_span < min_horizontal_span_m) {
    return std::nullopt;
  }

  const Eigen::Matrix2d unweighted_normal = design.transpose() * design;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(unweighted_normal);
  if (eigen_solver.info() != Eigen::Success) return std::nullopt;
  const double max_eigenvalue = std::max(eigen_solver.eigenvalues().maxCoeff(), 0.0);
  const double min_eigenvalue = std::max(eigen_solver.eigenvalues().minCoeff(), 0.0);
  if (max_eigenvalue <= 1.0e-9) return std::nullopt;
  const double observability_ratio = min_eigenvalue / max_eigenvalue;
  const bool two_axis_observable = observability_ratio >= 0.04;

  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  Eigen::VectorXd weights = Eigen::VectorXd::Ones(design.rows());
  for (int iteration = 0; iteration < 4; ++iteration) {
    const Eigen::Matrix2d normal =
        design.transpose() * weights.asDiagonal() * design;
    const double ridge = std::max(
        1.0e-8, std::max(ridge_ratio, 0.0) * unweighted_normal.trace() * 0.5);
    const Eigen::Matrix2d regularized = normal + ridge * Eigen::Matrix2d::Identity();
    gradient = regularized.ldlt().solve(
        design.transpose() * weights.asDiagonal() * response);
    if (!gradient.allFinite()) return std::nullopt;
    const Eigen::VectorXd residual = response - design * gradient;
    std::vector<double> absolute_residuals;
    absolute_residuals.reserve(static_cast<std::size_t>(residual.size()));
    for (Eigen::Index row = 0; row < residual.size(); ++row) {
      absolute_residuals.push_back(std::abs(residual(row)));
    }
    const double huber = std::max(0.05, 2.5 * median(absolute_residuals));
    for (Eigen::Index row = 0; row < residual.size(); ++row) {
      weights(row) = std::abs(residual(row)) <= huber
          ? 1.0 : huber / std::max(std::abs(residual(row)), 1.0e-12);
    }
  }

  const Eigen::VectorXd residual = response - design * gradient;
  const double rms = std::sqrt(residual.squaredNorm() /
                               static_cast<double>(residual.size()));
  if (!std::isfinite(rms) || !gradient.allFinite() || gradient.norm() > 2.0) {
    return std::nullopt;
  }

  const std::size_t group_count = std::max<std::size_t>(2U, xy.size() / 3U);
  Eigen::Vector2d early_xy = Eigen::Vector2d::Zero();
  Eigen::Vector2d late_xy = Eigen::Vector2d::Zero();
  std::vector<double> early_ranges;
  std::vector<double> late_ranges;
  early_ranges.reserve(group_count);
  late_ranges.reserve(group_count);
  for (std::size_t i = 0; i < group_count; ++i) {
    early_xy += xy[i];
    early_ranges.push_back(horizontal_ranges[i]);
    const std::size_t late_index = xy.size() - group_count + i;
    late_xy += xy[late_index];
    late_ranges.push_back(horizontal_ranges[late_index]);
  }
  early_xy /= static_cast<double>(group_count);
  late_xy /= static_cast<double>(group_count);
  const Eigen::Vector2d travel = late_xy - early_xy;
  const double travel_distance = travel.norm();
  const double signed_slope = travel_distance > 1.0e-6
      ? (median(late_ranges) - median(early_ranges)) / travel_distance : 0.0;
  RangeTravelTrend trend = RangeTravelTrend::kFlat;
  if (signed_slope < -min_abs_slope_m_per_m) trend = RangeTravelTrend::kApproaching;
  if (signed_slope > min_abs_slope_m_per_m) trend = RangeTravelTrend::kReceding;

  Eigen::Vector2d descent = Eigen::Vector2d::Zero();
  if (two_axis_observable && gradient.norm() >= min_abs_slope_m_per_m) {
    descent = -gradient.normalized();
  } else {
    // A straight leg cannot identify the cross-track component.  It can still
    // state safely whether continuing or reversing that exact leg decreases
    // range, which is enough for the bounded turnaround guard.
    if (travel_distance < min_horizontal_span_m || trend == RangeTravelTrend::kFlat) {
      return std::nullopt;
    }
    descent = trend == RangeTravelTrend::kApproaching
        ? travel.normalized() : -travel.normalized();
  }

  AmplitudeDescentEstimate estimate;
  estimate.direction_world = Eigen::Vector3d(descent.x(), descent.y(), 0.0);
  estimate.trend = trend;
  estimate.signed_slope_m_per_m = signed_slope;
  estimate.horizontal_span_m = horizontal_span;
  estimate.rms_residual_m = rms;
  estimate.observability_ratio = observability_ratio;
  estimate.sample_count = xy.size();
  estimate.two_axis_observable = two_axis_observable;
  return estimate;
}

// Fixed-depth amplitude multilateration used as a bounded RC safety guide.
// This consumes only the same calibrated IQ range already available to the
// controller plus filtered odometry; it does not alter either acoustic bearing
// estimator and never consumes simulator ground truth.
inline std::optional<AcousticPositionEstimate> estimate_acoustic_source_xy(
    const std::vector<Eigen::Vector3d> &positions_world,
    const std::vector<double> &slant_ranges_m,
    double source_z_world,
    double minimum_xy_spacing_m = 0.02) {
  if (positions_world.size() != slant_ranges_m.size() ||
      positions_world.size() < 12U || !std::isfinite(source_z_world)) {
    return std::nullopt;
  }

  std::vector<Eigen::Vector3d> positions;
  std::vector<double> ranges;
  positions.reserve(std::min<std::size_t>(positions_world.size(), 120U));
  ranges.reserve(std::min<std::size_t>(slant_ranges_m.size(), 120U));
  for (std::size_t i = 0; i < positions_world.size(); ++i) {
    if (!positions_world[i].allFinite() || !std::isfinite(slant_ranges_m[i]) ||
        slant_ranges_m[i] <= 0.1 || slant_ranges_m[i] >= 80.0) {
      continue;
    }
    if (!positions.empty() &&
        (positions_world[i].head<2>() - positions.back().head<2>()).norm() <
            std::max(minimum_xy_spacing_m, 0.0)) {
      continue;
    }
    positions.push_back(positions_world[i]);
    ranges.push_back(slant_ranges_m[i]);
  }
  if (positions.size() < 12U) return std::nullopt;
  if (positions.size() > 120U) {
    std::vector<Eigen::Vector3d> reduced_positions;
    std::vector<double> reduced_ranges;
    reduced_positions.reserve(120U);
    reduced_ranges.reserve(120U);
    for (std::size_t i = 0; i < 120U; ++i) {
      const std::size_t index = i * (positions.size() - 1U) / 119U;
      reduced_positions.push_back(positions[index]);
      reduced_ranges.push_back(ranges[index]);
    }
    positions = std::move(reduced_positions);
    ranges = std::move(reduced_ranges);
  }

  const auto &reference = positions.front();
  const double reference_dz = source_z_world - reference.z();
  const double reference_horizontal_sq =
      ranges.front() * ranges.front() - reference_dz * reference_dz;
  if (reference_horizontal_sq <= 0.01) return std::nullopt;
  Eigen::MatrixXd design(static_cast<Eigen::Index>(positions.size() - 1U), 2);
  Eigen::VectorXd measured(static_cast<Eigen::Index>(positions.size() - 1U));
  Eigen::Index row = 0;
  for (std::size_t i = 1; i < positions.size(); ++i) {
    const double dz = source_z_world - positions[i].z();
    const double horizontal_sq = ranges[i] * ranges[i] - dz * dz;
    if (horizontal_sq <= 0.01) continue;
    const Eigen::Vector2d delta = positions[i].head<2>() - reference.head<2>();
    design.row(row) = 2.0 * delta.transpose();
    measured(row) = positions[i].head<2>().squaredNorm() -
                    reference.head<2>().squaredNorm() - horizontal_sq +
                    reference_horizontal_sq;
    ++row;
  }
  if (row < 10) return std::nullopt;
  design.conservativeResize(row, Eigen::NoChange);
  measured.conservativeResize(row);
  const Eigen::JacobiSVD<Eigen::MatrixXd> linear_svd(
      design, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto linear_singular = linear_svd.singularValues();
  if (linear_singular.size() < 2 || !std::isfinite(linear_singular(0)) ||
      !std::isfinite(linear_singular(1)) || linear_singular(1) < 0.05) {
    return std::nullopt;
  }
  Eigen::Vector2d estimate = linear_svd.solve(measured);
  if (!estimate.allFinite() || estimate.norm() > 100.0) return std::nullopt;

  constexpr double kHuberScaleM = 0.20;
  constexpr double kDamping = 1.0e-3;
  for (int iteration = 0; iteration < 25; ++iteration) {
    Eigen::Matrix2d normal = Eigen::Matrix2d::Zero();
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
    for (std::size_t i = 0; i < positions.size(); ++i) {
      const double dz = source_z_world - positions[i].z();
      const Eigen::Vector2d horizontal = estimate - positions[i].head<2>();
      const double predicted = std::max(
          std::sqrt(horizontal.squaredNorm() + dz * dz), 1.0e-8);
      const double residual = predicted - ranges[i];
      const double weight = std::abs(residual) > kHuberScaleM
          ? kHuberScaleM / std::max(std::abs(residual), 1.0e-12) : 1.0;
      const Eigen::Vector2d jacobian = horizontal / predicted;
      normal += weight * jacobian * jacobian.transpose();
      gradient += weight * jacobian * residual;
    }
    const Eigen::Vector2d step =
        (normal + kDamping * Eigen::Matrix2d::Identity()).ldlt().solve(-gradient);
    if (!step.allFinite()) return std::nullopt;
    estimate += step;
    if (step.norm() < 1.0e-5) break;
  }
  if (!estimate.allFinite() || estimate.norm() > 100.0) return std::nullopt;

  std::vector<double> absolute_residuals;
  absolute_residuals.reserve(positions.size());
  Eigen::MatrixXd jacobian(static_cast<Eigen::Index>(positions.size()), 2);
  double squared_residual = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const double dz = source_z_world - positions[i].z();
    const Eigen::Vector2d horizontal = estimate - positions[i].head<2>();
    const double predicted = std::max(
        std::sqrt(horizontal.squaredNorm() + dz * dz), 1.0e-8);
    const double residual = predicted - ranges[i];
    absolute_residuals.push_back(std::abs(residual));
    squared_residual += residual * residual;
    jacobian.row(static_cast<Eigen::Index>(i)) = (horizontal / predicted).transpose();
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> refined_svd(
      jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto refined_singular = refined_svd.singularValues();
  if (refined_singular.size() < 2 || refined_singular(1) <= 1.0e-6) {
    return std::nullopt;
  }
  AcousticPositionEstimate result;
  result.source_xy_world = estimate;
  result.condition_number = refined_singular(0) / refined_singular(1);
  result.median_residual_m = median(absolute_residuals);
  result.rms_residual_m = std::sqrt(
      squared_residual / static_cast<double>(positions.size()));
  result.sample_count = positions.size();
  return result;
}

// Exact fixed-depth seed used by the validated Python Phase controller.  The
// sign of the spatial range gradient points away from the stationary source,
// so its negative supplies the horizontal bearing while the latest calibrated
// slant range supplies the distance.  This is only a seed for the robust
// absolute-range fit below; it is never exposed as a controller command.
inline std::optional<SourceEstimate> estimate_source_from_range_gradient(
    const std::vector<Eigen::Vector3d> &positions_world,
    const std::vector<double> &absolute_ranges_m,
    double source_z_world) {
  if (positions_world.size() != absolute_ranges_m.size() ||
      !std::isfinite(source_z_world)) {
    return std::nullopt;
  }

  std::vector<Eigen::Vector3d> positions;
  std::vector<double> ranges;
  positions.reserve(positions_world.size());
  ranges.reserve(absolute_ranges_m.size());
  for (std::size_t i = 0; i < positions_world.size(); ++i) {
    if (!positions_world[i].allFinite() || !std::isfinite(absolute_ranges_m[i]) ||
        absolute_ranges_m[i] <= 1.0 || absolute_ranges_m[i] >= 80.0) {
      continue;
    }
    positions.push_back(positions_world[i]);
    ranges.push_back(absolute_ranges_m[i]);
  }
  if (positions.size() < 20U) return std::nullopt;

  Eigen::Vector2d mean_xy = Eigen::Vector2d::Zero();
  double mean_range = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    mean_xy += positions[i].head<2>();
    mean_range += ranges[i];
  }
  mean_xy /= static_cast<double>(positions.size());
  mean_range /= static_cast<double>(positions.size());
  Eigen::MatrixXd centered_xy(static_cast<Eigen::Index>(positions.size()), 2);
  Eigen::VectorXd centered_ranges(static_cast<Eigen::Index>(positions.size()));
  for (std::size_t i = 0; i < positions.size(); ++i) {
    centered_xy.row(static_cast<Eigen::Index>(i)) =
        (positions[i].head<2>() - mean_xy).transpose();
    centered_ranges(static_cast<Eigen::Index>(i)) = ranges[i] - mean_range;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      centered_xy, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto singular = svd.singularValues();
  if (singular.size() < 2 || singular(1) < 0.05) return std::nullopt;
  const Eigen::Vector2d gradient = svd.solve(centered_ranges);
  const double gradient_norm = gradient.norm();
  if (!gradient.allFinite() || gradient_norm < 0.05) return std::nullopt;

  std::vector<double> recent_ranges;
  const std::size_t first_recent = ranges.size() > 9U ? ranges.size() - 9U : 0U;
  recent_ranges.assign(
      ranges.begin() + static_cast<std::ptrdiff_t>(first_recent), ranges.end());
  const double current_range = median(recent_ranges);
  const Eigen::Vector3d &current = positions.back();
  const double dz = source_z_world - current.z();
  const double horizontal_range =
      std::sqrt(std::max(current_range * current_range - dz * dz, 0.25));
  const Eigen::Vector2d direction = -gradient / gradient_norm;
  const Eigen::Vector3d source(
      current.x() + direction.x() * horizontal_range,
      current.y() + direction.y() * horizontal_range,
      source_z_world);

  std::vector<double> absolute_residuals;
  absolute_residuals.reserve(positions.size());
  double squared_residual = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const double residual = (source - positions[i]).norm() - ranges[i];
    absolute_residuals.push_back(std::abs(residual));
    squared_residual += residual * residual;
  }
  SourceEstimate result;
  result.source_world = source;
  result.initial_range_m = ranges.front();
  result.bias_range_rate_mps = 0.0;
  result.rms_residual_m =
      std::sqrt(squared_residual / static_cast<double>(positions.size()));
  result.absolute_median_residual_m = median(absolute_residuals);
  result.absolute_rms_residual_m = result.rms_residual_m;
  result.latest_absolute_error_m = absolute_residuals.back();
  result.latest_absolute_range_m = ranges.back();
  result.absolute_sample_count = positions.size();
  result.condition_number = singular(0) / singular(1);
  result.sample_count = positions.size();
  return result;
}

// Exact known-depth absolute-range solver from the validated Python
// controller.  Keeping this path ahead of the range-difference fallback is
// important: with a known pool depth it prevents a low-residual but displaced
// Phase solution from becoming the long-lived source lock.
inline std::optional<SourceEstimate> estimate_source_xy_from_absolute_ranges(
    const std::vector<Eigen::Vector3d> &positions_world,
    const std::vector<double> &absolute_ranges_m,
    double source_z_world,
    const std::optional<Eigen::Vector3d> &initial_source_world = std::nullopt,
    double huber_scale_m = 0.20) {
  if (positions_world.size() != absolute_ranges_m.size() ||
      !std::isfinite(source_z_world)) {
    return std::nullopt;
  }

  std::vector<Eigen::Vector3d> positions;
  std::vector<double> ranges;
  positions.reserve(positions_world.size());
  ranges.reserve(absolute_ranges_m.size());
  for (std::size_t i = 0; i < positions_world.size(); ++i) {
    if (!positions_world[i].allFinite() || !std::isfinite(absolute_ranges_m[i]) ||
        absolute_ranges_m[i] <= 0.1 || absolute_ranges_m[i] >= 80.0) {
      continue;
    }
    positions.push_back(positions_world[i]);
    ranges.push_back(absolute_ranges_m[i]);
  }
  if (positions.size() < 20U) return std::nullopt;

  const Eigen::Vector3d origin = positions.front();
  Eigen::MatrixXd displacement(static_cast<Eigen::Index>(positions.size()), 3);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    displacement.row(static_cast<Eigen::Index>(i)) =
        (positions[i] - origin).transpose();
  }
  const double z_local = source_z_world - origin.z();
  const Eigen::MatrixXd horizontal_span = displacement.leftCols(2);
  const Eigen::JacobiSVD<Eigen::MatrixXd> span_svd(
      horizontal_span, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto span_singular = span_svd.singularValues();
  if (span_singular.size() < 2 || span_singular(1) < 0.05) {
    return std::nullopt;
  }

  Eigen::MatrixXd matrix(static_cast<Eigen::Index>(positions.size() - 1U), 2);
  Eigen::VectorXd rhs(static_cast<Eigen::Index>(positions.size() - 1U));
  for (std::size_t i = 1; i < positions.size(); ++i) {
    const Eigen::Vector3d delta = displacement.row(static_cast<Eigen::Index>(i));
    matrix.row(static_cast<Eigen::Index>(i - 1U)) =
        (-2.0 * delta.head<2>()).transpose();
    rhs(static_cast<Eigen::Index>(i - 1U)) =
        ranges[i] * ranges[i] - ranges.front() * ranges.front() -
        delta.squaredNorm() + 2.0 * z_local * delta.z();
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> linear_svd(
      matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector2d xy = linear_svd.solve(rhs);
  if (!xy.allFinite()) return std::nullopt;
  if (initial_source_world && initial_source_world->allFinite()) {
    const Eigen::Vector3d seeded = *initial_source_world - origin;
    if (seeded.head<2>().norm() > 0.1) xy = seeded.head<2>();
  }

  constexpr double kDamping = 1.0e-3;
  const double scale = std::max(huber_scale_m, 0.03);
  for (int iteration = 0; iteration < 25; ++iteration) {
    Eigen::Matrix2d normal = Eigen::Matrix2d::Zero();
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
    for (std::size_t i = 0; i < positions.size(); ++i) {
      const Eigen::Vector3d source_local(xy.x(), xy.y(), z_local);
      const Eigen::Vector3d relative =
          source_local - displacement.row(static_cast<Eigen::Index>(i)).transpose();
      const double predicted = std::max(relative.norm(), 1.0e-8);
      const double residual = predicted - ranges[i];
      const double weight = std::abs(residual) > scale
          ? scale / std::max(std::abs(residual), 1.0e-12) : 1.0;
      const Eigen::Vector2d jacobian = relative.head<2>() / predicted;
      normal += weight * jacobian * jacobian.transpose();
      gradient += weight * jacobian * residual;
    }
    const Eigen::Vector2d step =
        (normal + kDamping * Eigen::Matrix2d::Identity()).ldlt().solve(-gradient);
    if (!step.allFinite()) return std::nullopt;
    xy += step;
    if (step.norm() < 1.0e-5) break;
  }

  const Eigen::Vector3d source_local(xy.x(), xy.y(), z_local);
  const Eigen::Vector3d source_world = origin + source_local;
  if (!source_world.allFinite() || source_world.norm() > 100.0) return std::nullopt;
  Eigen::MatrixXd jacobian(static_cast<Eigen::Index>(positions.size()), 2);
  std::vector<double> absolute_residuals;
  absolute_residuals.reserve(positions.size());
  double squared_residual = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const Eigen::Vector3d relative = source_world - positions[i];
    const double predicted = std::max(relative.norm(), 1.0e-8);
    const double residual = predicted - ranges[i];
    absolute_residuals.push_back(std::abs(residual));
    squared_residual += residual * residual;
    jacobian.row(static_cast<Eigen::Index>(i)) =
        (relative.head<2>() / predicted).transpose();
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> refined_svd(
      jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto refined_singular = refined_svd.singularValues();
  if (refined_singular.size() < 2 || refined_singular(1) <= 1.0e-9) {
    return std::nullopt;
  }

  SourceEstimate result;
  result.source_world = source_world;
  result.initial_range_m = ranges.front();
  result.bias_range_rate_mps = 0.0;
  result.rms_residual_m =
      std::sqrt(squared_residual / static_cast<double>(positions.size()));
  result.absolute_median_residual_m = median(absolute_residuals);
  result.absolute_rms_residual_m = result.rms_residual_m;
  result.latest_absolute_error_m = absolute_residuals.back();
  result.latest_absolute_range_m = ranges.back();
  result.absolute_sample_count = positions.size();
  result.condition_number = refined_singular(0) / refined_singular(1);
  result.sample_count = positions.size();
  return result;
}

inline double stabilized_yaw_command(
    double bearing_rad,
    double yaw_rate_rad_s,
    double previous_command,
    double dt_s,
    double gain,
    double rate_damping,
    double deadband_rad,
    double command_limit,
    double slew_rate_per_s) {
  if (!std::isfinite(bearing_rad) || !std::isfinite(yaw_rate_rad_s) ||
      !std::isfinite(previous_command)) {
    return 0.0;
  }
  // Preserve the validated Python controller exactly: inside the bearing
  // deadband its target is neutral, including the yaw-rate term.  The slew
  // limiter still ramps a previous command back to zero without introducing
  // the small opposite-yaw hunting that the C++-only D term caused.
  const bool inside_deadband =
      std::abs(bearing_rad) <= std::max(deadband_rad, 0.0);
  double target = inside_deadband
      ? 0.0 : gain * bearing_rad - rate_damping * yaw_rate_rad_s;
  const double limit = std::max(command_limit, 0.0);
  target = clamp(target, -limit, limit);
  const double step = std::max(slew_rate_per_s, 0.0) * std::max(dt_s, 0.0);
  return step <= 0.0 ? target : clamp(target, previous_command - step, previous_command + step);
}

inline double braking_yaw_command(
    double yaw_rate_rad_s,
    double previous_command,
    double dt_s,
    double brake_gain,
    double command_limit,
    double brake_slew_rate_per_s) {
  if (!std::isfinite(yaw_rate_rad_s) || !std::isfinite(previous_command)) {
    return 0.0;
  }
  const double limit = std::max(command_limit, 0.0);
  const double target = clamp(
      -std::max(brake_gain, 0.0) * yaw_rate_rad_s, -limit, limit);
  const double step =
      std::max(brake_slew_rate_per_s, 0.0) * std::max(dt_s, 0.0);
  return step <= 0.0
      ? target
      : clamp(target, previous_command - step, previous_command + step);
}

inline bool yaw_alignment_sample_settled(
    double bearing_rad,
    double yaw_rate_rad_s,
    double bearing_limit_rad,
    double yaw_rate_limit_rad_s) {
  return std::isfinite(bearing_rad) && std::isfinite(yaw_rate_rad_s) &&
         std::abs(bearing_rad) <= std::max(bearing_limit_rad, 0.0) &&
         std::abs(yaw_rate_rad_s) <= std::max(yaw_rate_limit_rad_s, 0.0);
}

inline Eigen::Vector3d world_vector_to_body_flu(
    const Eigen::Vector3d &vector_world,
    double qx,
    double qy,
    double qz,
    double qw) {
  Eigen::Quaterniond quaternion(qw, qx, qy, qz);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() <= 1.0e-9) {
    throw std::invalid_argument("zero-length quaternion");
  }
  quaternion.normalize();
  return quaternion.toRotationMatrix().transpose() * vector_world;
}

namespace detail {

inline void residual_and_jacobian(
    const Eigen::Matrix<double, 5, 1> &params,
    const Eigen::MatrixXd &displacement,
    const Eigen::VectorXd &changes,
    const Eigen::VectorXd &times,
    Eigen::VectorXd *residual,
    Eigen::MatrixXd *jacobian) {
  const Eigen::Index count = displacement.rows();
  residual->resize(count);
  jacobian->resize(count, 5);
  for (Eigen::Index row = 0; row < count; ++row) {
    const Eigen::Vector3d relative = params.head<3>() - displacement.row(row).transpose();
    const double range = std::max(relative.norm(), 1.0e-8);
    (*residual)(row) = range - params(3) + params(4) * times(row) - changes(row);
    jacobian->block<1, 3>(row, 0) = (relative / range).transpose();
    (*jacobian)(row, 3) = -1.0;
    (*jacobian)(row, 4) = times(row);
  }
}

inline double objective_cost(
    const Eigen::Matrix<double, 5, 1> &params,
    const Eigen::MatrixXd &displacement,
    const Eigen::VectorXd &changes,
    const Eigen::VectorXd &times,
    const Eigen::VectorXd &absolute_ranges,
    double huber_scale_m,
    double absolute_range_scale_m) {
  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian;
  residual_and_jacobian(params, displacement, changes, times, &residual, &jacobian);
  const double phase_scale = std::max(huber_scale_m, 1.0e-4);
  double cost = 0.0;
  for (Eigen::Index i = 0; i < residual.size(); ++i) {
    const double magnitude = std::abs(residual(i));
    cost += magnitude <= phase_scale
        ? 0.5 * residual(i) * residual(i)
        : phase_scale * (magnitude - 0.5 * phase_scale);
  }

  std::vector<double> initial_range_samples;
  const double abs_scale = std::max(absolute_range_scale_m, 0.05);
  const double threshold = 2.0 * abs_scale;
  for (Eigen::Index i = 0; i < absolute_ranges.size(); ++i) {
    if (!std::isfinite(absolute_ranges(i)) || absolute_ranges(i) <= 0.1 ||
        absolute_ranges(i) >= 80.0) {
      continue;
    }
    const Eigen::Vector3d relative = params.head<3>() - displacement.row(i).transpose();
    const double absolute_residual = relative.norm() - absolute_ranges(i);
    const double magnitude = std::abs(absolute_residual);
    cost += (magnitude <= threshold
        ? 0.5 * absolute_residual * absolute_residual
        : threshold * (magnitude - 0.5 * threshold)) / (abs_scale * abs_scale);
    initial_range_samples.push_back(absolute_ranges(i) - changes(i));
  }
  if (initial_range_samples.size() >= 8U) {
    const double amplitude_initial = median(initial_range_samples);
    cost += 0.5 * std::pow((params(3) - amplitude_initial) / abs_scale, 2.0);
  }
  cost += 0.5 * std::pow(params(4) / 0.04, 2.0);
  return cost;
}

}  // namespace detail

inline std::optional<SourceEstimate> estimate_source_from_range_differences(
    const std::vector<Eigen::Vector3d> &positions_world,
    const std::vector<double> &cumulative_range_changes_m,
    const std::vector<double> &times_s,
    const std::optional<Eigen::Vector3d> &initial_source_world = std::nullopt,
    const std::vector<double> &absolute_ranges_m = {},
    const std::optional<double> &min_source_z_world = std::nullopt,
    const std::optional<double> &max_source_z_world = std::nullopt,
    const std::optional<double> &fixed_source_z_world = std::nullopt,
    double huber_scale_m = 0.005,
    double absolute_range_scale_m = 0.45) {
  const std::size_t count = positions_world.size();
  if (count < 12U || cumulative_range_changes_m.size() != count || times_s.size() != count) {
    return std::nullopt;
  }
  const Eigen::Vector3d origin = positions_world.front();
  if (!origin.allFinite()) return std::nullopt;

  Eigen::MatrixXd displacement(static_cast<Eigen::Index>(count), 3);
  Eigen::VectorXd changes(static_cast<Eigen::Index>(count));
  Eigen::VectorXd times(static_cast<Eigen::Index>(count));
  Eigen::VectorXd absolute = Eigen::VectorXd::Constant(
      static_cast<Eigen::Index>(count), std::numeric_limits<double>::quiet_NaN());
  double maximum_displacement = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    if (!positions_world[i].allFinite() || !std::isfinite(cumulative_range_changes_m[i]) ||
        !std::isfinite(times_s[i])) {
      return std::nullopt;
    }
    const Eigen::Vector3d local = positions_world[i] - origin;
    displacement.row(static_cast<Eigen::Index>(i)) = local.transpose();
    changes(static_cast<Eigen::Index>(i)) =
        cumulative_range_changes_m[i] - cumulative_range_changes_m.front();
    times(static_cast<Eigen::Index>(i)) = times_s[i] - times_s.front();
    maximum_displacement = std::max(maximum_displacement, local.norm());
    if (absolute_ranges_m.size() == count) {
      absolute(static_cast<Eigen::Index>(i)) = absolute_ranges_m[i];
    }
  }
  if (maximum_displacement < 0.08) return std::nullopt;

  Eigen::MatrixXd linear_matrix(static_cast<Eigen::Index>(count - 1U), 4);
  Eigen::VectorXd linear_rhs(static_cast<Eigen::Index>(count - 1U));
  for (std::size_t i = 1; i < count; ++i) {
    const Eigen::Index row = static_cast<Eigen::Index>(i - 1U);
    linear_matrix.block<1, 3>(row, 0) =
        (-2.0 * displacement.row(static_cast<Eigen::Index>(i))).eval();
    linear_matrix(row, 3) = -2.0 * changes(static_cast<Eigen::Index>(i));
    linear_rhs(row) = std::pow(changes(static_cast<Eigen::Index>(i)), 2.0) -
                      displacement.row(static_cast<Eigen::Index>(i)).squaredNorm();
  }
  const Eigen::Vector4d linear = linear_matrix.colPivHouseholderQr().solve(linear_rhs);
  Eigen::Matrix<double, 5, 1> params = Eigen::Matrix<double, 5, 1>::Zero();
  params.head<3>() = linear.head<3>();
  params(3) = std::abs(linear(3));

  std::vector<double> amplitude_initial_samples;
  for (Eigen::Index i = 0; i < absolute.size(); ++i) {
    if (std::isfinite(absolute(i)) && absolute(i) > 0.1 && absolute(i) < 80.0) {
      amplitude_initial_samples.push_back(absolute(i) - changes(i));
    }
  }
  const std::optional<double> amplitude_initial = amplitude_initial_samples.empty()
      ? std::nullopt : std::optional<double>(median(amplitude_initial_samples));
  if (initial_source_world && initial_source_world->allFinite()) {
    Eigen::Vector3d seed = *initial_source_world - origin;
    if (seed.norm() > 0.1) {
      if (amplitude_initial) {
        seed.normalize();
        params.head<3>() = seed * *amplitude_initial;
      } else if (!params.head<3>().allFinite() || params.head<3>().norm() < 0.5 ||
                 params.head<3>().dot(seed) < 0.0) {
        params.head<3>() = seed;
      }
    }
  }
  if (!params.head<3>().allFinite() || params.head<3>().norm() < 0.5 ||
      params.head<3>().norm() > 80.0) {
    params.head<3>() = Eigen::Vector3d(12.0, 0.0, -6.0);
  }
  if (!std::isfinite(params(3)) || params(3) < 0.1 || params(3) > 80.0) {
    params(3) = params.head<3>().norm();
  }
  if (amplitude_initial) params(3) = *amplitude_initial;

  const std::optional<double> min_z = min_source_z_world
      ? std::optional<double>(*min_source_z_world - origin.z()) : std::nullopt;
  const std::optional<double> max_z = max_source_z_world
      ? std::optional<double>(*max_source_z_world - origin.z()) : std::nullopt;
  const std::optional<double> fixed_z = fixed_source_z_world
      ? std::optional<double>(*fixed_source_z_world - origin.z()) : std::nullopt;
  if (min_z && max_z && *min_z > *max_z) {
    throw std::invalid_argument("min_source_z_world must not exceed max_source_z_world");
  }
  if (min_z) params(2) = std::max(params(2), *min_z);
  if (max_z) params(2) = std::min(params(2), *max_z);
  if (fixed_z) params(2) = *fixed_z;

  std::vector<int> active{0, 1, 2, 3, 4};
  if (fixed_z) active = {0, 1, 3, 4};
  double damping = 1.0e-3;
  for (int iteration = 0; iteration < 30; ++iteration) {
    Eigen::VectorXd residual;
    Eigen::MatrixXd jacobian;
    detail::residual_and_jacobian(params, displacement, changes, times, &residual, &jacobian);
    const double scale = std::max(huber_scale_m, 1.0e-4);
    Eigen::VectorXd weights = Eigen::VectorXd::Ones(residual.size());
    for (Eigen::Index i = 0; i < residual.size(); ++i) {
      if (std::abs(residual(i)) > scale) {
        weights(i) = scale / std::max(std::abs(residual(i)), 1.0e-12);
      }
    }
    const Eigen::MatrixXd weighted_jacobian = weights.cwiseSqrt().asDiagonal() * jacobian;
    const Eigen::VectorXd weighted_residual = weights.cwiseSqrt().asDiagonal() * residual;
    Eigen::Matrix<double, 5, 5> normal = weighted_jacobian.transpose() * weighted_jacobian;
    Eigen::Matrix<double, 5, 1> gradient = weighted_jacobian.transpose() * weighted_residual;

    std::vector<Eigen::Index> valid_absolute;
    std::vector<double> amplitude_prior_samples;
    for (Eigen::Index i = 0; i < absolute.size(); ++i) {
      if (std::isfinite(absolute(i)) && absolute(i) > 0.1 && absolute(i) < 80.0) {
        valid_absolute.push_back(i);
        amplitude_prior_samples.push_back(absolute(i) - changes(i));
      }
    }
    if (valid_absolute.size() >= 8U) {
      const double abs_scale = std::max(absolute_range_scale_m, 0.05);
      const double prior_weight = 1.0 / (abs_scale * abs_scale);
      const double amplitude_range = median(amplitude_prior_samples);
      normal(3, 3) += prior_weight;
      gradient(3) += (params(3) - amplitude_range) * prior_weight;
      for (const Eigen::Index i : valid_absolute) {
        const Eigen::Vector3d relative = params.head<3>() - displacement.row(i).transpose();
        const double predicted = std::max(relative.norm(), 1.0e-8);
        const double absolute_residual = predicted - absolute(i);
        const double threshold = 2.0 * abs_scale;
        const double robust = std::abs(absolute_residual) > threshold
            ? threshold / std::max(std::abs(absolute_residual), 1.0e-12) : 1.0;
        Eigen::Matrix<double, 5, 1> row = Eigen::Matrix<double, 5, 1>::Zero();
        row.head<3>() = relative / predicted;
        const double weight = robust / (abs_scale * abs_scale);
        normal += weight * row * row.transpose();
        gradient += weight * row * absolute_residual;
      }
    }
    constexpr double bias_sigma_mps = 0.04;
    normal(4, 4) += 1.0 / (bias_sigma_mps * bias_sigma_mps);
    gradient(4) += params(4) / (bias_sigma_mps * bias_sigma_mps);

    Eigen::MatrixXd active_normal(active.size(), active.size());
    Eigen::VectorXd active_gradient(active.size());
    for (std::size_t row = 0; row < active.size(); ++row) {
      active_gradient(static_cast<Eigen::Index>(row)) = gradient(active[row]);
      for (std::size_t column = 0; column < active.size(); ++column) {
        active_normal(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
            normal(active[row], active[column]);
      }
    }
    active_normal += damping * Eigen::MatrixXd::Identity(active.size(), active.size());
    const Eigen::VectorXd active_step = active_normal.ldlt().solve(-active_gradient);
    if (!active_step.allFinite()) return std::nullopt;
    Eigen::Matrix<double, 5, 1> step = Eigen::Matrix<double, 5, 1>::Zero();
    for (std::size_t i = 0; i < active.size(); ++i) {
      step(active[i]) = active_step(static_cast<Eigen::Index>(i));
    }
    Eigen::Matrix<double, 5, 1> trial = params + step;
    if (min_z) trial(2) = std::max(trial(2), *min_z);
    if (max_z) trial(2) = std::min(trial(2), *max_z);
    if (fixed_z) trial(2) = *fixed_z;
    trial(3) = clamp(trial(3), 0.10, 80.0);
    trial(4) = clamp(trial(4), -0.50, 0.50);
    const double current_cost = detail::objective_cost(
        params, displacement, changes, times, absolute, huber_scale_m, absolute_range_scale_m);
    const double trial_cost = detail::objective_cost(
        trial, displacement, changes, times, absolute, huber_scale_m, absolute_range_scale_m);
    if (std::isfinite(trial_cost) && trial_cost <= current_cost) {
      params = trial;
      damping = std::max(1.0e-7, damping * 0.4);
      if (step.norm() < 1.0e-5) break;
    } else {
      damping = std::min(1.0e3, damping * 8.0);
    }
  }

  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian;
  detail::residual_and_jacobian(params, displacement, changes, times, &residual, &jacobian);
  std::vector<Eigen::Index> valid_absolute;
  for (Eigen::Index i = 0; i < absolute.size(); ++i) {
    if (std::isfinite(absolute(i)) && absolute(i) > 0.1 && absolute(i) < 80.0) {
      valid_absolute.push_back(i);
    }
  }
  const bool include_absolute_information = valid_absolute.size() >= 8U;
  const Eigen::Index extra_rows = include_absolute_information
      ? static_cast<Eigen::Index>(valid_absolute.size() + 1U) : 0;
  Eigen::MatrixXd full_information = Eigen::MatrixXd::Zero(
      jacobian.rows() + extra_rows, 5);
  full_information.topRows(jacobian.rows()) = jacobian;
  if (include_absolute_information) {
    const double abs_scale = std::max(absolute_range_scale_m, 0.05);
    full_information(jacobian.rows(), 3) = 1.0 / abs_scale;
    Eigen::Index row = jacobian.rows() + 1;
    for (const Eigen::Index sample_index : valid_absolute) {
      const Eigen::Vector3d relative =
          params.head<3>() - displacement.row(sample_index).transpose();
      const double range = std::max(relative.norm(), 1.0e-8);
      full_information.block<1, 3>(row++, 0) =
          (relative / (range * abs_scale)).transpose();
    }
  }
  Eigen::MatrixXd information(
      full_information.rows(), static_cast<Eigen::Index>(active.size()));
  for (std::size_t i = 0; i < active.size(); ++i) {
    information.col(static_cast<Eigen::Index>(i)) = full_information.col(active[i]);
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      information, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto singular = svd.singularValues();
  double condition = std::numeric_limits<double>::infinity();
  if (singular.size() > 0 && singular(singular.size() - 1) > 1.0e-9) {
    condition = singular(0) / singular(singular.size() - 1);
  }
  SourceEstimate estimate;
  estimate.source_world = origin + params.head<3>();
  if (!estimate.source_world.allFinite()) return std::nullopt;
  estimate.initial_range_m = params(3);
  estimate.bias_range_rate_mps = params(4);
  estimate.rms_residual_m = std::sqrt(residual.squaredNorm() / static_cast<double>(count));
  std::vector<double> absolute_magnitudes;
  absolute_magnitudes.reserve(valid_absolute.size());
  double absolute_squared_sum = 0.0;
  for (const Eigen::Index sample_index : valid_absolute) {
    const double predicted =
        (params.head<3>() - displacement.row(sample_index).transpose()).norm();
    const double absolute_error = predicted - absolute(sample_index);
    const double magnitude = std::abs(absolute_error);
    absolute_magnitudes.push_back(magnitude);
    absolute_squared_sum += absolute_error * absolute_error;
    estimate.latest_absolute_error_m = magnitude;
    estimate.latest_absolute_range_m = absolute(sample_index);
  }
  estimate.absolute_sample_count = absolute_magnitudes.size();
  if (!absolute_magnitudes.empty()) {
    estimate.absolute_median_residual_m = median(absolute_magnitudes);
    estimate.absolute_rms_residual_m = std::sqrt(
        absolute_squared_sum / static_cast<double>(absolute_magnitudes.size()));
  }
  estimate.condition_number = condition;
  estimate.sample_count = count;
  return estimate;
}

}  // namespace kmu26::pinger_homing
