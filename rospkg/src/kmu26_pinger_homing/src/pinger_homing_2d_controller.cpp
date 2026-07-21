#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace kmu26_pinger_homing {

class FingerHomingController final : public rclcpp::Node {
 public:
  FingerHomingController() : Node("pinger_homing_2d_controller") {
    mode_ = declare_parameter<std::string>("mode", "ALT_HOLD");
    if (mode_ != "ALT_HOLD") throw std::invalid_argument("pinger homing requires ALT_HOLD");
    estimator_mode_ = declare_parameter<std::string>("estimator_mode", "phase");
    if (estimator_mode_ != "phase" && estimator_mode_ != "snr") {
      throw std::invalid_argument("estimator_mode must be phase or snr");
    }
    odom_topic_ = declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/mavros/imu/data");
    use_imu_yaw_ = declare_parameter<bool>("use_imu_yaw", true);
    imu_timeout_s_ = std::clamp(declare_parameter<double>("imu_timeout_s", 0.5), 0.05, 5.0);
    rc_topic_ = declare_parameter<std::string>(
        "rc_output_topic", "/control/pinger/rc_override");
    selected_topic_ = declare_parameter<std::string>(
        "selected_frequency_topic", "/pinger_homing/selected_frequency_hz");
    required_mode_ = declare_parameter<std::string>("required_vehicle_mode", "ALT_HOLD");
    rc_span_ = std::clamp(declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);
    probe_command_ = std::clamp(declare_parameter<double>("probe_command", 0.20), 0.05, 0.45);
    forward_command_ = std::clamp(declare_parameter<double>("forward_command", 0.28), 0.05, 0.65);
    lateral_command_ = std::clamp(declare_parameter<double>("lateral_command", 0.22), 0.05, 0.5);
    probe_leg_s_ = std::clamp(declare_parameter<double>("probe_leg_s", 0.8), 0.3, 3.0);
    probe_neutral_s_ = std::clamp(declare_parameter<double>("probe_neutral_s", 0.3), 0.1, 2.0);
    probe_settle_s_ = std::clamp(declare_parameter<double>("probe_settle_s", 0.5), 0.1, 2.0);
    // A full ABBA probe establishes the first bearing.  During APPROACH we
    // continue to fit a rolling phase/SNR gradient, so this is only a
    // periodic reconditioning probe rather than the sole feedback source.
    reprobe_s_ = std::clamp(declare_parameter<double>("reprobe_s", 12.0), 2.0, 30.0);
    feedback_update_s_ = std::clamp(declare_parameter<double>("feedback_update_s", 0.5), 0.15, 3.0);
    feedback_window_s_ = std::clamp(declare_parameter<double>("feedback_window_s", 6.0), 2.0, 20.0);
    feedback_blend_ = std::clamp(declare_parameter<double>("feedback_blend", 0.25), 0.05, 0.8);
    min_xy_span_m_ = std::clamp(declare_parameter<double>("min_xy_span_m", 0.08), 0.02, 0.5);
    approach_dither_command_ = std::clamp(
        declare_parameter<double>("approach_dither_command", 0.06), 0.0, 0.20);
    approach_dither_period_s_ = std::clamp(
        declare_parameter<double>("approach_dither_period_s", 0.9), 0.3, 3.0);
    success_range_m_ = std::max(0.0, declare_parameter<double>("success_range_m", 1.2));
    range_constant_ = std::max(0.0, declare_parameter<double>("amplitude_range_constant", 0.325));
    min_snr_db_ = declare_parameter<double>("min_snr_db", 3.0);
    min_samples_ = std::max(8, static_cast<int>(declare_parameter<int>("min_samples", 16)));
    yaw_gain_ = std::clamp(declare_parameter<double>("yaw_gain", 0.65), 0.05, 2.0);
    yaw_rate_gain_ = std::clamp(declare_parameter<double>("yaw_rate_gain", 0.25), 0.0, 1.0);
    yaw_limit_ = std::clamp(declare_parameter<double>("yaw_limit", 0.30), 0.05, 0.7);
    align_rad_ = declare_parameter<double>("align_deg", 18.0) * M_PI / 180.0;
    max_runtime_s_ = std::max(10.0, declare_parameter<double>("max_runtime_s", 180.0));
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    auto_mode_ = declare_parameter<bool>("auto_mode", false);
    dry_run_ = declare_parameter<bool>("dry_run", false);
    // MAVROS/ArduSub yaw is NED/right-positive, while the acoustic direction
    // is published in ROS base_link FLU (left-positive).  Keep the logical
    // controller command in FLU and apply this transport conversion only at
    // RC4.  This matches the established Python controller contract.
    invert_rc_yaw_ = declare_parameter<bool>("invert_rc_yaw", true);

    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/pinger_homing/status", 10);
    direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/pinger_homing/direction", 10);
    gui_direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/homing/direction", 10);
    viewer_direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/pinger_homing/direction_body", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(*msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); });
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
          armed_ = msg->armed; vehicle_mode_ = msg->mode; last_state_ = now();
        });
    frequency_sub_ = create_subscription<std_msgs::msg::Float64>(
        selected_topic_, rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (std::isfinite(msg->data) && msg->data > 1000.0) {
            selected_ = true;
            selected_hz_ = msg->data;
            active_started_ = Clock::now();
          }
        });
    delta_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/pinger_homing/delta_range_m", 30,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { on_delta(msg->data); });
    iq_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/pinger_homing/iq_magnitude", 30,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { if (std::isfinite(msg->data)) iq_=msg->data; });
    snr_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/pinger_homing/iq_snr_db", 30,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { on_snr(msg->data); });
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this]() { tick(); });
    state_started_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),
                "standalone 2-D %s homing ready: ALT_HOLD, rc=%s, imu=%s, rc4_yaw_invert=%s",
                estimator_mode_.c_str(), rc_topic_.c_str(), imu_topic_.c_str(),
                invert_rc_yaw_ ? "true" : "false");
  }

 private:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  struct Obs { double x{0.0}, y{0.0}, t{0.0}, value{0.0}; };
  enum class State { WAIT_FREQUENCY, WAIT_VEHICLE, PROBE, ALIGN, APPROACH, SUCCESS, FAILED };

  static double elapsed(Time from) {
    return std::chrono::duration<double>(Clock::now() - from).count();
  }
  static int pwm(double command, double span) {
    return static_cast<int>(std::llround(1500.0 + std::clamp(command, -1.0, 1.0) * span));
  }
  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void on_odom(const nav_msgs::msg::Odometry &msg) {
    x_ = msg.pose.pose.position.x; y_ = msg.pose.pose.position.y;
    odom_yaw_ = yaw_from_quaternion(msg.pose.pose.orientation);
    odom_yaw_rate_ = msg.twist.twist.angular.z;
    have_odom_ = true; last_odom_ = now();
    align_imu_yaw_to_odom();
  }
  void on_imu(const sensor_msgs::msg::Imu &msg) {
    const auto &q = msg.orientation;
    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm_sq) || norm_sq < 1.0e-8) return;
    imu_yaw_ = yaw_from_quaternion(q);
    imu_yaw_rate_ = std::isfinite(msg.angular_velocity.z) ? msg.angular_velocity.z : 0.0;
    have_imu_ = true; last_imu_ = now();
    align_imu_yaw_to_odom();
  }
  void align_imu_yaw_to_odom() {
    if (!have_imu_ || !have_odom_ || imu_yaw_aligned_) return;
    // Positions in the phase regression are odometry-frame values.  IMU yaw
    // may use a different global zero (or NED/ENU convention), so capture the
    // constant offset once and then use live Pixhawk attitude for body-frame
    // steering.
    imu_to_odom_yaw_offset_ = wrap_pi(odom_yaw_ - imu_yaw_);
    imu_yaw_aligned_ = true;
  }
  static double wrap_pi(double value) {
    while (value > M_PI) value -= 2.0 * M_PI;
    while (value < -M_PI) value += 2.0 * M_PI;
    return value;
  }
  bool imu_yaw_fresh() {
    return use_imu_yaw_ && have_imu_ && imu_yaw_aligned_ &&
           (now() - last_imu_).seconds() <= imu_timeout_s_;
  }
  double body_yaw() {
    return imu_yaw_fresh() ? wrap_pi(imu_yaw_ + imu_to_odom_yaw_offset_) : odom_yaw_;
  }
  double body_yaw_rate() {
    return imu_yaw_fresh() ? imu_yaw_rate_ : odom_yaw_rate_;
  }
  const char *attitude_source() {
    return imu_yaw_fresh() ? "mavros_imu_aligned" : "odometry_fallback";
  }
  bool records_observation() const {
    return state_ == State::PROBE || state_ == State::APPROACH;
  }
  void trim_observations(std::deque<Obs> &samples, double time_s) {
    while (!samples.empty() && time_s - samples.front().t > feedback_window_s_) samples.pop_front();
    while (samples.size() > 500U) samples.pop_front();
  }
  void on_delta(double value) {
    if (!std::isfinite(value)) return;
    last_audio_ = now();
    cumulative_range_ += std::clamp(value, -0.15, 0.15);
    if (records_observation() && have_odom_) {
      const double time_s = now().seconds();
      obs_.push_back({x_, y_, time_s, cumulative_range_});
      trim_observations(obs_, time_s);
    }
  }
  void on_snr(double value) {
    if (!std::isfinite(value) || value < min_snr_db_) return;
    last_snr_ = value;
    if (records_observation() && have_odom_) {
      const double time_s = now().seconds();
      snr_obs_.push_back({x_, y_, time_s, value});
      trim_observations(snr_obs_, time_s);
    }
  }

  void transition(State next) {
    state_ = next; state_started_ = Clock::now();
    if (next == State::PROBE) {
      obs_.clear(); snr_obs_.clear(); cumulative_range_ = 0.0; feedback_updates_ = 0;
    }
    RCLCPP_INFO(get_logger(), "pinger homing state -> %s", state_name(next).c_str());
  }
  static std::string state_name(State state) {
    switch (state) {
      case State::WAIT_FREQUENCY: return "WAIT_FREQUENCY";
      case State::WAIT_VEHICLE: return "WAIT_VEHICLE";
      case State::PROBE: return "PROBE_2D";
      case State::ALIGN: return "ALIGN";
      case State::APPROACH: return "APPROACH";
      case State::SUCCESS: return "SUCCESS";
      default: return "FAILED";
    }
  }

  std::optional<Eigen::Vector2d> fit_direction() const {
    const auto &samples = estimator_mode_ == "snr" ? snr_obs_ : obs_;
    if (samples.size() < static_cast<std::size_t>(min_samples_)) return std::nullopt;
    Eigen::MatrixXd A(static_cast<Eigen::Index>(samples.size()), 4);
    Eigen::VectorXd b(static_cast<Eigen::Index>(samples.size()));
    for (std::size_t i = 0; i < samples.size(); ++i) {
      A(static_cast<Eigen::Index>(i), 0) = 1.0;
      A(static_cast<Eigen::Index>(i), 1) = samples[i].t;
      A(static_cast<Eigen::Index>(i), 2) = samples[i].x;
      A(static_cast<Eigen::Index>(i), 3) = samples[i].y;
      b(static_cast<Eigen::Index>(i)) = samples[i].value;
    }
    Eigen::VectorXd fit = A.colPivHouseholderQr().solve(b);
    if (estimator_mode_ == "snr") {
      // SNR data can contain short multipath spikes.  Use a small Huber IRLS
      // pass, matching the robust-gradient contract of the reference SNR
      // implementation while keeping this package self contained.
      for (int iteration = 0; iteration < 4; ++iteration) {
        const Eigen::VectorXd residual = b - A * fit;
        std::vector<double> abs_residual;
        abs_residual.reserve(static_cast<std::size_t>(residual.size()));
        for (Eigen::Index i = 0; i < residual.size(); ++i) {
          abs_residual.push_back(std::abs(residual(i)));
        }
        std::nth_element(abs_residual.begin(), abs_residual.begin() + abs_residual.size() / 2,
                         abs_residual.end());
        const double scale = std::max(1.0e-3, 1.4826 * abs_residual[abs_residual.size() / 2]);
        Eigen::VectorXd weights(residual.size());
        for (Eigen::Index i = 0; i < residual.size(); ++i) {
          const double u = std::abs(residual(i)) / (1.5 * scale);
          weights(i) = u <= 1.0 ? 1.0 : 1.0 / u;
        }
        Eigen::MatrixXd weighted_a = A;
        Eigen::VectorXd weighted_b = b;
        for (Eigen::Index i = 0; i < residual.size(); ++i) {
          const double root = std::sqrt(std::max(weights(i), 1.0e-4));
          weighted_a.row(i) *= root;
          weighted_b(i) *= root;
        }
        fit = weighted_a.colPivHouseholderQr().solve(weighted_b);
      }
    }
    if (!fit.allFinite()) return std::nullopt;
    Eigen::Vector2d direction = estimator_mode_ == "snr"
        ? Eigen::Vector2d(fit(2), fit(3))
        : Eigen::Vector2d(-fit(2), -fit(3));
    if (!direction.allFinite() || direction.norm() < 1.0e-5) return std::nullopt;
    return direction.normalized();
  }

  bool has_xy_observability() const {
    const auto &samples = estimator_mode_ == "snr" ? snr_obs_ : obs_;
    if (samples.size() < static_cast<std::size_t>(min_samples_)) return false;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto &sample : samples) {
      min_x = std::min(min_x, sample.x); max_x = std::max(max_x, sample.x);
      min_y = std::min(min_y, sample.y); max_y = std::max(max_y, sample.y);
    }
    // A rolling fit needs movement in both world-plane axes.  The small
    // dither during approach provides this without leaving the homing path.
    return (max_x - min_x) >= min_xy_span_m_ && (max_y - min_y) >= min_xy_span_m_;
  }

  bool update_direction_from_feedback(bool initial) {
    if (!has_xy_observability()) return false;
    const auto measured = fit_direction();
    if (!measured) return false;
    if (!direction_world_ || initial) {
      direction_world_ = *measured;
      ++feedback_updates_;
      return true;
    }
    const double agreement = direction_world_->dot(*measured);
    // Reject one-window 180-degree phase/multipath flips.  A clean periodic
    // ABBA probe will reinitialise the estimate if the environment changes.
    if (agreement < -0.20) return false;
    *direction_world_ = ((1.0 - feedback_blend_) * (*direction_world_) +
                         feedback_blend_ * (*measured)).normalized();
    ++feedback_updates_;
    return true;
  }

  void probe_command(double t, double &forward, double &lateral) const {
    forward = lateral = 0.0;
    double rem = t - probe_settle_s_;
    if (rem < 0.0) return;
    const int axes[] = {1, -1, -1, 1, 2, -2, -2, 2};
    for (const int axis : axes) {
      if (rem < probe_leg_s_) {
        if (axis == 1) forward = probe_command_;
        if (axis == -1) forward = -probe_command_;
        if (axis == 2) lateral = probe_command_;
        if (axis == -2) lateral = -probe_command_;
        return;
      }
      rem -= probe_leg_s_;
      if (rem < probe_neutral_s_) return;
      rem -= probe_neutral_s_;
    }
  }
  double probe_duration() const { return probe_settle_s_ + 8.0 * (probe_leg_s_ + probe_neutral_s_); }

  void tick() {
    if (state_ == State::SUCCESS || state_ == State::FAILED) {
      publish_release();
      publish_status();
      return;
    }
    if (!selected_) { transition_if_needed(State::WAIT_FREQUENCY); publish_neutral(); publish_status(); return; }
    if (!have_odom_ || (now() - last_odom_).seconds() > 1.0) {
      transition_if_needed(State::WAIT_VEHICLE); publish_neutral(); publish_status(); return;
    }
    if (auto_mode_ && vehicle_mode_ != required_mode_) {
      publish_neutral(); publish_status(); return;
    }
    if (!dry_run_ && auto_arm_ && !armed_) { publish_neutral(); publish_status(); return; }
    if (elapsed(active_started_) > max_runtime_s_) { transition(State::FAILED); publish_release(); publish_status(); return; }
    if (state_ == State::WAIT_FREQUENCY || state_ == State::WAIT_VEHICLE) transition(State::PROBE);
    if (state_ == State::PROBE) {
      if (elapsed(state_started_) >= probe_duration()) {
        if (!update_direction_from_feedback(true)) {
          transition(State::FAILED); publish_release(); publish_status(); return;
        }
        transition(State::ALIGN);
      } else {
        double f = 0.0, l = 0.0; probe_command(elapsed(state_started_), f, l); publish_command(f, l, 0.0); publish_status(); return;
      }
    }
    if (range_constant_ > 0.0 && iq_ > 1.0e-9 &&
        range_constant_ / std::sqrt(iq_) <= success_range_m_) {
      transition(State::SUCCESS); publish_release(); publish_status(); return;
    }
    if (!direction_world_) { transition(State::PROBE); publish_neutral(); return; }
    if (state_ == State::APPROACH && elapsed(last_feedback_update_) >= feedback_update_s_) {
      update_direction_from_feedback(false);
      last_feedback_update_ = Clock::now();
    }
    const double yaw_world_body = body_yaw();
    const double forward_axis = std::cos(yaw_world_body) * direction_world_->x() +
                                std::sin(yaw_world_body) * direction_world_->y();
    const double lateral_axis = -std::sin(yaw_world_body) * direction_world_->x() +
                                std::cos(yaw_world_body) * direction_world_->y();
    const double bearing = std::atan2(lateral_axis, forward_axis);
    const double yaw = std::clamp(yaw_gain_ * bearing - yaw_rate_gain_ * body_yaw_rate(),
                                  -yaw_limit_, yaw_limit_);
    if (std::abs(bearing) > align_rad_) {
      transition_if_needed(State::ALIGN); publish_command(0.0, 0.0, yaw); publish_status(); return;
    }
    if (state_ == State::ALIGN) transition(State::APPROACH);
    if (elapsed(state_started_) > reprobe_s_) { transition(State::PROBE); publish_neutral(); publish_status(); return; }
    const double dither_phase = std::fmod(elapsed(state_started_), 2.0 * approach_dither_period_s_);
    const double dither = approach_dither_command_ > 0.0
        ? (dither_phase < approach_dither_period_s_ ? approach_dither_command_ : -approach_dither_command_)
        : 0.0;
    publish_command(std::clamp(forward_command_ * std::max(0.0, forward_axis), 0.0, forward_command_),
                    std::clamp(lateral_command_ * lateral_axis + dither,
                               -lateral_command_, lateral_command_), yaw);
    publish_status();
  }

  void transition_if_needed(State next) { if (state_ != next) transition(next); }
  rclcpp::Time now() { return get_clock()->now(); }
  void publish_command(double forward, double lateral, double yaw) {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    msg.channels.fill(1500);
    msg.channels[2] = 1500;  // ALT_HOLD owns heave; this package never commands Z.
    msg.channels[3] = static_cast<uint16_t>(pwm(invert_rc_yaw_ ? -yaw : yaw, rc_span_));
    msg.channels[4] = static_cast<uint16_t>(pwm(forward, rc_span_));
    msg.channels[5] = static_cast<uint16_t>(pwm(lateral, rc_span_));
    last_forward_ = forward; last_lateral_ = lateral; last_yaw_ = yaw;
    if (!dry_run_) rc_pub_->publish(msg);
  }
  void publish_neutral() { publish_command(0.0, 0.0, 0.0); }
  void publish_release() {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    if (!dry_run_) rc_pub_->publish(msg);
  }
  void publish_status() {
    std::ostringstream json;
    json << "{\"state\":\"" << state_name(state_) << "\",\"mode\":\"" << mode_
         << "\",\"estimator\":\"" << estimator_mode_ << "\",\"selected_hz\":"
         << (selected_ ? selected_hz_ : 0.0) << ",\"direction\":["
         << (direction_world_ ? (*direction_world_).x() : 0.0) << ","
         << (direction_world_ ? (*direction_world_).y() : 0.0) << ",0],\"rc\":["
         << last_forward_ << "," << last_lateral_ << "," << last_yaw_ << "],\"dry_run\":"
         << (dry_run_ ? "true" : "false") << ",\"attitude_source\":\""
         << attitude_source() << "\",\"rc4_yaw_inverted\":"
         << (invert_rc_yaw_ ? "true" : "false") << ",\"feedback_updates\":"
         << feedback_updates_ << ",\"feedback_window_s\":" << feedback_window_s_ << "}";
    std_msgs::msg::String msg; msg.data = json.str(); status_pub_->publish(msg);
    if (direction_world_) {
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.stamp = now(); direction.header.frame_id = "odom";
      direction.vector.x = direction_world_->x(); direction.vector.y = direction_world_->y(); direction.vector.z = 0.0;
      direction_pub_->publish(direction);

      // The UUV Control GUI displays /homing/direction as a base_link FLU
      // vector.  Keep /pinger_homing/direction in odom for the controller
      // contract and publish this explicit visualization alias as well.
      geometry_msgs::msg::Vector3Stamped gui_direction;
      gui_direction.header.stamp = direction.header.stamp;
      gui_direction.header.frame_id = "base_link";
      const double yaw_world_body = body_yaw();
      gui_direction.vector.x = std::cos(yaw_world_body) * direction_world_->x() +
                               std::sin(yaw_world_body) * direction_world_->y();
      gui_direction.vector.y = -std::sin(yaw_world_body) * direction_world_->x() +
                               std::cos(yaw_world_body) * direction_world_->y();
      gui_direction.vector.z = 0.0;
      gui_direction_pub_->publish(gui_direction);
      viewer_direction_pub_->publish(gui_direction);
    }
  }

  std::string mode_, estimator_mode_, odom_topic_, imu_topic_, rc_topic_, selected_topic_, required_mode_;
  double rc_span_{400.0}, probe_command_{0.2}, forward_command_{0.28}, lateral_command_{0.22};
  double probe_leg_s_{0.8}, probe_neutral_s_{0.3}, probe_settle_s_{0.5}, reprobe_s_{12.0};
  double feedback_update_s_{0.5}, feedback_window_s_{6.0}, feedback_blend_{0.25}, min_xy_span_m_{0.08};
  double approach_dither_command_{0.06}, approach_dither_period_s_{0.9};
  double success_range_m_{1.2}, range_constant_{0.325}, min_snr_db_{3.0};
  int min_samples_{16};
  double yaw_gain_{0.65}, yaw_rate_gain_{0.25}, yaw_limit_{0.30}, align_rad_{0.31}, max_runtime_s_{180.0};
  bool auto_arm_{false}, auto_mode_{false}, dry_run_{false}, invert_rc_yaw_{true}, selected_{false}, armed_{false}, have_odom_{false};
  bool use_imu_yaw_{true}, have_imu_{false}, imu_yaw_aligned_{false};
  double imu_timeout_s_{0.5};
  double selected_hz_{0.0}, x_{0.0}, y_{0.0}, odom_yaw_{0.0}, odom_yaw_rate_{0.0}, imu_yaw_{0.0}, imu_yaw_rate_{0.0}, imu_to_odom_yaw_offset_{0.0}, cumulative_range_{0.0}, iq_{0.0}, last_snr_{0.0};
  double last_forward_{0.0}, last_lateral_{0.0}, last_yaw_{0.0};
  std::optional<Eigen::Vector2d> direction_world_;
  std::deque<Obs> obs_, snr_obs_;
  int feedback_updates_{0};
  State state_{State::WAIT_FREQUENCY}; Time state_started_{}, active_started_{Clock::now()}, last_feedback_update_{Clock::now()};
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME}, last_imu_{0, 0, RCL_ROS_TIME}, last_audio_{0, 0, RCL_ROS_TIME}, last_state_{0, 0, RCL_ROS_TIME};
  std::string vehicle_mode_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr frequency_sub_, delta_sub_, iq_sub_, snr_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gui_direction_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr viewer_direction_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kmu26_pinger_homing

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kmu26_pinger_homing::FingerHomingController>());
  rclcpp::shutdown();
  return 0;
}
