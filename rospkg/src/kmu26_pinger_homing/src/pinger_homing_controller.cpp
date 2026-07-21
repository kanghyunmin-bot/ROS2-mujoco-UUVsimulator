#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kRcNeutral = 1500;
constexpr int kRcNoChange = 0;
constexpr std::size_t kPrimaryRcChannelCount = 8;
constexpr std::size_t kChPitch = 1;
constexpr std::size_t kChHeave = 2;
constexpr std::size_t kChYaw = 3;
constexpr std::size_t kChForward = 4;
constexpr std::size_t kChSway = 5;

double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(hi, value));
}

double wrap_pi(double value) {
  while (value > kPi) value -= 2.0 * kPi;
  while (value < -kPi) value += 2.0 * kPi;
  return value;
}

bool finite3(double x, double y, double z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

std::optional<double> json_number(const std::string &text, const std::string &key) {
  const std::string quoted_key = "\"" + key + "\"";
  const auto key_pos = text.find(quoted_key);
  if (key_pos == std::string::npos) return std::nullopt;
  const auto colon = text.find(':', key_pos + quoted_key.size());
  if (colon == std::string::npos) return std::nullopt;
  const char *begin = text.c_str() + colon + 1;
  while (*begin != '\0' && std::isspace(static_cast<unsigned char>(*begin))) ++begin;
  char *end = nullptr;
  const double value = std::strtod(begin, &end);
  if (end == begin || !std::isfinite(value)) return std::nullopt;
  return value;
}

std::optional<bool> json_bool(const std::string &text, const std::string &key) {
  const std::string quoted_key = "\"" + key + "\"";
  const auto key_pos = text.find(quoted_key);
  if (key_pos == std::string::npos) return std::nullopt;
  const auto colon = text.find(':', key_pos + quoted_key.size());
  if (colon == std::string::npos) return std::nullopt;
  auto pos = colon + 1;
  while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) ++pos;
  if (text.compare(pos, 4, "true") == 0) return true;
  if (text.compare(pos, 5, "false") == 0) return false;
  return std::nullopt;
}

int axis_pwm(double value, bool invert, double span) {
  const double axis = invert ? -value : value;
  return static_cast<int>(std::llround(kRcNeutral + clamp(axis, -1.0, 1.0) * span));
}

std::string bool_text(bool value) {
  return value ? "true" : "false";
}

struct Command {
  double forward{0.0};
  double sway{0.0};
  double heave{0.0};
  double yaw{0.0};
  double pitch{0.0};
  bool capture{false};
};

struct YoloGuidance {
  bool valid{false};
  bool active{false};
  int count{0};
  double forward{0.0};
  double sway{0.0};
  double heave{0.0};
  double yaw{0.0};
  bool capture{false};
  std::chrono::steady_clock::time_point received{};
};

}  // namespace

class PingerHomingController : public rclcpp::Node {
 public:
  PingerHomingController()
      : Node("pinger_homing_controller") {
    transport_ = declare_parameter<std::string>("transport", "command_override");
    direction_topic_ = declare_parameter<std::string>("direction_topic", "/mujoco/hydrophone/direction");
    hydrophone_status_topic_ =
        declare_parameter<std::string>("hydrophone_status_topic", "/mujoco/hydrophone/status");
    yolo_topic_ = declare_parameter<std::string>("yolo_topic", "/uuv_mujoco/yolo_buoy_detections");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");
    command_override_topic_ =
        declare_parameter<std::string>("command_override_topic", "/uuv_mujoco/sitl/command_override");
    rc_override_topic_ = declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
    status_topic_ = declare_parameter<std::string>("status_topic", "/uuv_mujoco/pinger_homing/status");
    mode_ = declare_parameter<std::string>("mode", "MANUAL");

    rate_hz_ = clamp(declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
    forward_fast_ = clamp(declare_parameter<double>("forward_fast", 0.90), 0.0, 1.0);
    forward_mid_ = clamp(declare_parameter<double>("forward_mid", 0.58), 0.0, 1.0);
    forward_slow_ = clamp(declare_parameter<double>("forward_slow", 0.24), 0.0, 1.0);
    yaw_gain_ = clamp(declare_parameter<double>("yaw_gain", 1.15), 0.0, 4.0);
    yaw_limit_ = clamp(declare_parameter<double>("yaw_limit", 0.72), 0.0, 1.0);
    heave_gain_ = clamp(declare_parameter<double>("heave_gain", 0.42), 0.0, 2.0);
    heave_limit_ = clamp(declare_parameter<double>("heave_limit", 0.38), 0.0, 1.0);
    sway_gain_ = clamp(declare_parameter<double>("sway_gain", 0.0), 0.0, 2.0);
    sway_limit_ = clamp(declare_parameter<double>("sway_limit", 0.0), 0.0, 1.0);
    center_deadband_rad_ = clamp(declare_parameter<double>("center_deadband_rad", 0.055), 0.0, 0.50);
    lost_timeout_s_ = clamp(declare_parameter<double>("lost_timeout_s", 2.50), 0.05, 5.0);
    yolo_timeout_s_ = clamp(declare_parameter<double>("yolo_timeout_s", 0.50), 0.05, 5.0);
    yolo_switch_range_m_ = clamp(declare_parameter<double>("yolo_switch_range_m", 0.30), 0.02, 5.0);
    stop_range_m_ = clamp(declare_parameter<double>("stop_range_m", 0.0), 0.0, 5.0);
    rc_pwm_span_ = clamp(declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);
    use_yolo_final_ = declare_parameter<bool>("use_yolo_final", true);
    auto_arm_ = declare_parameter<bool>("auto_arm", true);
    auto_mode_ = declare_parameter<bool>("auto_mode", true);
    invert_rc_heave_ = declare_parameter<bool>("invert_rc_heave", true);
    invert_rc_yaw_ = declare_parameter<bool>("invert_rc_yaw", true);

    const auto telemetry_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        direction_topic_, telemetry_qos,
        [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { on_direction(msg); });
    hydrophone_status_sub_ = create_subscription<std_msgs::msg::String>(
        hydrophone_status_topic_, telemetry_qos,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_hydrophone_status(msg); });
    yolo_sub_ = create_subscription<std_msgs::msg::String>(
        yolo_topic_, telemetry_qos,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_yolo(msg); });
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        state_topic_, telemetry_qos,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
          last_state_ = *msg;
          have_state_ = true;
        });

    direct_pub_ = create_publisher<std_msgs::msg::String>(command_override_topic_, rclcpp::QoS(10));
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { on_timer(); });

    RCLCPP_INFO(
        get_logger(),
        "pinger homing ready transport=%s direction=%s status=%s yolo=%s state=%s rate=%.1fHz",
        transport_.c_str(), direction_topic_.c_str(), hydrophone_status_topic_.c_str(), yolo_topic_.c_str(),
        state_topic_.c_str(), rate_hz_);
  }

  ~PingerHomingController() override {
    publish_neutral("shutdown");
  }

 private:
  void on_direction(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    const double x = msg->vector.x;
    const double y = msg->vector.y;
    const double z = msg->vector.z;
    if (!finite3(x, y, z)) return;
    const double norm = std::sqrt(x * x + y * y + z * z);
    if (norm < 1.0e-6) return;
    dir_x_ = x / norm;
    dir_y_ = y / norm;
    dir_z_ = z / norm;
    have_direction_ = true;
    last_direction_ = std::chrono::steady_clock::now();
  }

  void on_hydrophone_status(const std_msgs::msg::String::SharedPtr msg) {
    const auto range = json_number(msg->data, "range_m");
    if (range) {
      range_m_ = *range;
      last_range_ = std::chrono::steady_clock::now();
    }
  }

  void on_yolo(const std_msgs::msg::String::SharedPtr msg) {
    YoloGuidance y;
    y.valid = true;
    y.received = std::chrono::steady_clock::now();
    y.active = json_bool(msg->data, "active").value_or(false);
    y.count = static_cast<int>(std::llround(json_number(msg->data, "count").value_or(0.0)));
    y.forward = clamp(json_number(msg->data, "forward").value_or(0.0), -1.0, 1.0);
    y.sway = clamp(json_number(msg->data, "sway").value_or(0.0), -1.0, 1.0);
    y.heave = clamp(json_number(msg->data, "heave").value_or(0.0), -1.0, 1.0);
    y.yaw = clamp(json_number(msg->data, "yaw").value_or(0.0), -1.0, 1.0);
    y.capture = json_bool(msg->data, "capture").value_or(false);
    yolo_ = y;
  }

  void on_timer() {
    maybe_request_arm_mode();
    const auto now = std::chrono::steady_clock::now();
    const double dir_age = have_direction_ ? age_s(last_direction_, now) : 1.0e9;
    if (!have_direction_ || dir_age > lost_timeout_s_) {
      publish_neutral("SEARCH_HYDROPHONE");
      publish_status("SEARCH_HYDROPHONE", Command{}, dir_age, false);
      return;
    }

    const double range_age = std::isfinite(range_m_) ? age_s(last_range_, now) : 1.0e9;
    const bool range_fresh = std::isfinite(range_m_) && range_age < 1.5;
    if (range_fresh && stop_range_m_ > 0.0 && range_m_ <= stop_range_m_) {
      publish_neutral("ARRIVED");
      publish_status("ARRIVED", Command{}, dir_age, false);
      return;
    }

    const bool yolo_ready = use_yolo_final_ && range_fresh && range_m_ <= yolo_switch_range_m_ && yolo_.valid &&
                            yolo_.active && yolo_.count > 0 && age_s(yolo_.received, now) <= yolo_timeout_s_;
    Command cmd = yolo_ready ? yolo_command() : hydrophone_command();
    const std::string phase = yolo_ready ? "YOLO_FINAL_ALIGN" : "HYDROPHONE_HOMING";
    publish_command(cmd, phase);
    publish_status(phase, cmd, dir_age, yolo_ready);
  }

  Command hydrophone_command() const {
    Command cmd;
    const double bearing = wrap_pi(std::atan2(dir_y_, dir_x_));
    const double yaw_abs = std::abs(bearing);
    cmd.yaw = yaw_abs <= center_deadband_rad_ ? 0.0 : clamp(yaw_gain_ * bearing, -yaw_limit_, yaw_limit_);
    cmd.heave = clamp(-heave_gain_ * dir_z_, -heave_limit_, heave_limit_);
    cmd.sway = clamp(sway_gain_ * dir_y_, -sway_limit_, sway_limit_);

    double forward = forward_fast_;
    if (yaw_abs > 1.10) forward = std::min(forward, forward_slow_);
    else if (yaw_abs > 0.72) forward = std::min(forward, forward_mid_);
    else if (yaw_abs > 0.42) forward = std::min(forward, std::max(forward_mid_, 0.62));
    if (dir_x_ < 0.05) forward = std::min(forward, 0.18);
    if (std::isfinite(range_m_) && range_m_ <= yolo_switch_range_m_) {
      forward = std::min(forward, forward_slow_);
    }
    cmd.forward = clamp(forward, 0.0, 1.0);
    return cmd;
  }

  Command yolo_command() const {
    Command cmd;
    cmd.forward = yolo_.capture ? std::max(0.22, yolo_.forward) : yolo_.forward;
    cmd.sway = yolo_.sway;
    cmd.heave = yolo_.heave;
    cmd.yaw = yolo_.yaw;
    cmd.capture = yolo_.capture;
    return cmd;
  }

  void maybe_request_arm_mode() {
    const auto now = std::chrono::steady_clock::now();
    if (age_s(last_service_request_, now) < 1.0) return;
    last_service_request_ = now;
    if (auto_arm_ && (!have_state_ || !last_state_.armed)) {
      if (arm_client_->service_is_ready()) {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        arm_client_->async_send_request(request);
      }
    }
    if (auto_mode_ && !mode_.empty() && (!have_state_ || last_state_.mode != mode_)) {
      if (mode_client_->service_is_ready()) {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode_;
        mode_client_->async_send_request(request);
      }
    }
  }

  void publish_command(const Command &cmd, const std::string &phase) {
    if (transport_ == "rc_override") {
      publish_rc(cmd);
      return;
    }
    std_msgs::msg::String msg;
    msg.data = direct_payload(cmd, phase);
    direct_pub_->publish(msg);
  }

  void publish_neutral(const std::string &phase) {
    publish_command(Command{}, phase);
  }

  void publish_rc(const Command &cmd) {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels.fill(kRcNoChange);
    for (std::size_t i = 0; i < kPrimaryRcChannelCount; ++i) {
      msg.channels[i] = kRcNeutral;
    }
    msg.channels[kChPitch] = static_cast<uint16_t>(axis_pwm(cmd.pitch, false, rc_pwm_span_));
    msg.channels[kChHeave] = static_cast<uint16_t>(axis_pwm(cmd.heave, invert_rc_heave_, rc_pwm_span_));
    msg.channels[kChYaw] = static_cast<uint16_t>(axis_pwm(cmd.yaw, invert_rc_yaw_, rc_pwm_span_));
    msg.channels[kChForward] = static_cast<uint16_t>(axis_pwm(cmd.forward, false, rc_pwm_span_));
    msg.channels[kChSway] = static_cast<uint16_t>(axis_pwm(cmd.sway, false, rc_pwm_span_));
    rc_pub_->publish(msg);
  }

  std::string direct_payload(const Command &cmd, const std::string &phase) const {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "{\"source\":\"pinger_homing\",\"phase\":\"" << phase << "\",\"direct_cmd\":{\"forward\":"
        << clamp(cmd.forward, -1.0, 1.0) << ",\"sway\":" << clamp(cmd.sway, -1.0, 1.0)
        << ",\"heave\":" << clamp(cmd.heave, -1.0, 1.0) << ",\"yaw\":" << clamp(cmd.yaw, -1.0, 1.0)
        << ",\"pitch\":" << clamp(cmd.pitch, -1.0, 1.0) << "}}";
    return out.str();
  }

  void publish_status(const std::string &phase, const Command &cmd, double dir_age, bool yolo_ready) {
    std_msgs::msg::String msg;
    const double bearing = have_direction_ ? wrap_pi(std::atan2(dir_y_, dir_x_)) : 0.0;
    std::ostringstream out;
    out << std::fixed << std::setprecision(4)
        << "{\"running\":true,\"phase\":\"" << phase << "\",\"transport\":\"" << transport_
        << "\",\"direction_age_s\":" << dir_age << ",\"bearing_rad\":" << bearing
        << ",\"direction_body\":[" << dir_x_ << "," << dir_y_ << "," << dir_z_ << "]";
    if (std::isfinite(range_m_)) {
      out << ",\"range_m\":" << range_m_;
    } else {
      out << ",\"range_m\":null";
    }
    out << ",\"yolo_ready\":" << bool_text(yolo_ready)
        << ",\"command\":{\"forward\":" << cmd.forward << ",\"sway\":" << cmd.sway
        << ",\"heave\":" << cmd.heave << ",\"yaw\":" << cmd.yaw << ",\"capture\":"
        << bool_text(cmd.capture) << "}}";
    msg.data = out.str();
    status_pub_->publish(msg);
  }

  static double age_s(
      const std::chrono::steady_clock::time_point &stamp,
      const std::chrono::steady_clock::time_point &now) {
    if (stamp.time_since_epoch().count() == 0) return 1.0e9;
    return std::chrono::duration<double>(now - stamp).count();
  }

  std::string transport_;
  std::string direction_topic_;
  std::string hydrophone_status_topic_;
  std::string yolo_topic_;
  std::string state_topic_;
  std::string command_override_topic_;
  std::string rc_override_topic_;
  std::string status_topic_;
  std::string mode_;
  double rate_hz_{30.0};
  double forward_fast_{0.90};
  double forward_mid_{0.58};
  double forward_slow_{0.24};
  double yaw_gain_{1.15};
  double yaw_limit_{0.72};
  double heave_gain_{0.42};
  double heave_limit_{0.38};
  double sway_gain_{0.0};
  double sway_limit_{0.0};
  double center_deadband_rad_{0.055};
  double lost_timeout_s_{0.70};
  double yolo_timeout_s_{0.50};
  double yolo_switch_range_m_{0.30};
  double stop_range_m_{0.0};
  double rc_pwm_span_{400.0};
  bool use_yolo_final_{true};
  bool auto_arm_{true};
  bool auto_mode_{true};
  bool invert_rc_heave_{true};
  bool invert_rc_yaw_{true};

  bool have_direction_{false};
  double dir_x_{1.0};
  double dir_y_{0.0};
  double dir_z_{0.0};
  double range_m_{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point last_direction_{};
  std::chrono::steady_clock::time_point last_range_{};
  std::chrono::steady_clock::time_point last_service_request_{};
  YoloGuidance yolo_{};
  mavros_msgs::msg::State last_state_{};
  bool have_state_{false};

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hydrophone_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direct_pub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingerHomingController>());
  rclcpp::shutdown();
  return 0;
}
