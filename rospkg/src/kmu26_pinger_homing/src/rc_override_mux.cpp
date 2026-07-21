#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

using Clock = std::chrono::steady_clock;
using Override = mavros_msgs::msg::OverrideRCIn;

struct Source {
  std::string name;
  std::string topic;
  int priority{0};
  Override message{};
  Clock::time_point received{};
  bool seen{false};
  rclcpp::Subscription<Override>::SharedPtr subscription;
};

double age_seconds(const Clock::time_point &stamp, const Clock::time_point &now) {
  return std::chrono::duration<double>(now - stamp).count();
}

bool claims_control(const Override &message) {
  return std::any_of(
      message.channels.begin(), message.channels.end(), [](uint16_t value) {
        return value != Override::CHAN_RELEASE && value != Override::CHAN_NOCHANGE;
      });
}

}  // namespace

class RcOverrideMux final : public rclcpp::Node {
 public:
  RcOverrideMux() : Node("rc_override_mux") {
    output_topic_ = declare_parameter<std::string>("output_topic", "/mavros/rc/override");
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.35);
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
    require_exclusive_output_ =
        declare_parameter<bool>("require_exclusive_output", false);
    output_discovery_grace_s_ =
        declare_parameter<double>("output_discovery_grace_s", 1.0);

    sources_.reserve(4);
    add_source("joystick", declare_parameter<std::string>(
        "joystick_topic", "/control/joystick/rc_override"), 100);
    add_source("pinger", declare_parameter<std::string>(
        "pinger_topic", "/control/pinger/rc_override"), 90);
    add_source("mission", declare_parameter<std::string>(
        "mission_topic", "/control/mission/rc_override"), 80);
    add_source("vision", declare_parameter<std::string>(
        "vision_topic", "/control/vision/rc_override"), 70);

    output_pub_ = create_publisher<Override>(output_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/control/rc_override_mux/status", rclcpp::QoS(10));
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { publish_selected(); });

    RCLCPP_INFO(
        get_logger(),
        "RC override mux ready output=%s timeout=%.3fs rate=%.1fHz exclusive=%s",
        output_topic_.c_str(), stale_timeout_s_, rate_hz_,
        require_exclusive_output_ ? "true" : "false");
  }

 private:
  void add_source(std::string name, std::string topic, int priority) {
    sources_.emplace_back();
    Source &source = sources_.back();
    source.name = std::move(name);
    source.topic = std::move(topic);
    source.priority = priority;
    source.subscription = create_subscription<Override>(
        source.topic, rclcpp::QoS(10),
        [this, index = sources_.size() - 1](const Override::SharedPtr message) {
          Source &updated = sources_[index];
          updated.message = *message;
          updated.received = Clock::now();
          updated.seen = true;
        });
  }

  void publish_selected() {
    const auto now = Clock::now();
    const auto publisher_count = get_publishers_info_by_topic(output_topic_).size();
    if (require_exclusive_output_) {
      if (age_seconds(started_, now) < output_discovery_grace_s_) return;
      if (publisher_count > 1) {
        if (!output_conflict_) {
          output_conflict_ = true;
          last_owner_ = "conflict";
          RCLCPP_ERROR(
              get_logger(),
              "RC override output conflict: %zu publishers on %s; publishing is disabled",
              publisher_count, output_topic_.c_str());
        }
        publish_status_if_due(now, "conflict", publisher_count, false, true);
        return;
      }
      if (output_conflict_) {
        output_conflict_ = false;
        RCLCPP_INFO(
            get_logger(), "RC override output conflict cleared on %s",
            output_topic_.c_str());
      }
    }

    const Source *selected = nullptr;
    for (const auto &source : sources_) {
      if (!source.seen || age_seconds(source.received, now) > stale_timeout_s_) continue;
      // A controller explicitly releases its channels when it hands off. Do
      // not let a higher-priority release frame starve a lower-priority active
      // controller (notably mission -> upstream vision control).
      if (!claims_control(source.message)) continue;
      if (selected == nullptr || source.priority > selected->priority) selected = &source;
    }

    Override output;
    std::string owner = "none";
    if (selected != nullptr) {
      output = selected->message;
      owner = selected->name;
    } else {
      output.channels.fill(Override::CHAN_RELEASE);
    }
    output_pub_->publish(output);

    const bool owner_changed = owner != last_owner_;
    if (owner_changed) {
      RCLCPP_INFO(get_logger(), "RC override owner: %s", owner.c_str());
      last_owner_ = owner;
    }
    publish_status_if_due(now, owner, publisher_count, true, false, owner_changed);
  }

  void publish_status_if_due(
      const Clock::time_point &now, const std::string &owner,
      std::size_t publisher_count, bool output_enabled, bool conflict,
      bool force = false) {
    if (!force && age_seconds(last_status_publish_, now) < 0.20) return;
    std_msgs::msg::String status;
    status.data =
        "{\"owner\":\"" + owner + "\",\"output_topic\":\"" + output_topic_ +
        "\",\"publisher_count\":" + std::to_string(publisher_count) +
        ",\"output_enabled\":" + (output_enabled ? "true" : "false") +
        ",\"conflict\":" + (conflict ? "true" : "false") + "}";
    status_pub_->publish(status);
    last_status_publish_ = now;
  }

  std::string output_topic_;
  std::string last_owner_;
  double stale_timeout_s_{0.35};
  double rate_hz_{50.0};
  bool require_exclusive_output_{false};
  bool output_conflict_{false};
  double output_discovery_grace_s_{1.0};
  Clock::time_point started_{Clock::now()};
  Clock::time_point last_status_publish_{};
  std::vector<Source> sources_;
  rclcpp::Publisher<Override>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RcOverrideMux>());
  rclcpp::shutdown();
  return 0;
}
