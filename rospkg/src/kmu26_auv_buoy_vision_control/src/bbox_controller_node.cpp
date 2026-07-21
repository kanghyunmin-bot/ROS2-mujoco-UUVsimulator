#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class BboxControllerNode : public rclcpp::Node
{
public:
  BboxControllerNode()
  : Node("bbox_controller_node")
  {
    bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/vision/buoy_bbox");
    rc_override_topic_ = declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    lost_timeout_sec_ = declare_parameter<double>("lost_timeout_sec", 0.5);
    lost_behavior_ = declare_parameter<std::string>("lost_behavior", "neutral");

    pitch_channel_ = declare_parameter<int>("pitch_channel", 1);
    roll_channel_ = declare_parameter<int>("roll_channel", 2);
    throttle_channel_ = declare_parameter<int>("throttle_channel", 3);
    yaw_channel_ = declare_parameter<int>("yaw_channel", 4);
    forward_channel_ = declare_parameter<int>("forward_channel", 5);
    lateral_channel_ = declare_parameter<int>("lateral_channel", 6);

    neutral_pwm_ = declare_parameter<int>("neutral_pwm", 1500);
    min_pwm_ = declare_parameter<int>("min_pwm", 1300);
    max_pwm_ = declare_parameter<int>("max_pwm", 1700);
    max_yaw_delta_ = declare_parameter<int>("max_yaw_delta", 250);
    max_throttle_delta_ = declare_parameter<int>("max_throttle_delta", 150);
    forward_pwm_ = declare_parameter<int>("forward_pwm", 1600);
    aligned_deadband_x_ = declare_parameter<double>("aligned_deadband_x", 0.12);
    aligned_deadband_y_ = declare_parameter<double>("aligned_deadband_y", 0.18);
    yaw_invert_ = declare_parameter<bool>("yaw_invert", false);
    vertical_positive_is_up_ = declare_parameter<bool>("vertical_positive_is_up", true);

    if (
      min_pwm_ < 1300 || max_pwm_ > 1700 || min_pwm_ >= max_pwm_ ||
      neutral_pwm_ < min_pwm_ || neutral_pwm_ > max_pwm_)
    {
      throw std::invalid_argument(
              "Thruster PWM range must stay within 1300..1700 and include neutral_pwm");
    }

    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, 10,
      std::bind(&BboxControllerNode::on_bbox, this, std::placeholders::_1));
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);

    const double period_sec = 1.0 / std::max(1.0, control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&BboxControllerNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "Subscribing: %s", bbox_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing RC override: %s", rc_override_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Channel map: pitch=%d roll=%d throttle=%d yaw=%d forward=%d lateral=%d",
      pitch_channel_, roll_channel_, throttle_channel_, yaw_channel_, forward_channel_, lateral_channel_);
  }

  void publish_lost_once()
  {
    auto channels = nochange_channels();
    apply_lost_behavior(channels);
    publish_channels(channels);
  }

private:
  void on_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 10) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring bbox message with fewer than 10 values");
      return;
    }

    std::array<float, 10> bbox{};
    std::copy_n(msg->data.begin(), bbox.size(), bbox.begin());
    last_bbox_ = bbox;
    last_bbox_time_ = now();
  }

  void on_timer()
  {
    auto channels = nochange_channels();

    if (!has_recent_detection()) {
      apply_lost_behavior(channels);
      publish_channels(channels);
      return;
    }

    const auto & bbox = *last_bbox_;
    const float center_x = bbox[4];
    const float center_y = bbox[5];
    const float image_width = bbox[8];
    const float image_height = bbox[9];

    if (image_width <= 0.0F || image_height <= 0.0F) {
      apply_lost_behavior(channels);
      publish_channels(channels);
      return;
    }

    float error_x = (center_x - image_width / 2.0F) / (image_width / 2.0F);
    float error_y = (center_y - image_height / 2.0F) / (image_height / 2.0F);
    error_x = clamp_float(error_x, -1.0F, 1.0F);
    error_y = clamp_float(error_y, -1.0F, 1.0F);

    const float yaw_sign = yaw_invert_ ? -1.0F : 1.0F;
    const int yaw_pwm = neutral_pwm_ + static_cast<int>(yaw_sign * error_x * static_cast<float>(max_yaw_delta_));

    // Image y grows downward. If positive throttle means up, a target below center needs negative throttle.
    const float vertical_sign = vertical_positive_is_up_ ? -1.0F : 1.0F;
    const int throttle_pwm = neutral_pwm_ +
      static_cast<int>(vertical_sign * error_y * static_cast<float>(max_throttle_delta_));

    const bool aligned =
      std::abs(error_x) <= aligned_deadband_x_ && std::abs(error_y) <= aligned_deadband_y_;
    const int forward_pwm = aligned ? forward_pwm_ : neutral_pwm_;

    set_channel(channels, yaw_channel_, yaw_pwm);
    set_channel(channels, throttle_channel_, throttle_pwm);
    set_channel(channels, forward_channel_, forward_pwm);

    publish_channels(channels);
  }

  bool has_recent_detection()
  {
    if (!last_bbox_) {
      return false;
    }

    const bool detected = (*last_bbox_)[1] >= 0.5F;
    const double age_sec = (now() - last_bbox_time_).seconds();
    return detected && age_sec <= lost_timeout_sec_;
  }

  std::array<uint16_t, 18> nochange_channels() const
  {
    std::array<uint16_t, 18> channels{};
    channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    return channels;
  }

  void apply_lost_behavior(std::array<uint16_t, 18> & channels)
  {
    const int lost_pwm = lost_behavior_ == "release" ?
      mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE :
      neutral_pwm_;

    set_channel(channels, throttle_channel_, lost_pwm);
    set_channel(channels, yaw_channel_, lost_pwm);
    set_channel(channels, forward_channel_, lost_pwm);
  }

  void set_channel(std::array<uint16_t, 18> & channels, int channel_number, int pwm)
  {
    if (channel_number < 1 || channel_number > static_cast<int>(channels.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid RC channel number: %d", channel_number);
      return;
    }

    if (
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE &&
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE)
    {
      pwm = clamp_int(pwm, min_pwm_, max_pwm_);
    }

    channels[static_cast<size_t>(channel_number - 1)] = static_cast<uint16_t>(pwm);
  }

  void publish_channels(const std::array<uint16_t, 18> & channels)
  {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels = channels;
    rc_pub_->publish(msg);
  }

  static int clamp_int(int value, int lower, int upper)
  {
    return std::max(lower, std::min(upper, value));
  }

  static float clamp_float(float value, float lower, float upper)
  {
    return std::max(lower, std::min(upper, value));
  }

  std::string bbox_topic_;
  std::string rc_override_topic_;
  std::string lost_behavior_;
  double control_rate_hz_{20.0};
  double lost_timeout_sec_{0.5};

  int pitch_channel_{1};
  int roll_channel_{2};
  int throttle_channel_{3};
  int yaw_channel_{4};
  int forward_channel_{5};
  int lateral_channel_{6};

  int neutral_pwm_{1500};
  int min_pwm_{1300};
  int max_pwm_{1700};
  int max_yaw_delta_{250};
  int max_throttle_delta_{150};
  int forward_pwm_{1600};
  double aligned_deadband_x_{0.12};
  double aligned_deadband_y_{0.18};
  bool yaw_invert_{false};
  bool vertical_positive_is_up_{true};

  std::optional<std::array<float, 10>> last_bbox_;
  rclcpp::Time last_bbox_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BboxControllerNode>();
  rclcpp::spin(node);
  node->publish_lost_once();
  rclcpp::shutdown();
  return 0;
}
