#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace audio_capture
{
class SimpleHomingControllerNode : public rclcpp::Node
{
public:
    explicit SimpleHomingControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("simple_homing_controller", options)
    {
        direction_topic_ = this->declare_parameter<std::string>("direction_topic", "/homing/direction");
        rc_override_topic_ = this->declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");

        rate_hz_ = clamp(this->declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
        direction_timeout_s_ =
            clamp(this->declare_parameter<double>("direction_timeout_s", 0.70), 0.05, 5.0);
        publish_neutral_when_lost_ =
            this->declare_parameter<bool>("publish_neutral_when_lost", true);

        forward_fast_ = clamp(this->declare_parameter<double>("forward_fast", 0.90), 0.0, 1.0);
        forward_mid_ = clamp(this->declare_parameter<double>("forward_mid", 0.58), 0.0, 1.0);
        forward_slow_ = clamp(this->declare_parameter<double>("forward_slow", 0.24), 0.0, 1.0);
        yaw_gain_ = clamp(this->declare_parameter<double>("yaw_gain", 1.15), 0.0, 4.0);
        yaw_limit_ = clamp(this->declare_parameter<double>("yaw_limit", 0.72), 0.0, 1.0);
        heave_gain_ = clamp(this->declare_parameter<double>("heave_gain", 0.42), 0.0, 2.0);
        heave_limit_ = clamp(this->declare_parameter<double>("heave_limit", 0.38), 0.0, 1.0);
        sway_gain_ = clamp(this->declare_parameter<double>("sway_gain", 0.0), 0.0, 2.0);
        sway_limit_ = clamp(this->declare_parameter<double>("sway_limit", 0.0), 0.0, 1.0);
        center_deadband_rad_ =
            clamp(this->declare_parameter<double>("center_deadband_rad", 0.055), 0.0, 0.50);
        rc_pwm_span_ = clamp(this->declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);
        invert_rc_heave_ = this->declare_parameter<bool>("invert_rc_heave", true);
        invert_rc_yaw_ = this->declare_parameter<bool>("invert_rc_yaw", true);

        direction_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            direction_topic_,
            10,
            std::bind(&SimpleHomingControllerNode::direction_callback, this, std::placeholders::_1));
        rc_pub_ =
            this->create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SimpleHomingControllerNode::control_loop, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Simple homing controller ready. direction=%s rc_override=%s rate=%.1fHz",
            direction_topic_.c_str(),
            rc_override_topic_.c_str(),
            rate_hz_);
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr uint16_t RC_NEUTRAL = 1500;
    static constexpr std::size_t PITCH_CHANNEL_INDEX = 0;
    static constexpr std::size_t ROLL_CHANNEL_INDEX = 1;
    static constexpr std::size_t VERTICAL_CHANNEL_INDEX = 2;
    static constexpr std::size_t YAW_CHANNEL_INDEX = 3;
    static constexpr std::size_t FORWARD_CHANNEL_INDEX = 4;
    static constexpr std::size_t LATERAL_CHANNEL_INDEX = 5;
    static constexpr std::size_t PRIMARY_CHANNEL_COUNT = 8;

    struct Command
    {
        double forward = 0.0;
        double sway = 0.0;
        double heave = 0.0;
        double yaw = 0.0;
    };

    void direction_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        const double x = msg->vector.x;
        const double y = msg->vector.y;
        const double z = msg->vector.z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            return;
        }

        const double norm = std::sqrt(x * x + y * y + z * z);
        if (norm < 1.0e-6) {
            return;
        }

        direction_x_ = x / norm;
        direction_y_ = y / norm;
        direction_z_ = z / norm;
        last_direction_time_ = this->now();
        have_direction_ = true;
        neutral_failsafe_sent_ = false;
    }

    void control_loop()
    {
        if (!have_fresh_direction()) {
            publish_neutral_failsafe();
            return;
        }

        rc_pub_->publish(make_rc_override(hydrophone_command()));
    }

    bool have_fresh_direction() const
    {
        return have_direction_ &&
               (this->now() - last_direction_time_).seconds() <= direction_timeout_s_;
    }

    Command hydrophone_command() const
    {
        Command cmd;

        const double bearing = wrap_pi(std::atan2(direction_y_, direction_x_));
        const double yaw_abs = std::abs(bearing);
        cmd.yaw = yaw_abs <= center_deadband_rad_ ? 0.0 : clamp(yaw_gain_ * bearing, -yaw_limit_, yaw_limit_);
        cmd.heave = clamp(-heave_gain_ * direction_z_, -heave_limit_, heave_limit_);
        cmd.sway = clamp(sway_gain_ * direction_y_, -sway_limit_, sway_limit_);

        double forward = forward_fast_;
        if (yaw_abs > 1.10) {
            forward = std::min(forward, forward_slow_);
        } else if (yaw_abs > 0.72) {
            forward = std::min(forward, forward_mid_);
        } else if (yaw_abs > 0.42) {
            forward = std::min(forward, std::max(forward_mid_, 0.62));
        }
        if (direction_x_ < 0.05) {
            forward = std::min(forward, 0.18);
        }
        cmd.forward = clamp(forward, 0.0, 1.0);
        return cmd;
    }

    mavros_msgs::msg::OverrideRCIn make_rc_override(const Command & cmd) const
    {
        mavros_msgs::msg::OverrideRCIn msg;
        msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        for (std::size_t i = 0; i < PRIMARY_CHANNEL_COUNT && i < msg.channels.size(); ++i) {
            msg.channels[i] = RC_NEUTRAL;
        }

        msg.channels[PITCH_CHANNEL_INDEX] = RC_NEUTRAL;
        msg.channels[ROLL_CHANNEL_INDEX] = RC_NEUTRAL;
        msg.channels[VERTICAL_CHANNEL_INDEX] = axis_pwm(cmd.heave, invert_rc_heave_);
        msg.channels[YAW_CHANNEL_INDEX] = axis_pwm(cmd.yaw, invert_rc_yaw_);
        msg.channels[FORWARD_CHANNEL_INDEX] = axis_pwm(cmd.forward, false);
        msg.channels[LATERAL_CHANNEL_INDEX] = axis_pwm(cmd.sway, false);
        return msg;
    }

    void publish_neutral_failsafe()
    {
        if (!publish_neutral_when_lost_) {
            return;
        }

        rc_pub_->publish(make_rc_override(Command{}));
        if (!neutral_failsafe_sent_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Direction input is missing or stale. Publishing neutral RC override.");
            neutral_failsafe_sent_ = true;
        }
    }

    uint16_t axis_pwm(const double value, const bool invert) const
    {
        const double axis = invert ? -value : value;
        const int pwm =
            static_cast<int>(std::llround(static_cast<double>(RC_NEUTRAL) + clamp(axis, -1.0, 1.0) * rc_pwm_span_));
        return static_cast<uint16_t>(std::clamp(pwm, 1100, 1900));
    }

    static double clamp(const double value, const double low, const double high)
    {
        return std::max(low, std::min(high, value));
    }

    static double wrap_pi(double value)
    {
        while (value > PI) {
            value -= 2.0 * PI;
        }
        while (value < -PI) {
            value += 2.0 * PI;
        }
        return value;
    }

    std::string direction_topic_;
    std::string rc_override_topic_;
    double rate_hz_ = 30.0;
    double direction_timeout_s_ = 0.70;
    bool publish_neutral_when_lost_ = true;

    double forward_fast_ = 0.90;
    double forward_mid_ = 0.58;
    double forward_slow_ = 0.24;
    double yaw_gain_ = 1.15;
    double yaw_limit_ = 0.72;
    double heave_gain_ = 0.42;
    double heave_limit_ = 0.38;
    double sway_gain_ = 0.0;
    double sway_limit_ = 0.0;
    double center_deadband_rad_ = 0.055;
    double rc_pwm_span_ = 400.0;
    bool invert_rc_heave_ = true;
    bool invert_rc_yaw_ = true;

    double direction_x_ = 1.0;
    double direction_y_ = 0.0;
    double direction_z_ = 0.0;
    rclcpp::Time last_direction_time_;
    bool have_direction_ = false;
    bool neutral_failsafe_sent_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::SimpleHomingControllerNode)
