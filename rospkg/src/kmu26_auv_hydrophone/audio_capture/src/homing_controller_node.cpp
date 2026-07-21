#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace audio_capture
{
class HomingControllerNode : public rclcpp::Node
{
public:
    explicit HomingControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("homing_controller", options)
    {
        direction_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/homing/direction",
            10,
            std::bind(&HomingControllerNode::direction_callback, this, std::placeholders::_1));
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            10,
            std::bind(&HomingControllerNode::odometry_callback, this, std::placeholders::_1));
        depth_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/depth/pose",
            10,
            std::bind(&HomingControllerNode::depth_callback, this, std::placeholders::_1));

        rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);

        homing_enabled_.store(this->declare_parameter<bool>("homing_enabled", true));
        control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 20.0);
        direction_timeout_s_ = this->declare_parameter<double>("direction_timeout_s", 1.0);
        pwm_slew_rate_per_s_ = this->declare_parameter<double>("pwm_slew_rate_per_s", 300.0);
        publish_neutral_when_inactive_ =
            this->declare_parameter<bool>("publish_neutral_when_inactive", true);

        yaw_kp_ = this->declare_parameter<double>("yaw_kp", 180.0);
        yaw_ki_ = this->declare_parameter<double>("yaw_ki", 0.0);
        yaw_kd_ = this->declare_parameter<double>("yaw_kd", 20.0);
        yaw_pwm_limit_ = this->declare_parameter<double>("yaw_pwm_limit", 180.0);

        depth_kp_ = this->declare_parameter<double>("depth_kp", 180.0);
        depth_ki_ = this->declare_parameter<double>("depth_ki", 0.0);
        depth_kd_ = this->declare_parameter<double>("depth_kd", 30.0);
        depth_pwm_limit_ = this->declare_parameter<double>("depth_pwm_limit", 180.0);
        depth_ref_rate_mps_ = this->declare_parameter<double>("depth_ref_rate_mps", 0.20);
        min_depth_ref_m_ = this->declare_parameter<double>("min_depth_ref_m", -100.0);
        max_depth_ref_m_ = this->declare_parameter<double>("max_depth_ref_m", 100.0);
        vertical_pwm_sign_ = this->declare_parameter<double>("vertical_pwm_sign", 1.0);

        forward_pwm_ = this->declare_parameter<int>("forward_pwm", 1550);
        yaw_align_threshold_rad_ = this->declare_parameter<double>("yaw_align_threshold_rad", 0.35);
        enable_keyboard_stop_ = this->declare_parameter<bool>("enable_keyboard_stop", true);
        emergency_stop_key_ = this->declare_parameter<std::string>("emergency_stop_key", "q");

        const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&HomingControllerNode::control_loop, this));

        if (enable_keyboard_stop_) {
            keyboard_thread_ = std::thread(&HomingControllerNode::keyboard_loop, this);
            RCLCPP_INFO(
                this->get_logger(),
                "Emergency stop enabled. Press '%s' to stop publishing RC override.",
                emergency_stop_key_.c_str());
        }
    }

    ~HomingControllerNode()
    {
        stop_keyboard_thread_.store(true);
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    static constexpr uint16_t NEUTRAL_PWM = 1500;
    static constexpr int PITCH_CHANNEL_INDEX = 0;
    static constexpr int ROLL_CHANNEL_INDEX = 1;
    static constexpr int VERTICAL_CHANNEL_INDEX = 2;
    static constexpr int YAW_CHANNEL_INDEX = 3;
    static constexpr int FORWARD_CHANNEL_INDEX = 4;
    static constexpr int LATERAL_CHANNEL_INDEX = 5;

    struct PidState
    {
        double integral = 0.0;
        double previous_error = 0.0;
        bool have_previous_error = false;
    };

    void direction_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        const double norm = std::sqrt(
            msg->vector.x * msg->vector.x +
            msg->vector.y * msg->vector.y +
            msg->vector.z * msg->vector.z);
        if (norm < 1.0e-6) {
            return;
        }

        direction_x_ = msg->vector.x / norm;
        direction_y_ = msg->vector.y / norm;
        direction_z_ = msg->vector.z / norm;
        last_direction_time_ = this->now();
        have_direction_ = true;
    }

    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        current_yaw_rad_ = yaw_from_quaternion(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        have_yaw_ = true;
    }

    void depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
    {
        current_depth_m_ = msg->pose.pose.position.z;
        if (!have_depth_ref_) {
            depth_ref_m_ = current_depth_m_;
            have_depth_ref_ = true;
        }
        have_depth_ = true;
    }

    void control_loop()
    {
        const rclcpp::Time now = this->now();
        const double dt = compute_dt(now);
        if (!homing_enabled_.load() || !have_direction_ || !have_yaw_ || !have_depth_ || !have_depth_ref_) {
            reset_controller_state();
            publish_neutral_failsafe();
            return;
        }
        if ((now - last_direction_time_).seconds() > direction_timeout_s_) {
            reset_controller_state();
            publish_neutral_failsafe();
            return;
        }

        const double yaw_ref_rad = std::atan2(direction_y_, direction_x_);
        const double yaw_error_rad = wrap_pi(yaw_ref_rad - current_yaw_rad_);

        depth_ref_m_ += depth_ref_rate_mps_ * direction_z_ * dt;
        depth_ref_m_ = std::clamp(depth_ref_m_, min_depth_ref_m_, max_depth_ref_m_);
        const double depth_error_m = depth_ref_m_ - current_depth_m_;

        const double yaw_pwm_delta =
            std::clamp(pid_update(yaw_pid_, yaw_error_rad, dt, yaw_kp_, yaw_ki_, yaw_kd_), -yaw_pwm_limit_, yaw_pwm_limit_);
        const double depth_pwm_delta =
            std::clamp(pid_update(depth_pid_, depth_error_m, dt, depth_kp_, depth_ki_, depth_kd_),
                -depth_pwm_limit_, depth_pwm_limit_);

        auto rc_msg = make_neutral_override_msg();

        const uint16_t desired_yaw_pwm = pwm_from_delta(yaw_pwm_delta);
        const uint16_t desired_vertical_pwm = pwm_from_delta(vertical_pwm_sign_ * depth_pwm_delta);
        const uint16_t desired_forward_pwm =
            std::abs(yaw_error_rad) < yaw_align_threshold_rad_ ? clamp_pwm(forward_pwm_) : NEUTRAL_PWM;

        rc_msg.channels[YAW_CHANNEL_INDEX] =
            slew_limit_pwm(desired_yaw_pwm, previous_yaw_pwm_, have_previous_yaw_pwm_, dt);
        rc_msg.channels[VERTICAL_CHANNEL_INDEX] =
            slew_limit_pwm(desired_vertical_pwm, previous_vertical_pwm_, have_previous_vertical_pwm_, dt);
        rc_msg.channels[FORWARD_CHANNEL_INDEX] =
            slew_limit_pwm(desired_forward_pwm, previous_forward_pwm_, have_previous_forward_pwm_, dt);
        rc_pub_->publish(rc_msg);
        neutral_failsafe_sent_ = false;
    }

    double compute_dt(const rclcpp::Time & now)
    {
        if (!have_previous_control_time_) {
            previous_control_time_ = now;
            have_previous_control_time_ = true;
            return 1.0 / std::max(control_rate_hz_, 1.0);
        }
        const double dt = (now - previous_control_time_).seconds();
        previous_control_time_ = now;
        return std::clamp(dt, 1.0e-3, 0.5);
    }

    double pid_update(
        PidState & state,
        const double error,
        const double dt,
        const double kp,
        const double ki,
        const double kd) const
    {
        state.integral += error * dt;
        state.integral = std::clamp(state.integral, -integral_limit_, integral_limit_);

        double derivative = 0.0;
        if (state.have_previous_error) {
            derivative = (error - state.previous_error) / dt;
        }
        state.previous_error = error;
        state.have_previous_error = true;
        return kp * error + ki * state.integral + kd * derivative;
    }

    void reset_controller_state()
    {
        yaw_pid_ = PidState{};
        depth_pid_ = PidState{};
        have_previous_yaw_pwm_ = false;
        have_previous_vertical_pwm_ = false;
        have_previous_forward_pwm_ = false;
    }

    mavros_msgs::msg::OverrideRCIn make_neutral_override_msg() const
    {
        mavros_msgs::msg::OverrideRCIn rc_msg;
        rc_msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        rc_msg.channels[PITCH_CHANNEL_INDEX] = NEUTRAL_PWM;
        rc_msg.channels[ROLL_CHANNEL_INDEX] = NEUTRAL_PWM;
        rc_msg.channels[VERTICAL_CHANNEL_INDEX] = NEUTRAL_PWM;
        rc_msg.channels[YAW_CHANNEL_INDEX] = NEUTRAL_PWM;
        rc_msg.channels[FORWARD_CHANNEL_INDEX] = NEUTRAL_PWM;
        rc_msg.channels[LATERAL_CHANNEL_INDEX] = NEUTRAL_PWM;
        return rc_msg;
    }

    void publish_neutral_failsafe()
    {
        if (!publish_neutral_when_inactive_) {
            return;
        }

        const auto rc_msg = make_neutral_override_msg();
        rc_pub_->publish(rc_msg);
        if (!neutral_failsafe_sent_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Publishing neutral RC override while homing is inactive or waiting for inputs.");
        }
        neutral_failsafe_sent_ = true;
    }

    uint16_t slew_limit_pwm(
        const uint16_t desired_pwm,
        uint16_t & previous_pwm,
        bool & have_previous_pwm,
        const double dt)
    {
        if (!have_previous_pwm) {
            previous_pwm = desired_pwm;
            have_previous_pwm = true;
            return previous_pwm;
        }

        const double max_step = std::max(0.0, pwm_slew_rate_per_s_) * dt;
        const double delta = std::clamp(
            static_cast<double>(desired_pwm) - static_cast<double>(previous_pwm),
            -max_step,
            max_step);
        previous_pwm = clamp_pwm(static_cast<int>(std::lround(static_cast<double>(previous_pwm) + delta)));
        return previous_pwm;
    }

    void keyboard_loop()
    {
        termios original_termios;
        bool restore_terminal = false;
        if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &original_termios) == 0) {
            termios raw_termios = original_termios;
            raw_termios.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw_termios.c_cc[VMIN] = 0;
            raw_termios.c_cc[VTIME] = 0;
            restore_terminal = tcsetattr(STDIN_FILENO, TCSANOW, &raw_termios) == 0;
        }

        while (rclcpp::ok() && !stop_keyboard_thread_.load()) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(STDIN_FILENO, &read_fds);

            timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000;

            const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
            if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
                continue;
            }

            char input = '\0';
            const ssize_t bytes_read = read(STDIN_FILENO, &input, 1);
            if (bytes_read == 0) {
                break;
            }
            if (bytes_read < 0) {
                continue;
            }
            if (!emergency_stop_key_.empty() && input == emergency_stop_key_.front()) {
                homing_enabled_.store(false);
                RCLCPP_WARN(
                    this->get_logger(),
                    "Emergency stop key '%c' received. Homing disabled; RC override publishing stopped.",
                    input);
            }
        }

        if (restore_terminal) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
        }
    }

    uint16_t pwm_from_delta(const double pwm_delta) const
    {
        return clamp_pwm(static_cast<int>(std::lround(static_cast<double>(NEUTRAL_PWM) + pwm_delta)));
    }

    uint16_t clamp_pwm(const int pwm) const
    {
        return static_cast<uint16_t>(std::clamp(pwm, 1100, 1900));
    }

    double yaw_from_quaternion(const double w, const double x, const double y, const double z) const
    {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    double wrap_pi(double angle_rad) const
    {
        while (angle_rad > M_PI) {
            angle_rad -= 2.0 * M_PI;
        }
        while (angle_rad < -M_PI) {
            angle_rad += 2.0 * M_PI;
        }
        return angle_rad;
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::atomic_bool homing_enabled_{true};
    double control_rate_hz_ = 20.0;
    double direction_timeout_s_ = 1.0;
    double pwm_slew_rate_per_s_ = 300.0;
    bool publish_neutral_when_inactive_ = true;

    double yaw_kp_ = 180.0;
    double yaw_ki_ = 0.0;
    double yaw_kd_ = 20.0;
    double yaw_pwm_limit_ = 180.0;

    double depth_kp_ = 180.0;
    double depth_ki_ = 0.0;
    double depth_kd_ = 30.0;
    double depth_pwm_limit_ = 180.0;
    double depth_ref_rate_mps_ = 0.20;
    double min_depth_ref_m_ = -100.0;
    double max_depth_ref_m_ = 100.0;
    double vertical_pwm_sign_ = 1.0;
    int forward_pwm_ = 1550;
    double yaw_align_threshold_rad_ = 0.35;
    double integral_limit_ = 2.0;
    bool enable_keyboard_stop_ = true;
    std::string emergency_stop_key_ = "q";

    double direction_x_ = 1.0;
    double direction_y_ = 0.0;
    double direction_z_ = 0.0;
    rclcpp::Time last_direction_time_;
    bool have_direction_ = false;

    double current_yaw_rad_ = 0.0;
    double current_depth_m_ = 0.0;
    double depth_ref_m_ = 0.0;
    bool have_yaw_ = false;
    bool have_depth_ = false;
    bool have_depth_ref_ = false;

    PidState yaw_pid_;
    PidState depth_pid_;
    rclcpp::Time previous_control_time_;
    bool have_previous_control_time_ = false;

    uint16_t previous_yaw_pwm_ = NEUTRAL_PWM;
    uint16_t previous_vertical_pwm_ = NEUTRAL_PWM;
    uint16_t previous_forward_pwm_ = NEUTRAL_PWM;
    bool have_previous_yaw_pwm_ = false;
    bool have_previous_vertical_pwm_ = false;
    bool have_previous_forward_pwm_ = false;
    bool neutral_failsafe_sent_ = false;

    std::atomic_bool stop_keyboard_thread_{false};
    std::thread keyboard_thread_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::HomingControllerNode)