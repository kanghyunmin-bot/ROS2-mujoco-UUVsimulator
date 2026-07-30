#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>

class JoyToMavros : public rclcpp::Node {
public:
    JoyToMavros()
    : Node("joy_to_mavros_node"), led_pwm_(1500)
    {
        // Publisher for RC override
        rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
            "/mavros/rc/override", rclcpp::QoS(10)
        );

        // Subscriber for joystick inputs
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            std::bind(&JoyToMavros::joyCallback, this, std::placeholders::_1)
        );
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", rclcpp::QoS(10),
            std::bind(&JoyToMavros::stateCallback, this, std::placeholders::_1)
        );
        guided_waypoint_enable_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
            "/guided/waypoint_enable", rclcpp::QoS(10));
        guided_cancel_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/guided/cancel", rclcpp::QoS(10));

        // Client for arming service
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        // Client for set mode service
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        axis_deadzone_ = this->declare_parameter<double>("axis_deadzone", 0.08);
        vertical_axis_deadzone_ = this->declare_parameter<double>("vertical_axis_deadzone", 0.10);
        pwm_range_ = this->declare_parameter<double>("pwm_range", 300.0);
        alt_hold_entry_neutral_sec_ =
            this->declare_parameter<double>("alt_hold_entry_neutral_sec", 1.0);
        alt_hold_post_entry_neutral_sec_ =
            this->declare_parameter<double>("alt_hold_post_entry_neutral_sec", 0.3);

        alt_hold_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JoyToMavros::handle_pending_alt_hold_request, this));

        RCLCPP_INFO(this->get_logger(), "Joy to Mavros node initialized.");
    }
private:
    static constexpr uint16_t NEUTRAL_PWM = 1500;
    static constexpr int VERTICAL_CHANNEL_INDEX = 2;

    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr guided_waypoint_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr guided_cancel_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr alt_hold_timer_;
    sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;

    bool first_msg_received_ = false;
    uint16_t led_pwm_;
    double axis_deadzone_ = 0.08;
    double vertical_axis_deadzone_ = 0.10;
    double pwm_range_ = 300.0;
    double alt_hold_entry_neutral_sec_ = 1.0;
    double alt_hold_post_entry_neutral_sec_ = 0.3;
    bool alt_hold_request_pending_ = false;
    std::string current_mode_;
    std::chrono::steady_clock::time_point alt_hold_mode_request_at_;
    std::chrono::steady_clock::time_point alt_hold_neutral_until_;

    float apply_deadzone(float axis_val, double deadzone) const {
        if (std::abs(axis_val) < deadzone) {
            return 0.0f;
        }
        return axis_val;
    }

    uint16_t scale_axis_to_pwm(float axis_val, double deadzone) const {
        const float filtered_axis = apply_deadzone(axis_val, deadzone);
        const double pwm = static_cast<double>(NEUTRAL_PWM) + filtered_axis * pwm_range_;
        return static_cast<uint16_t>(std::clamp(pwm, 1100.0, 1900.0));
    }

    bool hasButton(const sensor_msgs::msg::Joy::SharedPtr msg, std::size_t index) const {
        return index < msg->buttons.size();
    }

    bool hasAxis(const sensor_msgs::msg::Joy::SharedPtr msg, std::size_t index) const {
        return index < msg->axes.size();
    }

    bool buttonPressed(const sensor_msgs::msg::Joy::SharedPtr msg, std::size_t index) const {
        return hasButton(msg, index) && msg->buttons[index] == 1;
    }

    bool buttonRisingEdge(const sensor_msgs::msg::Joy::SharedPtr msg, std::size_t index) const {
        return hasButton(msg, index) && hasButton(last_joy_msg_, index) &&
               msg->buttons[index] == 1 && last_joy_msg_->buttons[index] == 0;
    }

    float axisValue(const sensor_msgs::msg::Joy::SharedPtr msg, std::size_t index) const {
        return hasAxis(msg, index) ? msg->axes[index] : 0.0f;
    }

    void schedule_alt_hold_request() {
        const auto now = std::chrono::steady_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(alt_hold_entry_neutral_sec_));
        alt_hold_request_pending_ = true;
        alt_hold_mode_request_at_ = now + duration;
        alt_hold_neutral_until_ = alt_hold_mode_request_at_;
        publish_neutral_vertical_override();
        RCLCPP_INFO(
            this->get_logger(),
            "ALT_HOLD requested. Holding vertical RC neutral for %.2f s before mode change.",
            alt_hold_entry_neutral_sec_);
    }

    void cancel_pending_alt_hold_request() {
        alt_hold_request_pending_ = false;
    }

    void handle_pending_alt_hold_request() {
        const auto now = std::chrono::steady_clock::now();
        if (alt_hold_request_pending_ || now < alt_hold_neutral_until_) {
            publish_neutral_vertical_override();
        }
        if (!alt_hold_request_pending_ || now < alt_hold_mode_request_at_) {
            return;
        }

        alt_hold_request_pending_ = false;
        const auto post_duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(alt_hold_post_entry_neutral_sec_));
        alt_hold_neutral_until_ = now + post_duration;
        send_set_mode_request("ALT_HOLD");
    }

    bool is_alt_hold_entry_neutral_active() const {
        return std::chrono::steady_clock::now() < alt_hold_neutral_until_;
    }

    void publish_neutral_vertical_override() {
        mavros_msgs::msg::OverrideRCIn rc_override_msg;
        for (auto & channel : rc_override_msg.channels) {
            channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
        }
        rc_override_msg.channels[VERTICAL_CHANNEL_INDEX] = NEUTRAL_PWM;
        rc_pub_->publish(rc_override_msg);
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
        current_mode_ = msg->mode;
    }

    void publish_guided_rc_release() {
        mavros_msgs::msg::OverrideRCIn rc_override_msg;
        for (auto & channel : rc_override_msg.channels) {
            channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
        }
        rc_override_msg.channels[2] = mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;
        rc_override_msg.channels[3] = mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;
        rc_override_msg.channels[4] = mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;
        rc_override_msg.channels[5] = mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;
        rc_override_msg.channels[8] = led_pwm_;
        rc_pub_->publish(rc_override_msg);
    }

    void set_guided_waypoint_enabled(bool enabled) {
        std_msgs::msg::Bool enable_msg;
        enable_msg.data = enabled;
        guided_waypoint_enable_pub_->publish(enable_msg);
        if (!enabled) {
            guided_cancel_pub_->publish(std_msgs::msg::Empty());
        }
    }
    
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!first_msg_received_) {
            last_joy_msg_ = msg;
            first_msg_received_ = true;
            return;
        }


        if (buttonRisingEdge(msg, 4)) {
            if (!buttonPressed(msg, 5)) {
                send_command_bool_request(false);
            }
        }
        if (buttonPressed(msg, 4) && buttonPressed(msg, 5) &&
            ((hasButton(last_joy_msg_, 4) && last_joy_msg_->buttons[4] == 0) ||
             (hasButton(last_joy_msg_, 5) && last_joy_msg_->buttons[5] == 0))) {
            send_command_bool_request(true);
        }

        setMode(msg);



        handleLedControl(msg);

        if (current_mode_ == "GUIDED") {
            publish_guided_rc_release();
            last_joy_msg_ = msg;
            return;
        }

        mavros_msgs::msg::OverrideRCIn rc_override_msg;


        for (int i = 0; i < 18; i++) {
            rc_override_msg.channels[i] = NEUTRAL_PWM; // 기본값 1500 (중립)
        }


        rc_override_msg.channels[3] = scale_axis_to_pwm(-axisValue(msg, 2), axis_deadzone_); //yaw

        const float vertical_axis = is_alt_hold_entry_neutral_active() ? 0.0f : axisValue(msg, 3);
        rc_override_msg.channels[VERTICAL_CHANNEL_INDEX] =
            scale_axis_to_pwm(vertical_axis, vertical_axis_deadzone_); //상승 하강

        rc_override_msg.channels[5] = scale_axis_to_pwm(-axisValue(msg, 0), axis_deadzone_); //lateral

        rc_override_msg.channels[4] = scale_axis_to_pwm(axisValue(msg, 1), axis_deadzone_); //전진 후진

        rc_override_msg.channels[8]= led_pwm_;

        rc_pub_->publish(rc_override_msg);

        last_joy_msg_ = msg;

    }

    void setMode(const sensor_msgs::msg::Joy::SharedPtr msg) {
        std::string new_mode = "";

        if (axisValue(msg, 7) == 1.0 && axisValue(last_joy_msg_, 7) != 1.0) new_mode = "MANUAL";

        if (axisValue(msg, 7) == -1.0 && axisValue(last_joy_msg_, 7) != -1.0) new_mode = "STABILIZE";

        if (axisValue(msg, 6) == 1.0 && axisValue(last_joy_msg_, 6) != 1.0) new_mode = "ALT_HOLD";

        if (axisValue(msg, 6) == -1.0 && axisValue(last_joy_msg_, 6) != -1.0) {
            if (buttonPressed(msg, 10)) {
                new_mode = "GUIDED";
            }
            else {
                new_mode = "POSHOLD";
            }
        }

        if (!new_mode.empty()) {
            set_guided_waypoint_enabled(new_mode == "GUIDED");
            if (new_mode == "ALT_HOLD") {
                schedule_alt_hold_request();
                return;
            }
            cancel_pending_alt_hold_request();
            send_set_mode_request(new_mode);
        }
    }

    void send_command_bool_request(bool value) {
        using namespace std::chrono_literals;
        if (!arm_client_->wait_for_service(0s)) {
            RCLCPP_WARN(this->get_logger(), "Arming service not available.");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = value;
        arm_client_->async_send_request(
            request,
            [this, value](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                try {
                    const auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(
                            this->get_logger(), "Vehicle %s.",
                            value ? "armed" : "disarmed"
                        );
                    } else {
                        RCLCPP_ERROR(
                            this->get_logger(), "Failed to %s vehicle.",
                            value ? "arm" : "disarm"
                        );
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(
                        this->get_logger(), "Arming service call failed: %s", e.what()
                    );
                }
            }
        );
    }

    void send_set_mode_request(const std::string& mode) {
        using namespace std::chrono_literals;
        if (!set_mode_client_->wait_for_service(0s)) {
            RCLCPP_WARN(this->get_logger(), "Set mode service not available.");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        set_mode_client_->async_send_request(
            request,
            [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                try {
                    const auto response = future.get();
                    if (response->mode_sent) {
                        RCLCPP_INFO(this->get_logger(), "Mode changed to %s", mode.c_str());
                    } else {
                        RCLCPP_ERROR(
                            this->get_logger(), "Failed to change mode to %s", mode.c_str()
                        );
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(
                        this->get_logger(), "Set mode service call failed: %s", e.what()
                    );
                }
            }
        );
    }

    void handleLedControl(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (buttonRisingEdge(msg, 6)) {
            if (buttonPressed(msg, 5)) {
                led_pwm_ -= 100;
                if (led_pwm_ < 1100) led_pwm_ = 1100;
                RCLCPP_INFO(this->get_logger(), "LED PWM Down: %d", led_pwm_);
            }

            else {
                led_pwm_ += 100;
                if (led_pwm_ > 1800) led_pwm_ = 1800;
                RCLCPP_INFO(this->get_logger(), "LED PWM Up: %d", led_pwm_);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyToMavros>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
