#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

class JoyToMavros : public rclcpp::Node
{
public:
  JoyToMavros()
  : Node("joy_to_mavros_node"), first_msg_received_(false), led_pwm_(1500)
  {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyToMavros::joyCallback, this, std::placeholders::_1));
    rc_override_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(
      "/mavros/rc/override", 10);

    arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    last_joy_msg_.buttons.resize(13, 0);
    last_joy_msg_.axes.resize(8, 0.0f);

    RCLCPP_INFO(get_logger(), "Joy to MAVROS node started.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  sensor_msgs::msg::Joy last_joy_msg_;
  bool first_msg_received_;
  uint16_t led_pwm_;

  static uint16_t scale_axis_to_pwm(float axis_val)
  {
    return static_cast<uint16_t>(1500 + axis_val * 300);
  }

  void send_arm_command(bool arm)
  {
    if (!arming_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Arming service not ready");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    auto future = arming_client_->async_send_request(req,
      [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        const auto resp = future.get();
        if (resp->success) {
          RCLCPP_INFO(get_logger(), arm ? "Vehicle armed" : "Vehicle disarmed");
        } else {
          RCLCPP_ERROR(get_logger(), arm ? "Failed to arm vehicle" : "Failed to disarm vehicle");
        }
      });
    (void)future;
  }

  void send_set_mode(const std::string &mode)
  {
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "SetMode service not ready");
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    auto future = set_mode_client_->async_send_request(req,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        const auto resp = future.get();
        if (resp->mode_sent) {
          RCLCPP_INFO(get_logger(), "Set mode to %s", mode.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to set mode %s", mode.c_str());
        }
      });
    (void)future;
  }

  void setMode(const sensor_msgs::msg::Joy &msg)
  {
    std::string new_mode;

    if (msg.axes.size() > 7) {
      if (msg.axes[7] == 1.0f && last_joy_msg_.axes[7] != 1.0f) new_mode = "MANUAL";
      if (msg.axes[7] == -1.0f && last_joy_msg_.axes[7] != -1.0f) new_mode = "STABILIZE";
    }

    if (msg.axes.size() > 6) {
      if (msg.axes[6] == 1.0f && last_joy_msg_.axes[6] != 1.0f) new_mode = "ALT_HOLD";
      if (msg.axes[6] == -1.0f && last_joy_msg_.axes[6] != -1.0f) {
        if (msg.buttons.size() > 10 && msg.buttons[10] == 1) {
          new_mode = "GUIDED";
        } else {
          new_mode = "POSHOLD";
        }
      }
    }

    if (!new_mode.empty()) {
      send_set_mode(new_mode);
    }
  }

  void handleLedControl(const sensor_msgs::msg::Joy &msg)
  {
    if (msg.buttons.size() > 9 && last_joy_msg_.buttons.size() > 9) {
      if (msg.buttons[9] == 1 && last_joy_msg_.buttons[9] == 0) {
        if (msg.buttons.size() > 10 && msg.buttons[10] == 1) {
          led_pwm_ = static_cast<uint16_t>(std::max(1100, static_cast<int>(led_pwm_) - 100));
          RCLCPP_INFO(get_logger(), "LED PWM Down: %d", led_pwm_);
        } else {
          led_pwm_ = static_cast<uint16_t>(std::min(1800, static_cast<int>(led_pwm_) + 100));
          RCLCPP_INFO(get_logger(), "LED PWM Up: %d", led_pwm_);
        }
      }
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!first_msg_received_) {
      last_joy_msg_ = *msg;
      first_msg_received_ = true;
      return;
    }

    if (msg->buttons.size() > 8 && last_joy_msg_.buttons.size() > 8) {
      if (msg->buttons[8] == 1 && last_joy_msg_.buttons[8] == 0) {
        if (msg->buttons.size() <= 10 || msg->buttons[10] != 1) {
          send_arm_command(true);
        }
      }
      if (msg->buttons.size() > 10 && msg->buttons[8] == 1 && msg->buttons[10] == 1 &&
          (last_joy_msg_.buttons[8] == 0 || last_joy_msg_.buttons[10] == 0)) {
        send_arm_command(false);
      }
    }

    setMode(*msg);
    handleLedControl(*msg);

    mavros_msgs::msg::OverrideRCIn rc_override_msg;
    rc_override_msg.channels.fill(1500);

    if (msg->axes.size() > 4) {
      rc_override_msg.channels[3] = scale_axis_to_pwm(-msg->axes[0]);
      rc_override_msg.channels[2] = scale_axis_to_pwm(msg->axes[1]);
      rc_override_msg.channels[5] = scale_axis_to_pwm(-msg->axes[3]);
      rc_override_msg.channels[4] = scale_axis_to_pwm(msg->axes[4]);
    }

    rc_override_msg.channels[8] = led_pwm_;
    rc_override_pub_->publish(rc_override_msg);

    last_joy_msg_ = *msg;
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
