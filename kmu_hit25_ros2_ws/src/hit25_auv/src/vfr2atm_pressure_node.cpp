#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

class VfrToPressureNode : public rclcpp::Node
{
public:
  VfrToPressureNode()
  : Node("vfr_passthrough_converter")
  {
    pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
      "/mavros/imu/atm_pressure", 10);
    sub_ = create_subscription<mavros_msgs::msg::VfrHud>(
      "/mavros/vfr_hud", 10,
      std::bind(&VfrToPressureNode::vfrHudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "VFR_HUD altitude -> FluidPressure passthrough running");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr sub_;

  void vfrHudCallback(const mavros_msgs::msg::VfrHud::SharedPtr msg)
  {
    sensor_msgs::msg::FluidPressure pressure_msg;
    pressure_msg.header.stamp = msg->header.stamp;
    pressure_msg.header.frame_id = "base_link";
    pressure_msg.fluid_pressure = msg->altitude;
    pressure_msg.variance = 0.0;
    pressure_pub_->publish(pressure_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VfrToPressureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
