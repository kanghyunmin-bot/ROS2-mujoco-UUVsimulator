#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <cmath>

namespace
{
constexpr double kSeaLevelPressurePa = 101325.0;
constexpr double kLapseRate = 0.0065;     // K/m
constexpr double kSeaLevelTempK = 288.15;  // K
constexpr double kLapseExponent = 5.25588; // g*M/(R*L) for standard atmosphere
} // namespace

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
    const double altitude_m = static_cast<double>(msg->altitude);
    // Convert altitude to a pressure estimate.
    // Approximation:
    // P = P0 * (1 - L*h/T0)^(g*M/(R*L))
    // where h is altitude (m), L is lapse rate (K/m).
    double pressure_ratio = 1.0 - (kLapseRate * altitude_m) / kSeaLevelTempK;
    if (pressure_ratio <= 0.0)
    {
      pressure_ratio = 0.0001;
    }
    const double atm_pressure_pa = kSeaLevelPressurePa * std::pow(pressure_ratio, kLapseExponent);

    sensor_msgs::msg::FluidPressure pressure_msg;
    pressure_msg.header.stamp = msg->header.stamp;
    pressure_msg.header.frame_id = "base_link";
    pressure_msg.fluid_pressure = static_cast<float>(atm_pressure_pa);
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
