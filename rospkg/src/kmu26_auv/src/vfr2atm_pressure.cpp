#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

class VFRHudPressureNode : public rclcpp::Node {
public:
    VFRHudPressureNode()
    : Node("vfr_passthrough_converter") {
        frame_id_ = this->declare_parameter<std::string>("frame_id", "depth_link");

        pressure_pub = this->create_publisher<sensor_msgs::msg::FluidPressure>(
            "/mavros/imu/atm_pressure", 
            rclcpp::QoS(10)
        );

        sub = this->create_subscription<mavros_msgs::msg::VfrHud>(
            "/mavros/vfr_hud", 
            rclcpp::QoS(10),
            std::bind(&VFRHudPressureNode::vfrHudCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "VFR_HUD 'altitude' to Pressure 'fluid_pressure' Passthrough [RUNNING]");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /mavros/vfr_hud...");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /mavros/imu/atm_pressure...");
    }
private:


    void vfrHudCallback(const mavros_msgs::msg::VfrHud::SharedPtr msg) {
        double altitude_value = msg->altitude;

        sensor_msgs::msg::FluidPressure pressure_msg;

        pressure_msg.header.stamp = msg->header.stamp;
        pressure_msg.header.frame_id = frame_id_;

        pressure_msg.fluid_pressure = altitude_value;

        pressure_msg.variance = 0.0;

        pressure_pub->publish(pressure_msg);
    }
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub;
    rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr sub;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VFRHudPressureNode>());
    rclcpp::shutdown();
    return 0;
}
