#include <array>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

class PressureToDepthPose : public rclcpp::Node
{
public:
  PressureToDepthPose()
  : Node("pressure_to_depth_pose")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/mavros/imu/static_pressure");
    input_mode_ = declare_parameter<std::string>("input_mode", "pressure_pa");
    output_topic_ = declare_parameter<std::string>("output_topic", "/depth/pose");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    fluid_density_ = declare_parameter<double>("fluid_density", 1000.0);
    gravity_ = declare_parameter<double>("gravity", 9.80665);
    surface_pressure_pa_ = declare_parameter<double>("surface_pressure_pa", 101325.0);
    zero_at_start_ = declare_parameter<bool>("zero_at_start", true);
    depth_offset_m_ = declare_parameter<double>("depth_offset_m", 0.0);
    z_variance_ = declare_parameter<double>("z_variance", 0.05);

    const auto sensor_qos = rclcpp::SensorDataQoS();

    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(output_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      input_topic_, sensor_qos,
      std::bind(&PressureToDepthPose::handle_msg, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Bridging %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void handle_msg(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    if (!std::isfinite(msg->fluid_pressure)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping non-finite pressure sample.");
      return;
    }

    double depth_m = 0.0;
    if (input_mode_ == "pressure_pa") {
      depth_m = (msg->fluid_pressure - surface_pressure_pa_) / (fluid_density_ * gravity_);
    } else {
      if (input_mode_ != "depth_m") {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Unknown input_mode '%s'. Falling back to depth_m.",
          input_mode_.c_str());
      }
      depth_m = msg->fluid_pressure;
    }
    if (!have_reference_) {
      reference_depth_m_ = zero_at_start_ ? depth_m : 0.0;
      have_reference_ = true;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = msg->header;
    out.header.frame_id = world_frame_;
    out.pose.pose.position.z = -(depth_m - reference_depth_m_) + depth_offset_m_;
    out.pose.covariance.fill(0.0);
    out.pose.covariance[14] = z_variance_;
    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string input_mode_;
  std::string output_topic_;
  std::string world_frame_;
  double fluid_density_;
  double gravity_;
  double surface_pressure_pa_;
  bool zero_at_start_;
  double depth_offset_m_;
  double z_variance_;
  bool have_reference_{false};
  double reference_depth_m_{0.0};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PressureToDepthPose>());
  rclcpp::shutdown();
  return 0;
}
