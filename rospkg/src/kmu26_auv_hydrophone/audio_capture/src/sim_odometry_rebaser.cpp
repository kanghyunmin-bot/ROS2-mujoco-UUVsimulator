#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace audio_capture
{
// MuJoCo 절대 odometry를 첫 수신 위치 기준으로 리베이스한다.
// XY는 항상 0으로 맞추고, z는 zero_z=true일 때만 시작 수심을 0으로 맞춘다.
// orientation은 raw 값을 유지한다. start_frame은 다루지 않는다.
class SimOdometryRebaserNode : public rclcpp::Node
{
public:
    explicit SimOdometryRebaserNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("sim_odometry_rebaser", options)
    {
        const auto input_topic = declare_parameter<std::string>(
            "input_topic", "/odometry/mujoco_raw");
        const auto output_topic = declare_parameter<std::string>(
            "output_topic", "/odometry/filtered");
        output_frame_ = declare_parameter<std::string>("output_frame", "odom");
        zero_z_ = declare_parameter<bool>("zero_z", false);

        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            input_topic, 30,
            std::bind(
                &SimOdometryRebaserNode::odometry_callback, this,
                std::placeholders::_1));
        odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 30);

        RCLCPP_INFO(
            get_logger(),
            "sim_odometry_rebaser: %s -> %s "
            "(XY zeroed, z=%s, orientation kept)",
            input_topic.c_str(), output_topic.c_str(),
            zero_z_ ? "zeroed at first sample" : "raw");
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        const Eigen::Vector2d position(
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        const double z = msg->pose.pose.position.z;
        if (!position.allFinite() || !std::isfinite(z)) {
            return;
        }

        if (!have_origin_) {
            origin_ = position;
            origin_z_ = z;
            have_origin_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Captured sim odom origin=(%.3f, %.3f, %.3f)",
                origin_.x(), origin_.y(), origin_z_);
        }

        const Eigen::Vector2d relative = position - origin_;
        nav_msgs::msg::Odometry output = *msg;
        output.header.frame_id = output_frame_;
        output.pose.pose.position.x = relative.x();
        output.pose.pose.position.y = relative.y();
        output.pose.pose.position.z = zero_z_ ? z - origin_z_ : z;
        odometry_pub_->publish(output);
    }

    bool have_origin_ = false;
    bool zero_z_ = false;
    std::string output_frame_ = "odom";
    Eigen::Vector2d origin_{0.0, 0.0};
    double origin_z_ = 0.0;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::SimOdometryRebaserNode)
