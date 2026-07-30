#include <cmath>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace hydrophone_ctrl
{
namespace
{
double yaw_from_quaternion(
    const geometry_msgs::msg::Quaternion & orientation)
{
    return std::atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z));
}

bool pose_is_finite(const geometry_msgs::msg::Pose & pose)
{
    const auto & p = pose.position;
    const auto & q = pose.orientation;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
        std::isfinite(q.w);
}
}  // namespace

class StartFramePublisherNode : public rclcpp::Node
{
public:
    explicit StartFramePublisherNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("start_frame_publisher", options)
    {
        const auto odometry_topic = declare_parameter<std::string>(
            "odometry_topic", "/odometry/filtered");
        start_frame_topic_ = declare_parameter<std::string>(
            "start_frame_topic", "/start_frame");
        frame_id_ = declare_parameter<std::string>("frame_id", "odom");
        const double publish_rate_hz = std::max(
            0.0, declare_parameter<double>("publish_rate_hz", 1.0));

        const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
        start_frame_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            start_frame_topic_, latched_qos);

        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 30,
            std::bind(
                &StartFramePublisherNode::odometry_callback, this,
                std::placeholders::_1));

        if (publish_rate_hz > 0.0) {
            const auto period = std::chrono::duration<double>(
                1.0 / publish_rate_hz);
            republish_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&StartFramePublisherNode::republish_timer_callback, this));
        }

        RCLCPP_INFO(
            get_logger(),
            "start_frame_publisher ready: odom=%s start_frame=%s rate=%.2f Hz "
            "(captures first valid odom; restart node to refresh)",
            odometry_topic.c_str(), start_frame_topic_.c_str(),
            publish_rate_hz);
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        if (!pose_is_finite(msg->pose.pose)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Ignoring non-finite odometry pose");
            return;
        }

        latest_odom_ = *msg;

        if (!have_start_frame_) {
            capture_from_latest();
        }
    }

    void capture_from_latest()
    {
        if (latest_odom_.header.stamp.sec > 0 ||
            latest_odom_.header.stamp.nanosec > 0)
        {
            start_frame_.header.stamp = latest_odom_.header.stamp;
        } else {
            start_frame_.header.stamp = now();
        }
        start_frame_.header.frame_id = frame_id_;
        start_frame_.pose = latest_odom_.pose.pose;
        have_start_frame_ = true;
        publish_start_frame();

        const double yaw = yaw_from_quaternion(start_frame_.pose.orientation);
        RCLCPP_INFO(
            get_logger(),
            "Captured start frame: origin_odom=(%.3f, %.3f, %.3f), "
            "yaw_odom=%.3f rad (%.1f deg) -> %s",
            start_frame_.pose.position.x,
            start_frame_.pose.position.y,
            start_frame_.pose.position.z,
            yaw, yaw * 180.0 / M_PI,
            start_frame_topic_.c_str());
    }

    void republish_timer_callback()
    {
        if (!have_start_frame_) {
            return;
        }
        publish_start_frame();
    }

    void publish_start_frame()
    {
        auto msg = start_frame_;
        msg.header.stamp = now();
        start_frame_pub_->publish(msg);
    }

    std::string start_frame_topic_;
    std::string frame_id_ = "odom";

    bool have_start_frame_ = false;
    nav_msgs::msg::Odometry latest_odom_;
    geometry_msgs::msg::PoseStamped start_frame_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        start_frame_pub_;
    rclcpp::TimerBase::SharedPtr republish_timer_;
};
}  // namespace hydrophone_ctrl

RCLCPP_COMPONENTS_REGISTER_NODE(hydrophone_ctrl::StartFramePublisherNode)
