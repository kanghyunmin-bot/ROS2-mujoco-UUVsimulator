#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class LocalizationDebugNode : public rclcpp::Node
{
public:
  LocalizationDebugNode()
  : Node("localization_debug_node")
  {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/localization/path");
    path_frame_ = this->declare_parameter<std::string>("path_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    min_translation_ = this->declare_parameter<double>("min_translation", 0.0);
    max_poses_ = this->declare_parameter<int>("max_poses", 5000);

    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(100),
      std::bind(&LocalizationDebugNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing to %s and publishing Path to %s in frame %s",
      odom_topic_.c_str(),
      path_topic_.c_str(),
      path_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Received odometry without header.frame_id. Skipping path update.");
      return;
    }

    if (msg->header.frame_id != path_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Received odometry in frame %s, but path_frame is %s. Skipping message.",
        msg->header.frame_id.c_str(),
        path_frame_.c_str());
      return;
    }

    if (!msg->child_frame_id.empty() && msg->child_frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Received odometry child_frame_id %s, expected %s.",
        msg->child_frame_id.c_str(),
        base_frame_.c_str());
    }

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.header.frame_id = path_frame_;
    pose_stamped.pose = msg->pose.pose;

    if (shouldAppendPose(pose_stamped.pose)) {
      path_msg_.poses.push_back(pose_stamped);
      last_pose_ = pose_stamped.pose;
      has_last_pose_ = true;
    }

    if (max_poses_ > 0 && static_cast<int>(path_msg_.poses.size()) > max_poses_) {
      const auto erase_count = path_msg_.poses.size() - static_cast<std::size_t>(max_poses_);
      path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + erase_count);
    }

    path_msg_.header.stamp = msg->header.stamp;
    path_msg_.header.frame_id = path_frame_;
    path_pub_->publish(path_msg_);
  }

  bool shouldAppendPose(const geometry_msgs::msg::Pose & pose) const
  {
    if (!has_last_pose_) {
      return true;
    }

    if (min_translation_ <= 0.0) {
      return true;
    }

    const double dx = pose.position.x - last_pose_.position.x;
    const double dy = pose.position.y - last_pose_.position.y;
    const double dz = pose.position.z - last_pose_.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    return distance >= min_translation_;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::Path path_msg_;
  geometry_msgs::msg::Pose last_pose_;
  bool has_last_pose_ {false};

  std::string odom_topic_;
  std::string path_topic_;
  std::string path_frame_;
  std::string base_frame_;
  double min_translation_ {0.0};
  int max_poses_ {5000};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationDebugNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
