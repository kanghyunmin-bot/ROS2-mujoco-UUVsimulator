#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomToVisionPose : public rclcpp::Node
{
public:
  OdomToVisionPose()
  : Node("odom_to_vision_pose")
  {
    // Legacy bridge: convert any odometry source (typically /dvl/odometry) into
    // MAVROS vision_pose for position controller integrations.
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/dvl/odometry");
    apply_rotation_ = declare_parameter<bool>("apply_rotation", false);

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/vision_pose/pose", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&OdomToVisionPose::cb, this, std::placeholders::_1));

    if (apply_rotation_) {
      tf2::Matrix3x3 R_source_to_base(
        0, 0, 1,
        1, 0, 0,
        0, -1, 0);
      R_source_to_base.getRotation(q_odom_to_base_);
      q_odom_to_base_.normalize();
    } else {
      q_odom_to_base_.setValue(0.0, 0.0, 0.0, 1.0);
    }

    RCLCPP_INFO(
      get_logger(),
      "Republishing %s to /mavros/vision_pose/pose (apply_rotation=%s)",
      odom_topic_.c_str(),
      apply_rotation_ ? "true" : "false");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  tf2::Quaternion q_odom_to_base_;
  std::string odom_topic_;
  bool apply_rotation_{false};

  void cb(const nav_msgs::msg::Odometry::SharedPtr msg_in)
  {
    geometry_msgs::msg::PoseStamped msg_out;
    msg_out.header = msg_in->header;
    msg_out.pose.position = msg_in->pose.pose.position;

    tf2::Quaternion q_in;
    tf2::fromMsg(msg_in->pose.pose.orientation, q_in);
    tf2::Quaternion q_out = q_in * q_odom_to_base_;
    q_out.normalize();
    msg_out.pose.orientation = tf2::toMsg(q_out);

    pub_->publish(msg_out);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToVisionPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
