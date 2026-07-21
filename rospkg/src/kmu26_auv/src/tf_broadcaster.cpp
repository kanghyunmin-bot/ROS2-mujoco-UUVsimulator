#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class TfBroadcasterNode : public rclcpp::Node {
public:
  TfBroadcasterNode()
  : Node("tf_broadcaster")
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "odom");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "base_link");
    translation_x_ = this->declare_parameter<double>("x", 0.0);
    translation_y_ = this->declare_parameter<double>("y", 0.0);
    translation_z_ = this->declare_parameter<double>("z", 0.0);
    roll_ = this->declare_parameter<double>("roll", 0.0);
    pitch_ = this->declare_parameter<double>("pitch", 0.0);
    yaw_ = this->declare_parameter<double>("yaw", 0.0);
    publish_rate_ = this->declare_parameter<double>("publish_rate", 30.0);

    if (publish_rate_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "publish_rate must be positive. Falling back to 30.0 Hz.");
      publish_rate_ = 30.0;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TfBroadcasterNode::publish_transform, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing TF %s -> %s at %.2f Hz",
      parent_frame_.c_str(),
      child_frame_.c_str(),
      publish_rate_);
  }

private:
  void publish_transform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;

    transform.transform.translation.x = translation_x_;
    transform.transform.translation.y = translation_y_;
    transform.transform.translation.z = translation_z_;

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll_, pitch_, yaw_);
    quaternion.normalize();

    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();

    tf_broadcaster_->sendTransform(transform);
  }

  std::string parent_frame_;
  std::string child_frame_;
  double translation_x_;
  double translation_y_;
  double translation_z_;
  double roll_;
  double pitch_;
  double yaw_;
  double publish_rate_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
