#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

class CamToBaselinkStaticTf : public rclcpp::Node
{
public:
  CamToBaselinkStaticTf()
  : Node("cam2baselink_static_tf")
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "camera1");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "base_link");

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = parent_frame_;
    t.child_frame_id = child_frame_;

    t.transform.translation.x = -0.0375;
    t.transform.translation.y = 0.0225;
    t.transform.translation.z = -0.07;

    tf2::Quaternion q;
    q.setRPY(0.0, M_PI_2, 0.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    broadcaster_->sendTransform(t);
  }

private:
  std::string parent_frame_;
  std::string child_frame_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CamToBaselinkStaticTf>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
