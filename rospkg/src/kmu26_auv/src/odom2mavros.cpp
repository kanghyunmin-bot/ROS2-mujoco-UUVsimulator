#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// TF2/Geometry_msgs 변환
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomToVisionPose : public rclcpp::Node {
public:
  OdomToVisionPose()
  : Node("odom_to_vision_pose_node")
  {
    // Publisher: /mavros/vision_pose/pose
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/vision_pose/pose",
      rclcpp::QoS(10)  // ROS1의 queue_size=10과 대응되는 "depth"
    );

    // Subscriber: /rovio/odometry
    // 센서/오도메트리 계열은 QoS mismatch가 자주 나서 SensorDataQoS를 권장
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/rovio/odometry",
      rclcpp::SensorDataQoS(),
      std::bind(&OdomToVisionPose::cb, this, std::placeholders::_1)
    );

    // ROVIO (X=Left, Y=Down, Z=Front) -> ROS FLU (X=Front, Y=Left, Z=Up)
    tf2::Matrix3x3 R_rovio_to_base(
      0, 0, 1,
      1, 0, 0,
      0, -1, 0
    );

    R_rovio_to_base.getRotation(q_rovio_to_base_);
    q_rovio_to_base_.normalize();

    RCLCPP_INFO(this->get_logger(),
                "Republishing and TRANSFORMING /rovio/odometry to /mavros/vision_pose/pose.");
  }

private:
  void cb(const nav_msgs::msg::Odometry::SharedPtr msg_in)
  {
    geometry_msgs::msg::PoseStamped msg_out;

    // header 그대로 복사
    msg_out.header = msg_in->header;

    // 위치는 그대로(가정: world frame이 이미 ENU)
    msg_out.pose.position = msg_in->pose.pose.position;

    // 방향 변환
    tf2::Quaternion q_in;
    tf2::fromMsg(msg_in->pose.pose.orientation, q_in);

    tf2::Quaternion q_out = q_in * q_rovio_to_base_;
    q_out.normalize();

    msg_out.pose.orientation = tf2::toMsg(q_out);

    pub_->publish(msg_out);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  tf2::Quaternion q_rovio_to_base_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToVisionPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
