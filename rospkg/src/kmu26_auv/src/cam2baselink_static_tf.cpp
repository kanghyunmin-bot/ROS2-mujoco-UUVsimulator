#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cmath>

class Cam2BaseLinkStaticTF : public rclcpp::Node {
public:
    Cam2BaseLinkStaticTF()
    : Node("cam2baselink_static_tf") {
        std::string parent_frame = "camera1";
        std::string child_frame = "base_link";

        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->now();
        t.header.frame_id = parent_frame;
        t.child_frame_id = child_frame;

        t.transform.translation.x = -0.0375;
        t.transform.translation.y = 0.0225;
        t.transform.translation.z = -0.07;

        tf2::Quaternion q;
        q.setRPY(0.0, M_PI_2, 0.0);
        t.transform.rotation.x = 0.5;
        t.transform.rotation.y = -0.5;
        t.transform.rotation.z = 0.5;
        t.transform.rotation.w = 0.5;
        static_broadcaster_->sendTransform(t);
    }
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Cam2BaseLinkStaticTF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}