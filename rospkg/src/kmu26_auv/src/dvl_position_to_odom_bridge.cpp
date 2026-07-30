#include <cmath>
#include <string>

#include <auv_dvl_a50_msg/msg/dvldr.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DvlPositionToOdomBridge : public rclcpp::Node
{
public:
  DvlPositionToOdomBridge()
  : Node("dvl_position_to_odom_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/dvl/position");
    output_topic_ = declare_parameter<std::string>("output_topic", "/dvl/odometry");
    frame_id_ = declare_parameter<std::string>("frame_id", "dvl_odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "dvl_link");
    expected_type_ = declare_parameter<std::string>("expected_type", "position_local");

    zero_position_on_start_ = declare_parameter<bool>("zero_position_on_start", false);
    zero_orientation_on_start_ = declare_parameter<bool>("zero_orientation_on_start", false);
    orientation_in_degrees_ = declare_parameter<bool>("orientation_in_degrees", true);
    require_status_zero_ = declare_parameter<bool>("require_status_zero", false);
    estimate_twist_ = declare_parameter<bool>("estimate_twist", false);
    reset_origin_on_jump_ = declare_parameter<bool>("reset_origin_on_jump", false);
    use_dvl_position_stddev_ = declare_parameter<bool>("use_dvl_position_stddev", true);

    position_scale_x_ = declare_parameter<double>("position_scale_x", 1.0);
    position_scale_y_ = declare_parameter<double>("position_scale_y", 1.0);
    position_scale_z_ = declare_parameter<double>("position_scale_z", 1.0);
    max_twist_dt_ = declare_parameter<double>("max_twist_dt", 1.0);
    max_position_norm_ = declare_parameter<double>("max_position_norm", 0.0);
    max_position_speed_ = declare_parameter<double>("max_position_speed", 0.0);

    position_variance_xy_ = declare_parameter<double>("position_variance_xy", 0.25);
    position_variance_z_ = declare_parameter<double>("position_variance_z", 100.0);
    orientation_variance_ = declare_parameter<double>("orientation_variance", 1.0);
    twist_variance_linear_ = declare_parameter<double>("twist_variance_linear", 0.25);
    twist_variance_angular_ = declare_parameter<double>("twist_variance_angular", 1.0);

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    subscription_ = create_subscription<auv_dvl_a50_msg::msg::DVLDR>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DvlPositionToOdomBridge::handle_msg, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Bridging %s -> %s as raw DVL DR in frame %s, child %s",
      input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(), child_frame_id_.c_str());
  }

private:
  void handle_msg(const auv_dvl_a50_msg::msg::DVLDR::SharedPtr msg)
  {
    if (!expected_type_.empty() && msg->type != expected_type_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVLDR sample with type '%s'.", msg->type.c_str());
      return;
    }
    if (require_status_zero_ && msg->status != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVLDR sample with status %ld.", msg->status);
      return;
    }

    const auto raw_x = static_cast<double>(msg->position.x);
    const auto raw_y = static_cast<double>(msg->position.y);
    const auto raw_z = static_cast<double>(msg->position.z);
    if (!std::isfinite(raw_x) || !std::isfinite(raw_y) || !std::isfinite(raw_z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVLDR sample with non-finite position.");
      return;
    }

    const bool origin_needed = zero_position_on_start_ || zero_orientation_on_start_;
    if (origin_needed && !have_origin_) {
      set_origin(*msg, true);
    }

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = frame_id_;
    out.child_frame_id = child_frame_id_;

    const auto x = zero_position_on_start_ ? raw_x - origin_x_ : raw_x;
    const auto y = zero_position_on_start_ ? raw_y - origin_y_ : raw_y;
    const auto z = zero_position_on_start_ ? raw_z - origin_z_ : raw_z;
    out.pose.pose.position.x = position_scale_x_ * x;
    out.pose.pose.position.y = position_scale_y_ * y;
    out.pose.pose.position.z = position_scale_z_ * z;

    auto roll = static_cast<double>(msg->roll);
    auto pitch = static_cast<double>(msg->pitch);
    auto yaw = static_cast<double>(msg->yaw);
    if (zero_orientation_on_start_) {
      roll -= origin_roll_;
      pitch -= origin_pitch_;
      yaw -= origin_yaw_;
    }

    if (!is_reasonable_position(out)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "DVLDR odometry jump rejected: position=(%.3f, %.3f, %.3f).",
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z);
      if (!reset_origin_on_jump_) {
        return;
      }
      set_origin(*msg, false);
      out.pose.pose.position.x = 0.0;
      out.pose.pose.position.y = 0.0;
      out.pose.pose.position.z = 0.0;
      have_last_pose_ = false;

      if (zero_orientation_on_start_) {
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
      }
    }

    if (orientation_in_degrees_) {
      roll = to_radians(roll);
      pitch = to_radians(pitch);
      yaw = to_radians(yaw);
    }

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    out.pose.pose.orientation = tf2::toMsg(q);

    out.pose.covariance.fill(0.0);
    const auto variance_xy = resolve_position_variance_xy(*msg);
    out.pose.covariance[0] = variance_xy;
    out.pose.covariance[7] = variance_xy;
    out.pose.covariance[14] = position_variance_z_;
    out.pose.covariance[21] = orientation_variance_;
    out.pose.covariance[28] = orientation_variance_;
    out.pose.covariance[35] = orientation_variance_;

    out.twist.covariance.fill(0.0);
    out.twist.covariance[0] = twist_variance_linear_;
    out.twist.covariance[7] = twist_variance_linear_;
    out.twist.covariance[14] = twist_variance_linear_;
    out.twist.covariance[21] = twist_variance_angular_;
    out.twist.covariance[28] = twist_variance_angular_;
    out.twist.covariance[35] = twist_variance_angular_;

    if (estimate_twist_) {
      estimate_twist(out);
    }

    last_stamp_ = rclcpp::Time(out.header.stamp);
    last_x_ = out.pose.pose.position.x;
    last_y_ = out.pose.pose.position.y;
    last_z_ = out.pose.pose.position.z;
    have_last_pose_ = true;

    publisher_->publish(out);
  }

  void set_origin(const auv_dvl_a50_msg::msg::DVLDR & msg, bool initial)
  {
    origin_x_ = msg.position.x;
    origin_y_ = msg.position.y;
    origin_z_ = msg.position.z;
    origin_roll_ = msg.roll;
    origin_pitch_ = msg.pitch;
    origin_yaw_ = msg.yaw;
    have_origin_ = true;
    if (initial) {
      RCLCPP_INFO(
        get_logger(),
        "DVLDR origin: position=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
        origin_x_, origin_y_, origin_z_, origin_roll_, origin_pitch_, origin_yaw_);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "DVLDR origin reset after jump: position=(%.3f, %.3f, %.3f)",
        origin_x_, origin_y_, origin_z_);
    }
  }

  bool is_reasonable_position(const nav_msgs::msg::Odometry & out) const
  {
    const auto x = out.pose.pose.position.x;
    const auto y = out.pose.pose.position.y;
    const auto z = out.pose.pose.position.z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }

    if (max_position_norm_ > 0.0 && std::sqrt(x * x + y * y + z * z) > max_position_norm_) {
      return false;
    }

    if (!have_last_pose_ || max_position_speed_ <= 0.0) {
      return true;
    }

    const auto stamp = rclcpp::Time(out.header.stamp);
    const auto dt = (stamp - last_stamp_).seconds();
    if (dt <= 0.0 || dt > max_twist_dt_) {
      return true;
    }

    const auto dx = x - last_x_;
    const auto dy = y - last_y_;
    const auto dz = z - last_z_;
    const auto speed = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
    return speed <= max_position_speed_;
  }

  void estimate_twist(nav_msgs::msg::Odometry & out)
  {
    if (!have_last_pose_) {
      return;
    }

    const auto stamp = rclcpp::Time(out.header.stamp);
    const auto dt = (stamp - last_stamp_).seconds();
    if (dt <= 0.0 || dt > max_twist_dt_) {
      return;
    }

    out.twist.twist.linear.x = (out.pose.pose.position.x - last_x_) / dt;
    out.twist.twist.linear.y = (out.pose.pose.position.y - last_y_) / dt;
    out.twist.twist.linear.z = (out.pose.pose.position.z - last_z_) / dt;
  }

  static double to_radians(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  double resolve_position_variance_xy(const auv_dvl_a50_msg::msg::DVLDR & msg) const
  {
    if (!use_dvl_position_stddev_) {
      return position_variance_xy_;
    }

    const auto stddev = static_cast<double>(msg.pos_std);
    if (!std::isfinite(stddev) || stddev <= 0.0) {
      return position_variance_xy_;
    }
    return stddev * stddev;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string expected_type_;

  bool zero_position_on_start_;
  bool zero_orientation_on_start_;
  bool orientation_in_degrees_;
  bool require_status_zero_;
  bool estimate_twist_;
  bool reset_origin_on_jump_;
  bool use_dvl_position_stddev_;

  double position_scale_x_;
  double position_scale_y_;
  double position_scale_z_;
  double max_twist_dt_;
  double max_position_norm_;
  double max_position_speed_;
  double position_variance_xy_;
  double position_variance_z_;
  double orientation_variance_;
  double twist_variance_linear_;
  double twist_variance_angular_;

  bool have_origin_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.0};
  double origin_roll_{0.0};
  double origin_pitch_{0.0};
  double origin_yaw_{0.0};

  bool have_last_pose_{false};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_z_{0.0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<auv_dvl_a50_msg::msg::DVLDR>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlPositionToOdomBridge>());
  rclcpp::shutdown();
  return 0;
}
