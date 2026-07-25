#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <dvl_msgs/msg/dvl.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class DvlToTwistBridge : public rclcpp::Node
{
public:
  DvlToTwistBridge()
  : Node("dvl_to_twist_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/dvl/data");
    output_topic_ = declare_parameter<std::string>("output_topic", "/dvl/twist");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "dvl_link");
    default_linear_variance_ = declare_parameter<double>("default_linear_variance", 0.02);
    min_linear_variance_ = declare_parameter<double>("min_linear_variance", 0.0);
    max_linear_variance_ = declare_parameter<double>("max_linear_variance", 1.0);
    covariance_scale_ = declare_parameter<double>("covariance_scale", 1.0);
    max_fom_ = declare_parameter<double>("max_fom", 0.05);
    min_altitude_ = declare_parameter<double>("min_altitude", 0.05);
    min_valid_beams_ = declare_parameter<int>("min_valid_beams", 3);
    use_dvl_covariance_ = declare_parameter<bool>("use_dvl_covariance", true);
    require_valid_velocity_ = declare_parameter<bool>("require_valid_velocity", true);
    reacquire_good_samples_ = declare_parameter<int>("reacquire_good_samples", 1);
    reacquire_duration_s_ = declare_parameter<double>("reacquire_duration_s", 0.0);
    input_velocity_is_frd_ = declare_parameter<bool>("input_velocity_is_frd", true);

    const auto sensor_qos = rclcpp::SensorDataQoS();

    publisher_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(output_topic_, 10);
    subscription_ = create_subscription<dvl_msgs::msg::DVL>(
      input_topic_, sensor_qos,
      std::bind(&DvlToTwistBridge::handle_msg, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Bridging %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "DVL gates: min_var=%.4g max_var=%.4g cov_scale=%.3f max_fom=%.4g min_alt=%.3f min_beams=%d",
      min_linear_variance_, max_linear_variance_, covariance_scale_, max_fom_,
      min_altitude_, min_valid_beams_);
    RCLCPP_INFO(
      get_logger(),
      "DVL reacquire gate: good_samples=%d duration=%.2fs",
      reacquire_good_samples_, reacquire_duration_s_);
    RCLCPP_INFO(
      get_logger(),
      "DVL velocity frame conversion: input=%s output=ROS FLU",
      input_velocity_is_frd_ ? "sensor FRD (x forward, y right, z down)" : "ROS FLU");
  }

private:
  void handle_msg(const dvl_msgs::msg::DVL::SharedPtr msg)
  {
    if (require_valid_velocity_ && !msg->velocity_valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample marked invalid.");
      reset_reacquisition();
      return;
    }
    if (!is_valid_measurement(*msg)) {
      reset_reacquisition();
      return;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped out;
    out.header = msg->header;
    if (!output_frame_id_.empty()) {
      out.header.frame_id = output_frame_id_;
    }

    // The physical Water Linked A50 driver publishes the sensor JSON velocity
    // unchanged: x forward, y right, z down (FRD).  robot_localization expects
    // a ROS body-frame twist: x forward, y left, z up (FLU).  Keep this
    // conversion at the physical/simulator message boundary so both producers
    // obey exactly the same /dvl/data contract.
    out.twist.twist.linear.x = msg->velocity.x;
    out.twist.twist.linear.y = input_velocity_is_frd_ ? -msg->velocity.y : msg->velocity.y;
    out.twist.twist.linear.z = input_velocity_is_frd_ ? -msg->velocity.z : msg->velocity.z;

    auto & cov = out.twist.covariance;
    cov.fill(0.0);

    if (use_dvl_covariance_ && msg->covariance.size() == 9) {
      for (size_t row = 0; row < 3; ++row) {
        for (size_t col = 0; col < 3; ++col) {
          cov[row * 6 + col] = msg->covariance[row * 3 + col] * covariance_scale_;
        }
      }
    } else if (use_dvl_covariance_ && msg->covariance.size() == 36) {
      for (size_t index = 0; index < cov.size(); ++index) {
        cov[index] = msg->covariance[index] * covariance_scale_;
      }
    } else {
      cov[0] = default_linear_variance_;
      cov[7] = default_linear_variance_;
      cov[14] = default_linear_variance_;
    }

    if (input_velocity_is_frd_) {
      convert_covariance_frd_to_flu(cov);
    }

    if (has_rejected_covariance(cov)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample with excessive covariance.");
      reset_reacquisition();
      return;
    }

    cov[0] = sanitize_variance(cov[0]);
    cov[7] = sanitize_variance(cov[7]);
    cov[14] = sanitize_variance(cov[14]);

    if (!is_reacquired()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Holding DVL twist during reacquire: %d/%d good samples.",
        consecutive_good_samples_, reacquire_good_samples_);
      return;
    }

    publisher_->publish(out);
  }

  bool is_valid_measurement(const dvl_msgs::msg::DVL & msg)
  {
    const auto & velocity = msg.velocity;
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample with non-finite velocity.");
      return false;
    }

    if (min_altitude_ > 0.0 && (!std::isfinite(msg.altitude) || msg.altitude < min_altitude_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample with low/invalid altitude: %.3f m.", msg.altitude);
      return false;
    }

    if (max_fom_ > 0.0 && (!std::isfinite(msg.fom) || msg.fom > max_fom_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample with high FOM: %.6f.", msg.fom);
      return false;
    }

    if (min_valid_beams_ > 0) {
      const auto valid_beams = static_cast<int>(std::count_if(
        msg.beams.begin(), msg.beams.end(),
        [](const auto & beam) { return beam.valid; }));
      if (valid_beams < min_valid_beams_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping DVL sample with %d valid beams.", valid_beams);
        return false;
      }
    }

    return true;
  }

  bool has_rejected_covariance(const std::array<double, 36> & cov) const
  {
    if (max_linear_variance_ <= 0.0) {
      return false;
    }
    return exceeds_max_variance(cov[0]) || exceeds_max_variance(cov[7]) ||
           exceeds_max_variance(cov[14]);
  }

  bool exceeds_max_variance(double value) const
  {
    return !std::isfinite(value) || value > max_linear_variance_;
  }

  double sanitize_variance(double value) const
  {
    if (min_linear_variance_ <= 0.0) {
      return value;
    }
    if (!std::isfinite(value) || value <= 0.0) {
      return min_linear_variance_;
    }
    return std::max(value, min_linear_variance_);
  }

  static void convert_covariance_frd_to_flu(std::array<double, 36> & covariance)
  {
    // C_flu = S C_frd S^T.  The angular signs are included for a complete
    // Twist covariance even though the current DVL only supplies linear data.
    constexpr std::array<double, 6> signs{{1.0, -1.0, -1.0, 1.0, -1.0, -1.0}};
    for (size_t row = 0; row < signs.size(); ++row) {
      for (size_t col = 0; col < signs.size(); ++col) {
        covariance[row * 6 + col] *= signs[row] * signs[col];
      }
    }
  }

  void reset_reacquisition()
  {
    reacquired_ = false;
    consecutive_good_samples_ = 0;
    first_good_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  bool is_reacquired()
  {
    if (reacquire_good_samples_ <= 1 && reacquire_duration_s_ <= 0.0) {
      reacquired_ = true;
      return true;
    }

    const auto now = get_clock()->now();
    if (consecutive_good_samples_ == 0) {
      first_good_time_ = now;
    }
    consecutive_good_samples_++;

    const bool samples_ok = consecutive_good_samples_ >= std::max(1, reacquire_good_samples_);
    const bool duration_ok =
      reacquire_duration_s_ <= 0.0 || (now - first_good_time_).seconds() >= reacquire_duration_s_;
    reacquired_ = samples_ok && duration_ok;
    return reacquired_;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  double default_linear_variance_;
  double min_linear_variance_;
  double max_linear_variance_;
  double covariance_scale_;
  double max_fom_;
  double min_altitude_;
  int min_valid_beams_;
  bool use_dvl_covariance_;
  bool require_valid_velocity_;
  int reacquire_good_samples_;
  double reacquire_duration_s_;
  bool input_velocity_is_frd_;
  bool reacquired_{false};
  int consecutive_good_samples_{0};
  rclcpp::Time first_good_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlToTwistBridge>());
  rclcpp::shutdown();
  return 0;
}
