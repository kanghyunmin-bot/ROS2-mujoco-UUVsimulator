/*
 * Copyright 2026.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Vision Position Delta plugin (using DVL-A50 topics)
 * @file vision_position_delta.cpp
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <Eigen/Core>

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"

#if __has_include("auv_dvl_a50_msg/msg/dvl.hpp") && __has_include("auv_dvl_a50_msg/msg/dvldr.hpp")
#define MAVROS_HAS_DVL_A50_MSGS 1
#include "auv_dvl_a50_msg/msg/dvl.hpp"
#include "auv_dvl_a50_msg/msg/dvldr.hpp"
#else
#define MAVROS_HAS_DVL_A50_MSGS 0
#endif

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

static inline double wrap_angle_pi_std(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

static inline double degrees_to_radians(const double degrees)
{
  return degrees * M_PI / 180.0;
}

/**
 * @brief Vision Position Delta plugin
 * @plugin vision_position_delta
 *
 * Subscribes to dvl_a50 topics and sends VISION_POSITION_DELTA MAVLink
 * messages.
 */
#if MAVROS_HAS_DVL_A50_MSGS
class VisionPositionDeltaPlugin : public plugin::Plugin
{
public:
  explicit VisionPositionDeltaPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "vision_position_delta"),
    last_attitude_rad(0.0, 0.0, 0.0),
    latest_attitude_rad(0.0, 0.0, 0.0),
    last_velocity_stamp(0, 0, RCL_ROS_TIME)
  {
    const auto orientation_param = node->declare_parameter<int>(
      "dvl_orientation", static_cast<int>(DvlOrientation::DOWN));
    dvl_orientation = parse_dvl_orientation(orientation_param);

    dvl_time_in_ms = node->declare_parameter<bool>("dvl_time_in_ms", true);
    const auto fom_max_param = node->declare_parameter<double>("fom_max", 0.4);
    if (std::isfinite(fom_max_param) && fom_max_param > 0.0) {
      fom_max = static_cast<float>(fom_max_param);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid fom_max. Falling back to 0.4.");
    }

    const auto dvl_topic = node->declare_parameter<std::string>("dvl_topic", "/dvl/data");
    const auto dvl_dr_topic = node->declare_parameter<std::string>("dvl_dr_topic", "/dvl/position");

    const auto sensor_qos = rclcpp::SensorDataQoS();

    dvl_sub = node->create_subscription<DvlMsg>(
      dvl_topic, sensor_qos, std::bind(&VisionPositionDeltaPlugin::dvl_cb, this, _1));
    dvl_dr_sub = node->create_subscription<DvlDrMsg>(
      dvl_dr_topic, sensor_qos, std::bind(&VisionPositionDeltaPlugin::dvl_dr_cb, this, _1));

    RCLCPP_INFO(
      get_logger(),
      "VisionPositionDeltaPlugin initialized. Subscribed to '%s' and '%s'.",
      dvl_topic.c_str(), dvl_dr_topic.c_str());
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  using DvlMsg = auv_dvl_a50_msg::msg::DVL;
  using DvlDrMsg = auv_dvl_a50_msg::msg::DVLDR;

  enum class DvlOrientation : int
  {
    DOWN = 1,
    FORWARD = 2
  };

  rclcpp::Subscription<DvlMsg>::SharedPtr dvl_sub;
  rclcpp::Subscription<DvlDrMsg>::SharedPtr dvl_dr_sub;

  DvlOrientation dvl_orientation{DvlOrientation::DOWN};
  bool dvl_time_in_ms{true};
  float fom_max{0.4F};

  std::mutex attitude_mutex;
  Eigen::Vector3d last_attitude_rad;
  Eigen::Vector3d latest_attitude_rad;
  bool has_latest_attitude{false};
  bool has_attitude_delta_seed{false};
  rclcpp::Time last_velocity_stamp;

  DvlOrientation parse_dvl_orientation(const int value)
  {
    if (value == static_cast<int>(DvlOrientation::FORWARD)) {
      return DvlOrientation::FORWARD;
    }
    if (value != static_cast<int>(DvlOrientation::DOWN)) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid dvl_orientation=%d. Falling back to DOWN(1).", value);
    }
    return DvlOrientation::DOWN;
  }

  uint32_t compute_time_delta_us(const DvlMsg::SharedPtr msg, const rclcpp::Time & stamp)
  {
    uint32_t time_delta_us = 0U;

    if (std::isfinite(msg->time) && msg->time > 0.0) {
      const double scale = dvl_time_in_ms ? 1000.0 : 1000000.0;
      const double candidate = msg->time * scale;
      if (
        candidate > 0.0 &&
        candidate <= static_cast<double>(std::numeric_limits<uint32_t>::max()))
      {
        time_delta_us = static_cast<uint32_t>(std::llround(candidate));
      }
    }

    if (time_delta_us == 0U && last_velocity_stamp.nanoseconds() != 0) {
      const auto dt_ns = (stamp - last_velocity_stamp).nanoseconds();
      if (dt_ns > 0) {
        const uint64_t dt_us_64 = static_cast<uint64_t>(dt_ns / 1000);
        time_delta_us = static_cast<uint32_t>(
          std::min<uint64_t>(dt_us_64, std::numeric_limits<uint32_t>::max()));
      }
    }

    last_velocity_stamp = stamp;
    return time_delta_us;
  }

  void send_vision_position_delta(
    const rclcpp::Time & stamp, const uint32_t time_delta_us,
    const Eigen::Vector3d & angle_delta,
    const Eigen::Vector3d & position_delta, const float confidence)
  {
    float clamped_confidence = std::clamp(confidence, 0.0f, 100.0f);
    if (!std::isfinite(clamped_confidence)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Non-finite confidence calculated. Sending 0.");
      clamped_confidence = 0.0f;
    }

    mavlink::ardupilotmega::msg::VISION_POSITION_DELTA vpd{};

    vpd.time_usec = stamp.nanoseconds() / 1000;
    vpd.time_delta_usec = time_delta_us;

    vpd.angle_delta[0] = static_cast<float>(angle_delta.x());
    vpd.angle_delta[1] = static_cast<float>(angle_delta.y());
    vpd.angle_delta[2] = static_cast<float>(angle_delta.z());

    vpd.position_delta[0] = static_cast<float>(position_delta.x());
    vpd.position_delta[1] = static_cast<float>(position_delta.y());
    vpd.position_delta[2] = static_cast<float>(position_delta.z());

    vpd.confidence = clamped_confidence;

    uas->send_message(vpd);
  }

  void dvl_dr_cb(const DvlDrMsg::SharedPtr msg)
  {
    if (!std::isfinite(msg->roll) || !std::isfinite(msg->pitch) || !std::isfinite(msg->yaw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Received non-finite DVLDR attitude. Ignoring frame.");
      return;
    }

    std::lock_guard<std::mutex> lock(attitude_mutex);
    latest_attitude_rad.x() = degrees_to_radians(msg->roll);
    latest_attitude_rad.y() = degrees_to_radians(msg->pitch);
    latest_attitude_rad.z() = degrees_to_radians(msg->yaw);
    has_latest_attitude = true;
  }

  void dvl_cb(const DvlMsg::SharedPtr msg)
  {
    if (!msg->velocity_valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received DVL data marked as invalid. Skipping VISION_POSITION_DELTA.");
      return;
    }

    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = node->now();
    }

    const uint32_t time_delta_us = compute_time_delta_us(msg, stamp);
    if (time_delta_us == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Could not determine time delta. Skipping frame.");
      return;
    }
    const double dt_sec = static_cast<double>(time_delta_us) / 1000000.0;

    const double vx = msg->velocity.x;
    const double vy = msg->velocity.y;
    const double vz = msg->velocity.z;
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Received non-finite velocity. Skipping frame.");
      return;
    }

    const double dx = vx * dt_sec;
    const double dy = vy * dt_sec;
    const double dz = vz * dt_sec;

    Eigen::Vector3d position_delta_frd;
    if (dvl_orientation == DvlOrientation::FORWARD) {
      position_delta_frd.x() = dz;
      position_delta_frd.y() = dy;
      position_delta_frd.z() = -dx;
    } else {
      position_delta_frd.x() = dx;
      position_delta_frd.y() = dy;
      position_delta_frd.z() = dz;
    }

    Eigen::Vector3d current_attitude_rad{0.0, 0.0, 0.0};
    bool has_attitude = false;
    {
      std::lock_guard<std::mutex> lock(attitude_mutex);
      has_attitude = has_latest_attitude;
      if (has_attitude) {
        current_attitude_rad = latest_attitude_rad;
      }
    }

    Eigen::Vector3d angle_delta_rad{0.0, 0.0, 0.0};
    if (has_attitude) {
      if (!has_attitude_delta_seed) {
        last_attitude_rad = current_attitude_rad;
        has_attitude_delta_seed = true;
      } else {
        angle_delta_rad = current_attitude_rad - last_attitude_rad;
        angle_delta_rad.z() = wrap_angle_pi_std(angle_delta_rad.z());
        last_attitude_rad = current_attitude_rad;
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No DVLDR attitude received yet. Sending zero angle_delta.");
    }

    Eigen::Vector3d angle_delta_frd;
    if (dvl_orientation == DvlOrientation::FORWARD) {
      angle_delta_frd.x() = angle_delta_rad.z();
      angle_delta_frd.y() = angle_delta_rad.y();
      angle_delta_frd.z() = -angle_delta_rad.x();
    } else {
      angle_delta_frd.x() = angle_delta_rad.x();
      angle_delta_frd.y() = angle_delta_rad.y();
      angle_delta_frd.z() = angle_delta_rad.z();
    }

    float confidence = 0.0F;
    if (std::isfinite(msg->fom) && fom_max > 0.0F) {
      confidence = 100.0F * (
        1.0F - std::min(fom_max, std::max(0.0F, static_cast<float>(msg->fom))) / fom_max);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Invalid FOM value. Sending zero confidence.");
    }

    send_vision_position_delta(
      stamp, time_delta_us, angle_delta_frd, position_delta_frd, confidence);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Sent VISION_POSITION_DELTA: dt_us=%u, conf=%.1f%%",
      time_delta_us, confidence);
  }
};
#else
class VisionPositionDeltaPlugin : public plugin::Plugin
{
public:
  explicit VisionPositionDeltaPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "vision_position_delta")
  {
    RCLCPP_ERROR(
      get_logger(),
      "auv_dvl_a50_msg::msg::DVL/DVLDR not found. vision_position_delta plugin is disabled.");
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }
};
#endif
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VisionPositionDeltaPlugin)
