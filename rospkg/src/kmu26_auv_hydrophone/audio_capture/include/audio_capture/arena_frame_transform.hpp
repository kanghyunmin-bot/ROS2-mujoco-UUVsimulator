#pragma once

#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace audio_capture
{
class ArenaFrameTransform2D
{
public:
    bool update(
        const geometry_msgs::msg::PoseStamped & message,
        std::string & error)
    {
        if (message.header.frame_id.empty()) {
            error = "parent frame is empty";
            return false;
        }

        const auto & position = message.pose.position;
        const auto & orientation = message.pose.orientation;
        const double norm = std::sqrt(
            orientation.x * orientation.x +
            orientation.y * orientation.y +
            orientation.z * orientation.z +
            orientation.w * orientation.w);
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z) || !std::isfinite(norm) ||
            norm <= 1.0e-9)
        {
            error = "origin or orientation is invalid";
            return false;
        }

        const double x = orientation.x / norm;
        const double y = orientation.y / norm;
        const double z = orientation.z / norm;
        const double w = orientation.w / norm;
        const double yaw = std::atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z));
        if (!std::isfinite(yaw)) {
            error = "yaw is invalid";
            return false;
        }

        parent_frame_ = normalized_frame(message.header.frame_id);
        origin_ = {position.x, position.y};
        origin_z_ = position.z;
        yaw_rad_ = yaw;
        ready_ = true;
        error.clear();
        return true;
    }

    bool ready() const
    {
        return ready_;
    }

    const std::string & parent_frame() const
    {
        return parent_frame_;
    }

    const Eigen::Vector2d & origin() const
    {
        return origin_;
    }

    double origin_z() const
    {
        return origin_z_;
    }

    double yaw_rad() const
    {
        return yaw_rad_;
    }

    Eigen::Vector2d odom_to_arena(const Eigen::Vector2d & odom) const
    {
        const Eigen::Vector2d delta = odom - origin_;
        const double cosine = std::cos(yaw_rad_);
        const double sine = std::sin(yaw_rad_);
        return {
            cosine * delta.x() + sine * delta.y(),
            -sine * delta.x() + cosine * delta.y()};
    }

    Eigen::Vector2d arena_to_odom(const Eigen::Vector2d & arena) const
    {
        const double cosine = std::cos(yaw_rad_);
        const double sine = std::sin(yaw_rad_);
        return origin_ + Eigen::Vector2d(
            cosine * arena.x() - sine * arena.y(),
            sine * arena.x() + cosine * arena.y());
    }

    static std::string normalized_frame(const std::string & frame)
    {
        const auto first = frame.find_first_not_of('/');
        return first == std::string::npos ? std::string() : frame.substr(first);
    }

private:
    bool ready_ = false;
    std::string parent_frame_;
    Eigen::Vector2d origin_{0.0, 0.0};
    double origin_z_ = 0.0;
    double yaw_rad_ = 0.0;
};
}  // namespace audio_capture
