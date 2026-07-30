#pragma once

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

namespace hydrophone_ctrl
{
class ArenaFrameTransform
{
public:
    bool initialize(
        const Eigen::Vector2d & odom_origin,
        const double odom_yaw_rad)
    {
        if (initialized_) {
            return false;
        }
        if (!odom_origin.allFinite() || !std::isfinite(odom_yaw_rad)) {
            throw std::invalid_argument(
                "start-frame origin and yaw must be finite");
        }
        odom_origin_ = odom_origin;
        initial_yaw_rad_ = wrap_pi(odom_yaw_rad);
        cos_yaw_ = std::cos(initial_yaw_rad_);
        sin_yaw_ = std::sin(initial_yaw_rad_);
        initialized_ = true;
        return true;
    }

    bool initialized() const
    {
        return initialized_;
    }

    double initial_yaw_rad() const
    {
        return initial_yaw_rad_;
    }

    Eigen::Vector2d position_from_odom(
        const Eigen::Vector2d & odom_position) const
    {
        // odom 위치를 /start_frame 기준 위치로 변환한다.
        require_initialized();
        const Eigen::Vector2d relative = odom_position - odom_origin_;
        return {
            cos_yaw_ * relative.x() + sin_yaw_ * relative.y(),
            -sin_yaw_ * relative.x() + cos_yaw_ * relative.y()};
    }

    double yaw_from_odom(const double odom_yaw_rad) const
    {
        // odom 기준 yaw를 /start_frame 기준 yaw로 변환한다.
        require_initialized();
        return wrap_pi(odom_yaw_rad - initial_yaw_rad_);
    }

    static double yaw_from_quaternion(
        const double w, const double x, const double y, const double z)
    {
        return std::atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z));
    }

private:
    static double wrap_pi(const double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void require_initialized() const
    {
        if (!initialized_) {
            throw std::logic_error("arena frame transform is not initialized");
        }
    }

    Eigen::Vector2d odom_origin_{0.0, 0.0};
    double initial_yaw_rad_ = 0.0;
    double cos_yaw_ = 1.0;
    double sin_yaw_ = 0.0;
    bool initialized_ = false;
};
}  // namespace hydrophone_ctrl
