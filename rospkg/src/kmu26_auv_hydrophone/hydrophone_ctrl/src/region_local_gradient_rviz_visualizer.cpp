#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <Eigen/Dense>
#include "hydrophone_ctrl/arena_frame_transform.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace audio_capture
{
// Timestamp가 있는 SNR과 odometry로 2D SNR 격자를 만들고 Region Gradient를
// 현재 AUV 위치에서 시작하는 RViz MarkerArray 화살표로 표시한다.
class RegionLocalGradientRvizVisualizerNode : public rclcpp::Node
{
public:
    explicit RegionLocalGradientRvizVisualizerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("region_local_gradient_rviz_visualizer", options)
    {
        const auto snr_topic = declare_parameter<std::string>(
            "snr_topic", "/audio_frequency_detector/snr_db_stamped");
        const auto odometry_topic = declare_parameter<std::string>(
            "odometry_topic", "/odometry/filtered");
        const auto start_frame_topic = declare_parameter<std::string>(
            "start_frame_topic", "/start_frame");
        const auto region_gradient_topic = declare_parameter<std::string>(
            "region_gradient_topic", "/homing/region_gradient");
        const auto rolling_gradient_topic = declare_parameter<std::string>(
            "rolling_gradient_topic", "/homing/rolling_gradient");
        const auto homing_direction_topic = declare_parameter<std::string>(
            "homing_direction_topic", "/homing/homing_direction");
        const auto waypoint_topic = declare_parameter<std::string>(
            "waypoint_topic", "/homing/current_waypoint");
        const auto scan_center_topic = declare_parameter<std::string>(
            "scan_center_topic", "/homing/scan_center");
        const auto state_topic = declare_parameter<std::string>(
            "state_topic", "/homing/control_state");
        const auto marker_topic = declare_parameter<std::string>(
            "marker_topic", "/homing/rviz/markers");
        arena_frame_id_ = declare_parameter<std::string>(
            "arena_frame_id", "arena");

        arena_length_m_ = std::max(
            0.1, declare_parameter<double>("arena_length_m", 15.0));
        arena_width_m_ = std::max(
            0.1, declare_parameter<double>("arena_width_m", 16.0));
        arena_offset_x_m_ = declare_parameter<double>("arena_offset_x_m", 0.0);
        arena_offset_y_m_ = declare_parameter<double>("arena_offset_y_m", 0.0);
        arena_safety_margin_m_ = std::max(
            0.0, declare_parameter<double>("arena_safety_margin_m", 0.5));
        vision_near_zone_width_m_ = std::clamp(
            declare_parameter<double>("vision_near_zone_width_m", 2.0),
            0.0, arena_width_m_);
        scan_radius_m_ = std::max(
            0.1, declare_parameter<double>("scan_radius_m", 1.30));
        owned_region_enabled_ = declare_parameter<bool>(
            "owned_region_enabled", false);
        owned_region_x_min_m_ = declare_parameter<double>(
            "owned_region_x_min_m", 0.0);
        owned_region_x_max_m_ = declare_parameter<double>(
            "owned_region_x_max_m", 0.0);
        owned_region_y_min_m_ = declare_parameter<double>(
            "owned_region_y_min_m", 0.0);
        owned_region_y_max_m_ = declare_parameter<double>(
            "owned_region_y_max_m", 0.0);
        trajectory_spacing_m_ = std::max(
            0.01, declare_parameter<double>("trajectory_spacing_m", 0.04));
        arena_start_corner_ = declare_parameter<std::string>(
            "arena_start_corner", "bottom_left");
        if (!std::isfinite(arena_offset_x_m_) || !std::isfinite(arena_offset_y_m_) ||
            (arena_start_corner_ != "bottom_left" &&
            arena_start_corner_ != "bottom_right"))
        {
            throw std::invalid_argument("invalid arena offset or start corner");
        }
        if (2.0 * arena_safety_margin_m_ >=
            std::min(arena_length_m_, arena_width_m_))
        {
            throw std::invalid_argument("arena safety margin leaves no waypoint area");
        }

        map_cell_size_m_ = std::max(
            0.02, declare_parameter<double>("map_cell_size_m", 0.15));
        map_cell_height_m_ = std::max(
            0.005, declare_parameter<double>("map_cell_height_m", 0.04));
        max_cell_values_ = static_cast<std::size_t>(std::max<std::int64_t>(
            1, declare_parameter<std::int64_t>("max_cell_values", 31)));
        max_odometry_extrapolation_s_ = std::max(
            0.0, declare_parameter<double>("max_odometry_extrapolation_s", 0.15));
        dynamic_snr_range_ = declare_parameter<bool>("dynamic_snr_range", true);
        snr_color_min_db_ = declare_parameter<double>("snr_color_min_db", -10.0);
        snr_color_max_db_ = declare_parameter<double>("snr_color_max_db", 20.0);
        if (snr_color_max_db_ <= snr_color_min_db_) {
            snr_color_max_db_ = snr_color_min_db_ + 1.0;
        }
        arrow_length_m_ = std::max(
            0.1, declare_parameter<double>("arrow_length_m", 1.0));
        map_z_m_ = declare_parameter<double>("map_z_m", 0.0);
        const double publish_rate_hz = std::clamp(
            declare_parameter<double>("publish_rate_hz", 5.0), 0.5, 30.0);

        snr_sub_ = create_subscription<audio_common_msgs::msg::Float64Stamped>(
            snr_topic, 20,
            std::bind(&RegionLocalGradientRvizVisualizerNode::snr_callback, this,
                std::placeholders::_1));
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 30,
            std::bind(&RegionLocalGradientRvizVisualizerNode::odometry_callback, this,
                std::placeholders::_1));
        start_frame_sub_ =
            create_subscription<geometry_msgs::msg::PoseStamped>(
                start_frame_topic,
                rclcpp::QoS(1).reliable().transient_local(),
                std::bind(
                    &RegionLocalGradientRvizVisualizerNode::start_frame_callback,
                    this, std::placeholders::_1));
        region_gradient_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            region_gradient_topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
                update_gradient(msg, region_gradient_, have_region_gradient_);
            });
        rolling_gradient_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            rolling_gradient_topic, 10,
            [this](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
                update_gradient(msg, rolling_gradient_, have_rolling_gradient_);
            });
        homing_direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            homing_direction_topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
                update_gradient(msg, homing_direction_, have_homing_direction_);
            });
        waypoint_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            waypoint_topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
                const Eigen::Vector2d waypoint(msg->point.x, msg->point.y);
                if (waypoint.allFinite()) {
                    current_waypoint_ = waypoint;
                    have_waypoint_ = true;
                }
            });
        scan_center_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            scan_center_topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
                const Eigen::Vector2d center(msg->point.x, msg->point.y);
                if (center.allFinite()) {
                    scan_center_ = center;
                    have_scan_center_ = true;
                }
            });
        state_sub_ = create_subscription<std_msgs::msg::String>(
            state_topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::String::ConstSharedPtr msg) {
                if (msg->data == "REGION_SCAN" && control_state_ != msg->data) {
                    have_peak_snr_ = false;
                    peak_snr_db_ = -240.0;
                }
                control_state_ = msg->data;
            });
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic, 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&RegionLocalGradientRvizVisualizerNode::publish_markers, this));

        RCLCPP_INFO(
            get_logger(),
            "RViz SNR visualizer ready. marker_topic=%s cell=%.2f m "
            "follow_tf=%s->%s",
            marker_topic.c_str(), map_cell_size_m_,
            arena_frame_id_.c_str(), vehicle_follow_frame_id_.c_str());
    }

private:
    struct PoseSample
    {
        rclcpp::Time stamp;
        Eigen::Vector2d position{0.0, 0.0};
    };

    struct PendingSnr
    {
        rclcpp::Time stamp;
        double snr_db = 0.0;
    };

    struct GridCell
    {
        std::deque<double> values;
    };

    enum class PoseLookup
    {
        FOUND,
        WAIT_FOR_ODOMETRY,
        TOO_OLD
    };

    void start_frame_callback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        if (arena_transform_.initialized()) {
            return;
        }
        const Eigen::Vector2d origin(
            msg->pose.position.x, msg->pose.position.y);
        const double yaw = hydrophone_ctrl::ArenaFrameTransform::
            yaw_from_quaternion(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        if (!origin.allFinite() || !std::isfinite(yaw)) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid guided start frame");
            return;
        }
        arena_transform_.initialize(origin, yaw);
        RCLCPP_INFO(
            get_logger(),
            "Guided start frame accepted: origin_odom=(%.3f, %.3f), "
            "yaw_odom=%.3f rad; arena boundary offset in start frame=(%.2f, %.2f)",
            origin.x(), origin.y(), yaw,
            arena_offset_x_m_, arena_offset_y_m_);
    }

    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        const Eigen::Vector2d odom_position(
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        const rclcpp::Time stamp(msg->header.stamp);
        if (!odom_position.allFinite() || stamp.nanoseconds() <= 0 ||
            !arena_transform_.initialized())
        {
            return;
        }
        const Eigen::Vector2d position =
            arena_transform_.position_from_odom(odom_position);
        if (!odometry_history_.empty() && stamp < odometry_history_.back().stamp) {
            odometry_history_.clear();
            pending_snr_.clear();
            grid_.clear();
            have_region_gradient_ = false;
            have_rolling_gradient_ = false;
            have_homing_direction_ = false;
            have_waypoint_ = false;
            have_scan_center_ = false;
            trajectory_.clear();
            have_peak_snr_ = false;
        }
        odometry_history_.push_back({stamp, position});
        while (odometry_history_.size() > 500) {
            odometry_history_.pop_front();
        }
        current_position_ = position;
        have_odometry_ = true;
        publish_vehicle_follow_transform(stamp);
        if (trajectory_.empty() ||
            (position - trajectory_.back()).norm() >= trajectory_spacing_m_)
        {
            trajectory_.push_back(position);
            while (trajectory_.size() > max_trajectory_points_) {
                trajectory_.pop_front();
            }
        }
        process_pending_snr();
    }

    void publish_vehicle_follow_transform(const rclcpp::Time & stamp)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = arena_frame_id_;
        transform.child_frame_id = vehicle_follow_frame_id_;
        transform.transform.translation.x = current_position_.x();
        transform.transform.translation.y = current_position_.y();
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(transform);
    }

    void snr_callback(const audio_common_msgs::msg::Float64Stamped::ConstSharedPtr msg)
    {
        const rclcpp::Time stamp(msg->header.stamp);
        if (!std::isfinite(msg->data) || stamp.nanoseconds() <= 0) {
            return;
        }
        latest_snr_db_ = msg->data;
        have_latest_snr_ = true;
        const PendingSnr sample{stamp, std::clamp(msg->data, -240.0, 80.0)};
        const auto insert_at = std::upper_bound(
            pending_snr_.begin(), pending_snr_.end(), stamp,
            [](const rclcpp::Time & time, const PendingSnr & pending) {
                return time < pending.stamp;
            });
        pending_snr_.insert(insert_at, sample);
        while (pending_snr_.size() > 500) {
            pending_snr_.pop_front();
        }
        process_pending_snr();
    }

    void process_pending_snr()
    {
        while (!pending_snr_.empty() && !odometry_history_.empty()) {
            Eigen::Vector2d position;
            const PoseLookup lookup = position_at(pending_snr_.front().stamp, position);
            if (lookup == PoseLookup::WAIT_FOR_ODOMETRY) {
                return;
            }
            const PendingSnr sample = pending_snr_.front();
            pending_snr_.pop_front();
            if (lookup == PoseLookup::TOO_OLD) {
                continue;
            }
            if (!have_peak_snr_ || sample.snr_db > peak_snr_db_) {
                peak_snr_db_ = sample.snr_db;
                peak_snr_position_ = position;
                have_peak_snr_ = true;
            }
            const auto cell_index = std::make_pair(
                static_cast<int>(std::floor(position.x() / map_cell_size_m_)),
                static_cast<int>(std::floor(position.y() / map_cell_size_m_)));
            GridCell & cell = grid_[cell_index];
            cell.values.push_back(sample.snr_db);
            while (cell.values.size() > max_cell_values_) {
                cell.values.pop_front();
            }
        }
    }

    PoseLookup position_at(const rclcpp::Time & stamp, Eigen::Vector2d & position) const
    {
        if (stamp <= odometry_history_.front().stamp) {
            if ((odometry_history_.front().stamp - stamp).seconds() >
                max_odometry_extrapolation_s_)
            {
                return PoseLookup::TOO_OLD;
            }
            position = odometry_history_.front().position;
            return PoseLookup::FOUND;
        }
        if (stamp >= odometry_history_.back().stamp) {
            if ((stamp - odometry_history_.back().stamp).seconds() >
                max_odometry_extrapolation_s_)
            {
                return PoseLookup::WAIT_FOR_ODOMETRY;
            }
            position = odometry_history_.back().position;
            return PoseLookup::FOUND;
        }
        for (std::size_t i = 1; i < odometry_history_.size(); ++i) {
            if (odometry_history_[i].stamp >= stamp) {
                const PoseSample & before = odometry_history_[i - 1];
                const PoseSample & after = odometry_history_[i];
                const double duration = (after.stamp - before.stamp).seconds();
                const double alpha = duration > 0.0 ?
                    (stamp - before.stamp).seconds() / duration : 0.0;
                position = (1.0 - alpha) * before.position + alpha * after.position;
                return PoseLookup::FOUND;
            }
        }
        return PoseLookup::WAIT_FOR_ODOMETRY;
    }

    void update_gradient(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr & msg,
        Eigen::Vector2d & gradient,
        bool & received)
    {
        const Eigen::Vector2d value(msg->vector.x, msg->vector.y);
        const double magnitude = value.norm();
        if (!value.allFinite() || !std::isfinite(magnitude) || magnitude < 1.0e-6) {
            return;
        }
        gradient = value / magnitude;
        received = true;
    }

    void publish_markers()
    {
        if (!have_odometry_) {
            return;
        }
        const rclcpp::Time stamp = now();
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(make_arena_marker(
            stamp, 0.0, "arena_boundary", 0.1F, 0.8F, 1.0F));
        markers.markers.push_back(make_arena_marker(
            stamp, arena_safety_margin_m_, "waypoint_safe_boundary",
            1.0F, 0.75F, 0.0F));
        if (owned_region_enabled_) {
            markers.markers.push_back(make_owned_region_marker(stamp));
        }
        if (vision_near_zone_width_m_ > 0.0) {
            markers.markers.push_back(make_vision_zone_marker(stamp));
            markers.markers.push_back(make_vision_zone_label_marker(stamp));
        }
        markers.markers.push_back(make_snr_map_marker(stamp));
        markers.markers.push_back(make_trajectory_marker(stamp));
        markers.markers.push_back(make_auv_marker(stamp));
        if (have_scan_center_) {
            markers.markers.push_back(make_scan_center_marker(stamp));
            markers.markers.push_back(make_scan_center_label_marker(stamp));
            markers.markers.push_back(make_scan_circle_marker(stamp));
        }
        if (have_peak_snr_) {
            markers.markers.push_back(make_peak_snr_marker(stamp));
        }
        // 재스캔 중에도 마지막으로 확정된 Region Gradient를 유지해 표시한다.
        if (have_region_gradient_) {
            markers.markers.push_back(make_arrow_marker(
                stamp, "region_gradient", 0, region_gradient_, 0.0F, 1.0F, 0.0F));
            markers.markers.push_back(make_text_marker(
                stamp, "region_gradient_label", 0, region_gradient_,
                "Region Gradient", 0.0F, 1.0F, 0.0F));
        }
        if (have_rolling_gradient_) {
            markers.markers.push_back(make_arrow_marker(
                stamp, "rolling_gradient", 0, rolling_gradient_, 1.0F, 0.45F, 0.0F));
            markers.markers.push_back(make_text_marker(
                stamp, "rolling_gradient_label", 0, rolling_gradient_,
                "Rolling Gradient", 1.0F, 0.45F, 0.0F));
        }
        if (have_homing_direction_) {
            markers.markers.push_back(make_arrow_marker(
                stamp, "homing_direction", 0, homing_direction_, 0.0F, 0.9F, 1.0F));
            markers.markers.push_back(make_text_marker(
                stamp, "homing_direction_label", 0, homing_direction_,
                "Homing Direction", 0.0F, 0.9F, 1.0F));
        }
        if (have_waypoint_) {
            markers.markers.push_back(make_waypoint_marker(stamp));
            markers.markers.push_back(make_waypoint_line_marker(stamp));
            markers.markers.push_back(make_waypoint_label_marker(stamp));
        }
        markers.markers.push_back(make_status_marker(stamp));
        marker_pub_->publish(markers);
    }

    visualization_msgs::msg::Marker make_arena_marker(
        const rclcpp::Time & stamp,
        const double inset,
        const std::string & name,
        const float red,
        const float green,
        const float blue) const
    {
        const double x_min = arena_offset_x_m_ + inset;
        const double x_max =
            arena_offset_x_m_ + arena_length_m_ - inset;
        const double y_min = arena_start_corner_ == "bottom_left" ?
            arena_offset_y_m_ - arena_width_m_ + inset :
            arena_offset_y_m_ + inset;
        const double y_max = arena_start_corner_ == "bottom_left" ?
            arena_offset_y_m_ - inset :
            arena_offset_y_m_ + arena_width_m_ - inset;

        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, name, 0);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.04;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 1.0F;

        const auto point = [this](const double x, const double y) {
            geometry_msgs::msg::Point value;
            value.x = x;
            value.y = y;
            value.z = map_z_m_ + 0.02;
            return value;
        };
        marker.points = {
            point(x_min, y_min), point(x_max, y_min), point(x_max, y_max),
            point(x_min, y_max), point(x_min, y_min)};
        return marker;
    }

    visualization_msgs::msg::Marker make_owned_region_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "owned_region_boundary", 0);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.055;
        marker.color.r = 0.2F;
        marker.color.g = 1.0F;
        marker.color.b = 0.35F;
        marker.color.a = 0.95F;
        const auto point = [this](const double x, const double y) {
            geometry_msgs::msg::Point value;
            value.x = x;
            value.y = y;
            value.z = map_z_m_ + 0.035;
            return value;
        };
        marker.points = {
            point(owned_region_x_min_m_, owned_region_y_min_m_),
            point(owned_region_x_max_m_, owned_region_y_min_m_),
            point(owned_region_x_max_m_, owned_region_y_max_m_),
            point(owned_region_x_min_m_, owned_region_y_max_m_),
            point(owned_region_x_min_m_, owned_region_y_min_m_)};
        return marker;
    }

    visualization_msgs::msg::Marker make_scan_circle_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "planned_scan_circle", 0);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.045;
        marker.color.r = 1.0F;
        marker.color.g = 0.15F;
        marker.color.b = 0.85F;
        marker.color.a = 0.9F;
        for (int i = 0; i <= 72; ++i) {
            const double angle = 2.0 * M_PI * static_cast<double>(i) / 72.0;
            geometry_msgs::msg::Point point;
            point.x = scan_center_.x() + scan_radius_m_ * std::cos(angle);
            point.y = scan_center_.y() + scan_radius_m_ * std::sin(angle);
            point.z = map_z_m_ + 0.055;
            marker.points.push_back(point);
        }
        return marker;
    }

    visualization_msgs::msg::Marker make_trajectory_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "auv_trajectory", 0);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.055;
        marker.color.r = 1.0F;
        marker.color.g = 1.0F;
        marker.color.b = 1.0F;
        marker.color.a = 0.95F;
        for (const auto & position : trajectory_) {
            geometry_msgs::msg::Point point;
            point.x = position.x();
            point.y = position.y();
            point.z = map_z_m_ + 0.075;
            marker.points.push_back(point);
        }
        return marker;
    }

    visualization_msgs::msg::Marker make_peak_snr_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "peak_snr_position", 0);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = peak_snr_position_.x();
        marker.pose.position.y = peak_snr_position_.y();
        marker.pose.position.z = map_z_m_ + 0.12;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.24;
        marker.scale.y = 0.24;
        marker.scale.z = 0.18;
        marker.color.r = 1.0F;
        marker.color.g = 0.05F;
        marker.color.b = 0.05F;
        marker.color.a = 1.0F;
        return marker;
    }

    visualization_msgs::msg::Marker make_status_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "homing_status", 0);
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = arena_offset_x_m_ + 2.4;
        marker.pose.position.y = arena_start_corner_ == "bottom_left" ?
            arena_offset_y_m_ - 0.7 : arena_offset_y_m_ + 0.7;
        marker.pose.position.z = map_z_m_ + 0.30;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.13;
        marker.color.r = 1.0F;
        marker.color.g = 1.0F;
        marker.color.b = 1.0F;
        marker.color.a = 1.0F;
        char text[192];
        if (have_latest_snr_) {
            std::snprintf(
                text, sizeof(text), "%s | SNR %.1f dB | peak %.1f dB",
                control_state_.c_str(), latest_snr_db_, peak_snr_db_);
        } else {
            std::snprintf(
                text, sizeof(text), "%s | waiting for SNR",
                control_state_.c_str());
        }
        marker.text = text;
        return marker;
    }

    visualization_msgs::msg::Marker make_vision_zone_marker(
        const rclcpp::Time & stamp) const
    {
        const double safe_x_min =
            arena_offset_x_m_ + arena_safety_margin_m_;
        const double safe_x_max =
            arena_offset_x_m_ + arena_length_m_ - arena_safety_margin_m_;
        const double safe_y_min = arena_start_corner_ == "bottom_left" ?
            arena_offset_y_m_ - arena_width_m_ + arena_safety_margin_m_ :
            arena_offset_y_m_ + arena_safety_margin_m_;
        const double safe_y_max = arena_start_corner_ == "bottom_left" ?
            arena_offset_y_m_ - arena_safety_margin_m_ :
            arena_offset_y_m_ + arena_width_m_ - arena_safety_margin_m_;
        const double width = std::min(
            vision_near_zone_width_m_, safe_y_max - safe_y_min);
        const double zone_y = arena_start_corner_ == "bottom_left" ?
            safe_y_min + 0.5 * width : safe_y_max - 0.5 * width;

        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "vision_near_zone", 0);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.5 * (safe_x_min + safe_x_max);
        marker.pose.position.y = zone_y;
        marker.pose.position.z = map_z_m_ - 0.03;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = safe_x_max - safe_x_min;
        marker.scale.y = width;
        marker.scale.z = 0.01;
        marker.color.r = 0.65F;
        marker.color.g = 0.15F;
        marker.color.b = 1.0F;
        marker.color.a = 0.20F;
        return marker;
    }

    visualization_msgs::msg::Marker make_vision_zone_label_marker(
        const rclcpp::Time & stamp) const
    {
        const double safe_x_min =
            arena_offset_x_m_ + arena_safety_margin_m_;
        const double safe_x_max =
            arena_offset_x_m_ + arena_length_m_ - arena_safety_margin_m_;
        const double safe_y_min = arena_start_corner_ == "bottom_left" ?
            arena_offset_y_m_ - arena_width_m_ + arena_safety_margin_m_ :
            arena_offset_y_m_ + arena_safety_margin_m_;
        const double safe_y_max = arena_start_corner_ == "bottom_left" ?
            arena_offset_y_m_ - arena_safety_margin_m_ :
            arena_offset_y_m_ + arena_width_m_ - arena_safety_margin_m_;
        const double width = std::min(
            vision_near_zone_width_m_, safe_y_max - safe_y_min);
        const double zone_y = arena_start_corner_ == "bottom_left" ?
            safe_y_min + 0.5 * width : safe_y_max - 0.5 * width;

        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "vision_near_zone_label", 0);
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.5 * (safe_x_min + safe_x_max);
        marker.pose.position.y = zone_y;
        marker.pose.position.z = map_z_m_ + 0.18;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.16;
        marker.color.r = 0.8F;
        marker.color.g = 0.4F;
        marker.color.b = 1.0F;
        marker.color.a = 1.0F;
        marker.text = "Vision Near Zone";
        return marker;
    }

    visualization_msgs::msg::Marker make_scan_center_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "scan_center", 0);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = scan_center_.x();
        marker.pose.position.y = scan_center_.y();
        marker.pose.position.z = map_z_m_ + 0.10;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.18;
        marker.scale.y = 0.18;
        marker.scale.z = 0.18;
        marker.color.r = 1.0F;
        marker.color.g = 0.1F;
        marker.color.b = 0.8F;
        marker.color.a = 1.0F;
        return marker;
    }

    visualization_msgs::msg::Marker make_scan_center_label_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "scan_center_label", 0);
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = scan_center_.x();
        marker.pose.position.y = scan_center_.y();
        marker.pose.position.z = map_z_m_ + 0.32;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.15;
        marker.color.r = 1.0F;
        marker.color.g = 0.1F;
        marker.color.b = 0.8F;
        marker.color.a = 1.0F;
        marker.text = "Scan Center";
        return marker;
    }

    visualization_msgs::msg::Marker make_snr_map_marker(const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "snr_map", 0);
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.95 * map_cell_size_m_;
        marker.scale.y = 0.95 * map_cell_size_m_;
        marker.scale.z = map_cell_height_m_;

        std::vector<double> cell_values;
        cell_values.reserve(grid_.size());
        for (const auto & entry : grid_) {
            if (!entry.second.values.empty()) {
                cell_values.push_back(median(entry.second.values));
            }
        }
        double color_min = snr_color_min_db_;
        double color_max = snr_color_max_db_;
        if (dynamic_snr_range_ && !cell_values.empty()) {
            std::sort(cell_values.begin(), cell_values.end());
            const std::size_t low_index = (cell_values.size() - 1) / 20;
            const std::size_t high_index = 19 * (cell_values.size() - 1) / 20;
            color_min = cell_values[low_index];
            color_max = cell_values[high_index];
            if (color_max - color_min < 1.0) {
                color_max = color_min + 1.0;
            }
        }

        marker.points.reserve(grid_.size());
        marker.colors.reserve(grid_.size());
        for (const auto & entry : grid_) {
            if (entry.second.values.empty()) {
                continue;
            }
            geometry_msgs::msg::Point point;
            point.x = (static_cast<double>(entry.first.first) + 0.5) * map_cell_size_m_;
            point.y = (static_cast<double>(entry.first.second) + 0.5) * map_cell_size_m_;
            point.z = map_z_m_;
            marker.points.push_back(point);
            const double normalized = std::clamp(
                (median(entry.second.values) - color_min) / (color_max - color_min),
                0.0, 1.0);
            marker.colors.push_back(snr_color(normalized));
        }
        return marker;
    }

    visualization_msgs::msg::Marker make_auv_marker(const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "auv", 0);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = current_position_.x();
        marker.pose.position.y = current_position_.y();
        marker.pose.position.z = map_z_m_ + 0.12;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.22;
        marker.scale.y = 0.22;
        marker.scale.z = 0.12;
        marker.color.r = 1.0F;
        marker.color.g = 1.0F;
        marker.color.b = 1.0F;
        marker.color.a = 1.0F;
        return marker;
    }

    visualization_msgs::msg::Marker make_waypoint_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "current_waypoint", 0);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = current_waypoint_.x();
        marker.pose.position.y = current_waypoint_.y();
        marker.pose.position.z = map_z_m_ + 0.12;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.20;
        marker.scale.y = 0.20;
        marker.scale.z = 0.20;
        marker.color.r = 1.0F;
        marker.color.g = 0.85F;
        marker.color.b = 0.0F;
        marker.color.a = 1.0F;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        return marker;
    }

    visualization_msgs::msg::Marker make_waypoint_line_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "current_waypoint_line", 0);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point auv;
        auv.x = current_position_.x();
        auv.y = current_position_.y();
        auv.z = map_z_m_ + 0.08;
        geometry_msgs::msg::Point waypoint = auv;
        waypoint.x = current_waypoint_.x();
        waypoint.y = current_waypoint_.y();
        marker.points = {auv, waypoint};
        marker.scale.x = 0.025;
        marker.color.r = 1.0F;
        marker.color.g = 0.85F;
        marker.color.b = 0.0F;
        marker.color.a = 0.8F;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        return marker;
    }

    visualization_msgs::msg::Marker make_waypoint_label_marker(
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, "current_waypoint_label", 0);
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = current_waypoint_.x();
        marker.pose.position.y = current_waypoint_.y();
        marker.pose.position.z = map_z_m_ + 0.34;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.16;
        marker.color.r = 1.0F;
        marker.color.g = 0.85F;
        marker.color.b = 0.0F;
        marker.color.a = 1.0F;
        marker.text = "Waypoint";
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        return marker;
    }

    visualization_msgs::msg::Marker make_arrow_marker(
        const rclcpp::Time & stamp,
        const std::string & name,
        const int id,
        const Eigen::Vector2d & direction,
        const float red,
        const float green,
        const float blue) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, name, id);
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point start;
        start.x = current_position_.x();
        start.y = current_position_.y();
        start.z = map_z_m_ + 0.15;
        geometry_msgs::msg::Point end = start;
        end.x += arrow_length_m_ * direction.x();
        end.y += arrow_length_m_ * direction.y();
        marker.points = {start, end};
        marker.scale.x = 0.07;
        marker.scale.y = 0.16;
        marker.scale.z = 0.20;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 1.0F;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        return marker;
    }

    visualization_msgs::msg::Marker make_text_marker(
        const rclcpp::Time & stamp,
        const std::string & name,
        const int id,
        const Eigen::Vector2d & direction,
        const std::string & text,
        const float red,
        const float green,
        const float blue) const
    {
        visualization_msgs::msg::Marker marker;
        set_marker_header(marker, stamp, name, id);
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = current_position_.x() + arrow_length_m_ * direction.x();
        marker.pose.position.y = current_position_.y() + arrow_length_m_ * direction.y();
        marker.pose.position.z = map_z_m_ + 0.38;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.18;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 1.0F;
        marker.text = text;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        return marker;
    }

    void set_marker_header(
        visualization_msgs::msg::Marker & marker,
        const rclcpp::Time & stamp,
        const std::string & name,
        const int id) const
    {
        marker.header.stamp = stamp;
        marker.header.frame_id = arena_frame_id_;
        marker.ns = name;
        marker.id = id;
    }

    static double median(const std::deque<double> & values)
    {
        std::vector<double> sorted(values.begin(), values.end());
        const std::size_t middle = sorted.size() / 2;
        std::nth_element(sorted.begin(), sorted.begin() + middle, sorted.end());
        if (sorted.size() % 2 != 0) {
            return sorted[middle];
        }
        const double upper = sorted[middle];
        std::nth_element(sorted.begin(), sorted.begin() + middle - 1, sorted.end());
        return 0.5 * (sorted[middle - 1] + upper);
    }

    static std_msgs::msg::ColorRGBA snr_color(const double normalized)
    {
        const double hue = (1.0 - normalized) * 240.0;
        const double sector = hue / 60.0;
        const double chroma = 1.0;
        const double x = chroma * (1.0 - std::abs(std::fmod(sector, 2.0) - 1.0));
        double red = 0.0;
        double green = 0.0;
        double blue = 0.0;
        if (sector < 1.0) {
            red = chroma;
            green = x;
        } else if (sector < 2.0) {
            red = x;
            green = chroma;
        } else if (sector < 3.0) {
            green = chroma;
            blue = x;
        } else {
            green = x;
            blue = chroma;
        }
        std_msgs::msg::ColorRGBA color;
        color.r = static_cast<float>(red);
        color.g = static_cast<float>(green);
        color.b = static_cast<float>(blue);
        color.a = 0.88F;
        return color;
    }

    double map_cell_size_m_ = 0.15;
    double map_cell_height_m_ = 0.04;
    double max_odometry_extrapolation_s_ = 0.15;
    double snr_color_min_db_ = -10.0;
    double snr_color_max_db_ = 20.0;
    double arrow_length_m_ = 1.0;
    double map_z_m_ = 0.0;
    double arena_length_m_ = 15.0;
    double arena_width_m_ = 16.0;
    double arena_offset_x_m_ = 0.0;
    double arena_offset_y_m_ = 0.0;
    double arena_safety_margin_m_ = 0.5;
    double vision_near_zone_width_m_ = 2.0;
    double scan_radius_m_ = 1.30;
    double owned_region_x_min_m_ = 0.0;
    double owned_region_x_max_m_ = 0.0;
    double owned_region_y_min_m_ = 0.0;
    double owned_region_y_max_m_ = 0.0;
    double trajectory_spacing_m_ = 0.04;
    double latest_snr_db_ = 0.0;
    double peak_snr_db_ = -240.0;
    std::size_t max_cell_values_ = 31;
    std::size_t max_trajectory_points_ = 5000;
    bool dynamic_snr_range_ = true;
    bool owned_region_enabled_ = false;
    bool have_odometry_ = false;
    bool have_region_gradient_ = false;
    bool have_waypoint_ = false;
    bool have_scan_center_ = false;
    bool have_latest_snr_ = false;
    bool have_peak_snr_ = false;
    std::string arena_frame_id_ = "arena";
    std::string vehicle_follow_frame_id_ = "hydrophone_vehicle_follow";
    std::string arena_start_corner_ = "bottom_left";
    std::string control_state_ = "WAITING_FOR_START_FRAME";
    Eigen::Vector2d current_position_{0.0, 0.0};
    Eigen::Vector2d current_waypoint_{0.0, 0.0};
    Eigen::Vector2d scan_center_{0.0, 0.0};
    Eigen::Vector2d peak_snr_position_{0.0, 0.0};
    Eigen::Vector2d region_gradient_{1.0, 0.0};
    Eigen::Vector2d rolling_gradient_{1.0, 0.0};
    Eigen::Vector2d homing_direction_{1.0, 0.0};
    bool have_rolling_gradient_ = false;
    bool have_homing_direction_ = false;
    std::deque<PoseSample> odometry_history_;
    std::deque<Eigen::Vector2d> trajectory_;
    std::deque<PendingSnr> pending_snr_;
    std::map<std::pair<int, int>, GridCell> grid_;
    hydrophone_ctrl::ArenaFrameTransform arena_transform_;

    rclcpp::Subscription<audio_common_msgs::msg::Float64Stamped>::SharedPtr snr_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        start_frame_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        region_gradient_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        rolling_gradient_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        homing_direction_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr scan_center_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(
    audio_capture::RegionLocalGradientRvizVisualizerNode)
