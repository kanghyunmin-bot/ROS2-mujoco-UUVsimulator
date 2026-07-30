#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <Eigen/Dense>
#include "hydrophone_ctrl/arena_frame_transform.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace audio_capture
{
// Vision near zone의 중앙으로 이동한 뒤 zone의 긴 축을 따라 SNR peak를 통과할
// 때까지 주행하고, 기록한 peak 위치로 복귀해 Vision에 제어권을 넘긴다.
class NearZoneLineSearchControllerNode : public rclcpp::Node
{
public:
    explicit NearZoneLineSearchControllerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("near_zone_line_search_controller", options)
    {
        const auto odometry_topic = declare_parameter<std::string>(
            "odometry_topic", "/odometry/filtered");
        const auto start_frame_topic = declare_parameter<std::string>(
            "start_frame_topic", "/start_frame");
        const auto snr_topic = declare_parameter<std::string>(
            "snr_topic", "/audio_frequency_detector/snr_db_stamped");
        const auto state_topic = declare_parameter<std::string>(
            "state_topic", "/homing/control_state");
        const auto waypoint_topic = declare_parameter<std::string>(
            "waypoint_topic", "/homing/current_waypoint");
        arena_frame_id_ = declare_parameter<std::string>(
            "arena_frame_id", "arena");
        const auto peak_topic = declare_parameter<std::string>(
            "peak_topic", "/homing/snr_peak_position");
        const auto vision_search_request_topic = declare_parameter<std::string>(
            "vision_search_request_topic", "/homing/vision_search_active");
        const auto target_confirmed_topic = declare_parameter<std::string>(
            "target_confirmed_topic", "/vision/target_confirmed");
        const auto vision_control_granted_topic = declare_parameter<std::string>(
            "vision_control_granted_topic", "/homing/vision_control_granted");
        const auto rc_override_topic = declare_parameter<std::string>(
            "rc_override_topic", "/mavros/rc/override");
        const auto emergency_stop_topic = declare_parameter<std::string>(
            "emergency_stop_topic", "/mission/emergency_stop");
        const bool enable_keyboard_emergency_stop = declare_parameter<bool>(
            "enable_keyboard_emergency_stop", true);
        emergency_stop_key_ = declare_parameter<std::string>(
            "emergency_stop_key", "s");

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
        // 0 이하면 Acoustic 타임아웃 핸드오프를 끈다.
        // line_search는 zone 안에서 동작하므로 zone 진입이 아니라 peak/timeout에서 인계한다.
        acoustic_timeout_s_ = declare_parameter<double>("acoustic_timeout_s", 90.0);
        if (!std::isfinite(acoustic_timeout_s_)) {
            throw std::invalid_argument("acoustic_timeout_s must be finite");
        }
        arena_start_corner_ = declare_parameter<std::string>(
            "arena_start_corner", "bottom_left");
        if (arena_start_corner_ != "bottom_left" &&
            arena_start_corner_ != "bottom_right")
        {
            throw std::invalid_argument(
                "arena_start_corner must be bottom_left or bottom_right");
        }
        line_search_direction_ = static_cast<int>(std::clamp<std::int64_t>(
            declare_parameter<std::int64_t>("line_search_direction", 1), -1, 1));
        if (line_search_direction_ == 0) {
            throw std::invalid_argument("line_search_direction must be -1 or 1");
        }

        target_depth_z_m_ = declare_parameter<double>("target_depth_z_m", -8.0);
        line_search_start_depth_tolerance_m_ = std::max(
            0.0, declare_parameter<double>(
                "line_search_start_depth_tolerance_m", 0.2));
        waypoint_reach_tolerance_m_ = std::max(
            0.01, declare_parameter<double>("waypoint_reach_tolerance_m", 0.15));
        snr_sample_spacing_m_ = std::max(
            0.01, declare_parameter<double>("snr_sample_spacing_m", 0.15));
        snr_drop_from_peak_db_ = std::max(
            0.0, declare_parameter<double>("snr_drop_from_peak_db", 2.0));
        snr_decline_count_limit_ =
            static_cast<std::size_t>(std::max<std::int64_t>(
                1, declare_parameter<std::int64_t>(
                    "snr_decline_count_limit", 5)));
        snr_median_window_size_ =
            static_cast<std::size_t>(std::max<std::int64_t>(
                1, declare_parameter<std::int64_t>(
                    "snr_median_window_size", 3)));
        if (snr_median_window_size_ % 2 == 0) {
            throw std::invalid_argument("snr_median_window_size must be odd");
        }
        snr_timeout_s_ = std::max(
            0.05, declare_parameter<double>("snr_timeout_s", 1.0));
        max_snr_odom_skew_s_ = std::max(
            0.0, declare_parameter<double>("max_snr_odom_skew_s", 0.15));
        odometry_timeout_s_ = std::max(
            0.05, declare_parameter<double>("odometry_timeout_s", 0.5));
        rate_hz_ = std::clamp(
            declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
        forward_cruise_ = std::clamp(
            declare_parameter<double>("forward_cruise", 0.5), 0.0, 1.0);
        yaw_kp_ = std::max(
            0.0, declare_parameter<double>("yaw_kp", 1.15));
        yaw_ki_ = std::max(
            0.0, declare_parameter<double>("yaw_ki", 0.15));
        yaw_kd_ = std::max(
            0.0, declare_parameter<double>("yaw_kd", 0.08));
        yaw_integral_limit_ = std::max(
            0.0, declare_parameter<double>("yaw_integral_limit", 2.0));
        yaw_limit_ = std::clamp(
            declare_parameter<double>("yaw_limit", 0.72), 0.0, 1.0);
        move_heading_tolerance_rad_ = std::clamp(
            declare_parameter<double>("move_heading_tolerance_rad", 0.1745),
            0.01, PI);
        rc_pwm_span_ = std::clamp(
            declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);
        invert_rc_yaw_ = declare_parameter<bool>("invert_rc_yaw", true);

        const ArenaBounds safe_bounds = arena_bounds(arena_safety_margin_m_);
        if (safe_bounds.x_min >= safe_bounds.x_max ||
            safe_bounds.y_min >= safe_bounds.y_max)
        {
            throw std::invalid_argument("arena_safety_margin_m leaves no safe arena");
        }

        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 30,
            std::bind(&NearZoneLineSearchControllerNode::odometry_callback, this,
                std::placeholders::_1));
        start_frame_sub_ =
            create_subscription<geometry_msgs::msg::PoseStamped>(
                start_frame_topic,
                rclcpp::QoS(1).reliable().transient_local(),
                std::bind(
                    &NearZoneLineSearchControllerNode::start_frame_callback,
                    this, std::placeholders::_1));
        snr_sub_ =
            create_subscription<audio_common_msgs::msg::Float64Stamped>(
                snr_topic, 20,
                std::bind(&NearZoneLineSearchControllerNode::snr_callback, this,
                    std::placeholders::_1));
        target_confirmed_sub_ = create_subscription<std_msgs::msg::Bool>(
            target_confirmed_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(
                &NearZoneLineSearchControllerNode::target_confirmed_callback,
                this, std::placeholders::_1));
        emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            emergency_stop_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(
                &NearZoneLineSearchControllerNode::emergency_stop_callback,
                this, std::placeholders::_1));

        state_pub_ = create_publisher<std_msgs::msg::String>(
            state_topic, rclcpp::QoS(1).reliable().transient_local());
        waypoint_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            waypoint_topic, rclcpp::QoS(1).reliable().transient_local());
        peak_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            peak_topic, rclcpp::QoS(1).reliable().transient_local());
        vision_search_request_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_search_request_topic,
            rclcpp::QoS(1).reliable().transient_local());
        vision_control_granted_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_control_granted_topic,
            rclcpp::QoS(1).reliable().transient_local());
        rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(
            rc_override_topic, 10);
        emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
            emergency_stop_topic,
            rclcpp::QoS(1).reliable().transient_local());

        publish_state();
        publish_vision_search_request(false);
        publish_vision_control_granted(false);
        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&NearZoneLineSearchControllerNode::control_loop, this));

        if (enable_keyboard_emergency_stop && !emergency_stop_key_.empty()) {
            keyboard_thread_ = std::thread(
                &NearZoneLineSearchControllerNode::keyboard_loop, this);
        }
        RCLCPP_INFO(
            get_logger(),
            "Near-zone line search ready: drop=%.1f dB count=%zu spacing=%.2f m "
            "acoustic_timeout_s=%.1f (search request in zone; peak/confirm/timeout -> grant)",
            snr_drop_from_peak_db_, snr_decline_count_limit_,
            snr_sample_spacing_m_, acoustic_timeout_s_);
    }

    ~NearZoneLineSearchControllerNode() override
    {
        stop_keyboard_thread_.store(true);
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr std::uint16_t RC_NEUTRAL = 1500;
    static constexpr std::size_t VERTICAL_CHANNEL_INDEX = 2;
    static constexpr std::size_t YAW_CHANNEL_INDEX = 3;
    static constexpr std::size_t FORWARD_CHANNEL_INDEX = 4;
    static constexpr std::size_t LATERAL_CHANNEL_INDEX = 5;
    static constexpr double DEPTH_KP = 0.8;
    static constexpr double DEPTH_KI = 0.15;
    static constexpr double DEPTH_INTEGRAL_LIMIT = 2.0;
    static constexpr double HEAVE_LIMIT = 0.2;
    static constexpr double YAW_DERIVATIVE_ALPHA = 0.2;
    static constexpr double REALIGN_HEADING_ERROR_RAD = PI / 3.0;

    enum class State
    {
        MOVE_TO_LINE_CENTER,
        LINE_SEARCH,
        RETURN_TO_PEAK,
        HANDOFF_NEUTRAL,
        HANDOFF_COMPLETE
    };

    struct ArenaBounds
    {
        double x_min = 0.0;
        double x_max = 0.0;
        double y_min = 0.0;
        double y_max = 0.0;
    };

    struct Command
    {
        double forward = 0.0;
        double yaw = 0.0;
        double heave = 0.0;
    };

    struct SnrSample
    {
        Eigen::Vector2d position{0.0, 0.0};
        double snr_db = 0.0;
    };

    struct PoseSample
    {
        rclcpp::Time stamp;
        Eigen::Vector2d position{0.0, 0.0};
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
        const double z_m = msg->pose.pose.position.z;
        const double odom_yaw = hydrophone_ctrl::ArenaFrameTransform::
            yaw_from_quaternion(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        if (!odom_position.allFinite() || !std::isfinite(z_m) ||
            !std::isfinite(odom_yaw))
        {
            return;
        }
        if (!arena_transform_.initialized()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Waiting for external guided start frame");
            return;
        }
        const bool first_odometry = !have_odometry_;
        current_position_ =
            arena_transform_.position_from_odom(odom_position);
        current_z_m_ = z_m;
        current_yaw_rad_ = arena_transform_.yaw_from_odom(odom_yaw);
        last_odometry_receive_time_ = now();
        have_odometry_ = true;
        const rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() > 0) {
            if (!odometry_history_.empty() &&
                stamp < odometry_history_.back().stamp)
            {
                odometry_history_.clear();
            }
            odometry_history_.push_back({stamp, current_position_});
            while (odometry_history_.size() > 300) {
                odometry_history_.pop_front();
            }
        }

        if (first_odometry) {
            initial_z_m_ = current_z_m_;
            start_acoustic_deadline_if_needed(now());
            line_center_ = nearest_near_zone_center(current_position_);
            set_current_waypoint(line_center_);
            publish_state();
            RCLCPP_INFO(
                get_logger(),
                "[LINE] moving to near-zone center=(%.2f, %.2f), depth=%.2f m",
                line_center_.x(), line_center_.y(), target_depth_z_m_);
        }
    }

    void snr_callback(
        const audio_common_msgs::msg::Float64Stamped::ConstSharedPtr msg)
    {
        if (state_ != State::LINE_SEARCH || !std::isfinite(msg->data) ||
            !have_odometry_)
        {
            return;
        }
        Eigen::Vector2d sample_position;
        const rclcpp::Time sample_stamp(msg->header.stamp);
        if (sample_stamp.nanoseconds() <= 0 ||
            !position_at(sample_stamp, sample_position))
        {
            return;
        }
        last_snr_receive_time_ = now();
        have_snr_ = true;
        if (last_sample_position_ &&
            (sample_position - *last_sample_position_).norm() <
            snr_sample_spacing_m_)
        {
            return;
        }
        last_sample_position_ = sample_position;
        snr_filter_window_.push_back({sample_position, msg->data});
        while (snr_filter_window_.size() > snr_median_window_size_) {
            snr_filter_window_.pop_front();
        }
        if (snr_filter_window_.size() < snr_median_window_size_) {
            return;
        }

        std::vector<double> values;
        values.reserve(snr_filter_window_.size());
        for (const auto & sample : snr_filter_window_) {
            values.push_back(sample.snr_db);
        }
        const auto middle = values.begin() +
            static_cast<std::ptrdiff_t>(values.size() / 2);
        std::nth_element(values.begin(), middle, values.end());
        const double filtered_snr = *middle;
        const Eigen::Vector2d filtered_position =
            snr_filter_window_[snr_filter_window_.size() / 2].position;

        if (!have_peak_ || filtered_snr > peak_snr_db_) {
            have_peak_ = true;
            peak_snr_db_ = filtered_snr;
            peak_position_ = filtered_position;
            decline_count_ = 0;
            publish_peak_position();
            RCLCPP_INFO(
                get_logger(),
                "[SNR] new peak=%.1f dB position=(%.2f, %.2f)",
                peak_snr_db_, peak_position_.x(), peak_position_.y());
            return;
        }

        if (peak_snr_db_ - filtered_snr >= snr_drop_from_peak_db_) {
            ++decline_count_;
        } else {
            decline_count_ = 0;
        }
        RCLCPP_INFO(
            get_logger(),
            "[SNR] filtered=%.1f dB peak=%.1f dB decline=%zu/%zu",
            filtered_snr, peak_snr_db_,
            decline_count_, snr_decline_count_limit_);
        if (decline_count_ >= snr_decline_count_limit_) {
            return_to_peak_requested_ = true;
        }
    }

    void target_confirmed_callback(
        const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (!msg->data || !vision_search_requested_ || handoff_in_progress()) {
            return;
        }
        begin_vision_handoff("target_confirmed");
    }

    bool handoff_in_progress() const
    {
        return state_ == State::HANDOFF_NEUTRAL ||
            state_ == State::HANDOFF_COMPLETE;
    }

    void start_acoustic_deadline_if_needed(const rclcpp::Time & current_time)
    {
        if (acoustic_deadline_started_ || acoustic_timeout_s_ <= 0.0) {
            return;
        }
        acoustic_deadline_started_ = true;
        acoustic_start_time_ = current_time;
        RCLCPP_INFO(
            get_logger(),
            "[VISION] acoustic deadline started (%.1f s)",
            acoustic_timeout_s_);
    }

    bool acoustic_deadline_expired(const rclcpp::Time & current_time) const
    {
        return acoustic_deadline_started_ &&
            acoustic_timeout_s_ > 0.0 &&
            (current_time - acoustic_start_time_).seconds() >= acoustic_timeout_s_;
    }

    // line-search는 이미 near zone에서 동작하므로 탐색 요청만 먼저 보낸다.
    void request_vision_confirmation()
    {
        if (vision_search_requested_ || handoff_in_progress()) {
            return;
        }
        vision_search_requested_ = true;
        publish_vision_control_granted(false);
        publish_vision_search_request(true);
        RCLCPP_INFO(
            get_logger(),
            "[VISION] search requested at position=(%.2f, %.2f); acoustic control kept",
            current_position_.x(), current_position_.y());
    }

    // peak 복귀 / confirm / timeout 시 중립 후 Vision에 제어권을 넘긴다.
    void begin_vision_handoff(const char * reason)
    {
        if (handoff_in_progress()) {
            return;
        }
        if (!vision_search_requested_) {
            request_vision_confirmation();
        }
        handoff_neutral_sent_ = false;
        reset_motion_controllers();
        publish_rc(Command{});
        transition_to(State::HANDOFF_NEUTRAL);
        RCLCPP_INFO(
            get_logger(),
            "[VISION] acoustic handoff started reason=%s position=(%.2f, %.2f)",
            reason, current_position_.x(), current_position_.y());
    }

    void emergency_stop_callback(
        const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (msg->data) {
            emergency_stop_active_.store(true);
        }
    }

    void control_loop()
    {
        const rclcpp::Time current_time = now();
        if (emergency_stop_active_.load()) {
            publish_rc(Command{});
            return;
        }
        if (state_ == State::HANDOFF_NEUTRAL) {
            if (!handoff_neutral_sent_) {
                publish_rc(Command{});
                handoff_neutral_sent_ = true;
                handoff_neutral_time_ = current_time;
                return;
            }
            if ((current_time - handoff_neutral_time_).seconds() >=
                1.0 / rate_hz_)
            {
                transition_to(State::HANDOFF_COMPLETE);
                publish_vision_control_granted(true);
                RCLCPP_INFO(
                    get_logger(),
                    "[VISION] acoustic RC stopped; vision control granted");
            }
            return;
        }
        if (state_ == State::HANDOFF_COMPLETE) {
            return;
        }
        if (!odometry_is_fresh(current_time)) {
            reset_motion_controllers();
            publish_rc(Command{});
            return;
        }
        log_status();

        if (acoustic_deadline_expired(current_time)) {
            begin_vision_handoff("acoustic_timeout");
            return;
        }

        switch (state_) {
            case State::MOVE_TO_LINE_CENTER:
                if (follow_waypoint(current_time) &&
                    depth_ready_for_line_search())
                {
                    begin_line_search();
                }
                return;
            case State::LINE_SEARCH:
                if (return_to_peak_requested_) {
                    begin_return_to_peak("sustained_snr_decline");
                    return;
                }
                if (!snr_is_fresh(current_time)) {
                    publish_rc(depth_hold_command(current_time));
                    return;
                }
                if (follow_waypoint(current_time)) {
                    if (have_peak_) {
                        begin_return_to_peak("line_boundary");
                    } else {
                        begin_vision_handoff("line_boundary");
                    }
                }
                return;
            case State::RETURN_TO_PEAK:
                if (follow_waypoint(current_time)) {
                    begin_vision_handoff("snr_peak");
                }
                return;
            case State::HANDOFF_NEUTRAL:
            case State::HANDOFF_COMPLETE:
                return;
        }
    }

    void begin_line_search()
    {
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        line_endpoint_ = line_center_;
        line_endpoint_.x() =
            line_search_direction_ > 0 ? bounds.x_max : bounds.x_min;
        have_snr_ = false;
        have_peak_ = false;
        return_to_peak_requested_ = false;
        decline_count_ = 0;
        last_sample_position_.reset();
        snr_filter_window_.clear();
        request_vision_confirmation();
        transition_to(State::LINE_SEARCH);
        set_current_waypoint(line_endpoint_);
        RCLCPP_INFO(
            get_logger(),
            "[LINE] search started toward %sX endpoint=(%.2f, %.2f)",
            line_search_direction_ > 0 ? "+" : "-",
            line_endpoint_.x(), line_endpoint_.y());
    }

    void begin_return_to_peak(const char * reason)
    {
        if (!have_peak_) {
            publish_rc(depth_hold_command(now()));
            RCLCPP_ERROR(
                get_logger(), "[LINE] cannot return: no valid SNR peak");
            return;
        }
        return_to_peak_requested_ = false;
        transition_to(State::RETURN_TO_PEAK);
        set_current_waypoint(peak_position_);
        RCLCPP_INFO(
            get_logger(),
            "[LINE] return reason=%s peak=%.1f dB position=(%.2f, %.2f)",
            reason, peak_snr_db_, peak_position_.x(), peak_position_.y());
    }

    bool follow_waypoint(const rclcpp::Time & current_time)
    {
        const Eigen::Vector2d delta = current_waypoint_ - current_position_;
        if (delta.norm() <= waypoint_reach_tolerance_m_) {
            publish_rc(depth_hold_command(current_time));
            return true;
        }
        const double desired_yaw = std::atan2(delta.y(), delta.x());
        const double yaw_error = wrap_pi(desired_yaw - current_yaw_rad_);
        Command command = depth_hold_command(current_time);
        command.yaw = yaw_pid_command(yaw_error, current_time);
        if (!heading_aligned_ &&
            std::abs(yaw_error) <= move_heading_tolerance_rad_)
        {
            heading_aligned_ = true;
        }
        if (heading_aligned_ &&
            std::abs(yaw_error) >= REALIGN_HEADING_ERROR_RAD)
        {
            heading_aligned_ = false;
        }
        if (heading_aligned_) {
            command.forward = forward_cruise_;
        }
        publish_rc(command);
        return false;
    }

    Command depth_hold_command(const rclcpp::Time & current_time)
    {
        const double error = target_depth_z_m_ - current_z_m_;
        double dt = 0.0;
        if (depth_pid_initialized_) {
            dt = std::clamp(
                (current_time - last_depth_control_time_).seconds(), 0.0, 0.2);
        }
        last_depth_control_time_ = current_time;
        depth_pid_initialized_ = true;
        const double candidate_integral = std::clamp(
            depth_error_integral_ + error * dt,
            -DEPTH_INTEGRAL_LIMIT, DEPTH_INTEGRAL_LIMIT);
        const double candidate_heave =
            -(DEPTH_KP * error + DEPTH_KI * candidate_integral);
        if (std::abs(candidate_heave) <= HEAVE_LIMIT ||
            candidate_heave * error > 0.0)
        {
            depth_error_integral_ = candidate_integral;
        }
        Command command;
        command.heave = std::clamp(
            -(DEPTH_KP * error + DEPTH_KI * depth_error_integral_),
            -HEAVE_LIMIT, HEAVE_LIMIT);
        return command;
    }

    double yaw_pid_command(
        const double yaw_error, const rclcpp::Time & current_time)
    {
        double dt = 0.0;
        if (yaw_pid_initialized_) {
            dt = std::clamp(
                (current_time - last_yaw_control_time_).seconds(), 0.0, 0.2);
        }
        last_yaw_control_time_ = current_time;
        if (yaw_pid_initialized_ && dt > 1.0e-6) {
            const double raw_derivative =
                wrap_pi(yaw_error - previous_yaw_error_) / dt;
            yaw_error_derivative_ =
                (1.0 - YAW_DERIVATIVE_ALPHA) * yaw_error_derivative_ +
                YAW_DERIVATIVE_ALPHA * raw_derivative;
        }
        previous_yaw_error_ = yaw_error;
        yaw_pid_initialized_ = true;
        const double candidate_integral = std::clamp(
            yaw_error_integral_ + yaw_error * dt,
            -yaw_integral_limit_, yaw_integral_limit_);
        const double candidate_command =
            yaw_kp_ * yaw_error + yaw_ki_ * candidate_integral +
            yaw_kd_ * yaw_error_derivative_;
        if (std::abs(candidate_command) <= yaw_limit_ ||
            candidate_command * yaw_error < 0.0)
        {
            yaw_error_integral_ = candidate_integral;
        }
        return std::clamp(
            yaw_kp_ * yaw_error + yaw_ki_ * yaw_error_integral_ +
            yaw_kd_ * yaw_error_derivative_,
            -yaw_limit_, yaw_limit_);
    }

    void reset_motion_controllers()
    {
        heading_aligned_ = false;
        yaw_pid_initialized_ = false;
        depth_pid_initialized_ = false;
        previous_yaw_error_ = 0.0;
        yaw_error_integral_ = 0.0;
        yaw_error_derivative_ = 0.0;
        depth_error_integral_ = 0.0;
    }

    ArenaBounds arena_bounds(const double inset) const
    {
        ArenaBounds bounds;
        bounds.x_min = arena_offset_x_m_ + inset;
        bounds.x_max = arena_offset_x_m_ + arena_length_m_ - inset;
        if (arena_start_corner_ == "bottom_left") {
            bounds.y_min = arena_offset_y_m_ - arena_width_m_ + inset;
            bounds.y_max = arena_offset_y_m_ - inset;
        } else {
            bounds.y_min = arena_offset_y_m_ + inset;
            bounds.y_max = arena_offset_y_m_ + arena_width_m_ - inset;
        }
        return bounds;
    }

    Eigen::Vector2d nearest_near_zone_center(
        const Eigen::Vector2d & position) const
    {
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        const double width = std::min(
            vision_near_zone_width_m_, bounds.y_max - bounds.y_min);
        const double y = arena_start_corner_ == "bottom_left" ?
            bounds.y_min + 0.5 * width :
            bounds.y_max - 0.5 * width;
        return {std::clamp(position.x(), bounds.x_min, bounds.x_max), y};
    }

    bool odometry_is_fresh(const rclcpp::Time & current_time) const
    {
        return have_odometry_ &&
            (current_time - last_odometry_receive_time_).seconds() <=
            odometry_timeout_s_;
    }

    bool position_at(
        const rclcpp::Time & stamp, Eigen::Vector2d & position) const
    {
        if (odometry_history_.empty()) {
            return false;
        }
        const auto upper = std::lower_bound(
            odometry_history_.begin(), odometry_history_.end(), stamp,
            [](const PoseSample & sample, const rclcpp::Time & time) {
                return sample.stamp < time;
            });
        if (upper == odometry_history_.begin()) {
            if (std::abs((upper->stamp - stamp).seconds()) >
                max_snr_odom_skew_s_)
            {
                return false;
            }
            position = upper->position;
            return true;
        }
        if (upper == odometry_history_.end()) {
            const auto & last = odometry_history_.back();
            if (std::abs((stamp - last.stamp).seconds()) >
                max_snr_odom_skew_s_)
            {
                return false;
            }
            position = last.position;
            return true;
        }
        const auto lower = std::prev(upper);
        const double interval = (upper->stamp - lower->stamp).seconds();
        if (interval <= 1.0e-9) {
            position = upper->position;
            return true;
        }
        const double alpha = std::clamp(
            (stamp - lower->stamp).seconds() / interval, 0.0, 1.0);
        position =
            (1.0 - alpha) * lower->position + alpha * upper->position;
        return true;
    }

    bool snr_is_fresh(const rclcpp::Time & current_time) const
    {
        return have_snr_ &&
            (current_time - last_snr_receive_time_).seconds() <= snr_timeout_s_;
    }

    bool depth_ready_for_line_search() const
    {
        if (!have_odometry_) {
            return false;
        }
        if (target_depth_z_m_ < initial_z_m_) {
            return current_z_m_ <=
                target_depth_z_m_ + line_search_start_depth_tolerance_m_;
        }
        return current_z_m_ >=
            target_depth_z_m_ - line_search_start_depth_tolerance_m_;
    }

    void log_status()
    {
        const double distance =
            (current_waypoint_ - current_position_).norm();
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[CONTROL] state=%s distance=%.2f m z=%.2f/%.2f m "
            "depth_ready=%s snr_fresh=%s",
            state_name(state_), distance, current_z_m_, target_depth_z_m_,
            depth_ready_for_line_search() ? "true" : "false",
            snr_is_fresh(now()) ? "true" : "false");
    }

    void set_current_waypoint(const Eigen::Vector2d & waypoint)
    {
        current_waypoint_ = waypoint;
        heading_aligned_ = false;
        yaw_pid_initialized_ = false;
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = arena_frame_id_;
        msg.point.x = waypoint.x();
        msg.point.y = waypoint.y();
        waypoint_pub_->publish(msg);
    }

    void publish_peak_position()
    {
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = arena_frame_id_;
        msg.point.x = peak_position_.x();
        msg.point.y = peak_position_.y();
        peak_pub_->publish(msg);
    }

    void transition_to(const State next_state)
    {
        if (state_ == next_state) {
            return;
        }
        const State previous = state_;
        state_ = next_state;
        publish_state();
        RCLCPP_INFO(
            get_logger(), "[STATE] %s -> %s",
            state_name(previous), state_name(state_));
    }

    void publish_state()
    {
        std_msgs::msg::String msg;
        msg.data = state_name(state_);
        state_pub_->publish(msg);
    }

    void publish_vision_search_request(const bool active)
    {
        std_msgs::msg::Bool msg;
        msg.data = active;
        vision_search_request_pub_->publish(msg);
    }

    void publish_vision_control_granted(const bool granted)
    {
        std_msgs::msg::Bool msg;
        msg.data = granted;
        vision_control_granted_pub_->publish(msg);
    }

    void publish_rc(const Command & command)
    {
        mavros_msgs::msg::OverrideRCIn msg;
        msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        msg.channels[VERTICAL_CHANNEL_INDEX] = axis_pwm(command.heave, true);
        msg.channels[YAW_CHANNEL_INDEX] = axis_pwm(command.yaw, invert_rc_yaw_);
        msg.channels[FORWARD_CHANNEL_INDEX] = axis_pwm(command.forward, false);
        msg.channels[LATERAL_CHANNEL_INDEX] = RC_NEUTRAL;
        rc_pub_->publish(msg);
    }

    void trigger_emergency_stop()
    {
        if (emergency_stop_active_.exchange(true)) {
            return;
        }
        publish_rc(Command{});
        std_msgs::msg::Bool msg;
        msg.data = true;
        emergency_stop_pub_->publish(msg);
        RCLCPP_ERROR(
            get_logger(), "[EMERGENCY] key '%c' pressed; neutral RC latched",
            emergency_stop_key_.front());
    }

    void keyboard_loop()
    {
        const int terminal_fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
        if (terminal_fd < 0) {
            RCLCPP_WARN(
                get_logger(),
                "[EMERGENCY] keyboard disabled: cannot open controlling terminal");
            return;
        }
        termios original_termios;
        bool restore_terminal = false;
        if (tcgetattr(terminal_fd, &original_termios) == 0) {
            termios raw_termios = original_termios;
            raw_termios.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw_termios.c_cc[VMIN] = 0;
            raw_termios.c_cc[VTIME] = 0;
            restore_terminal =
                tcsetattr(terminal_fd, TCSANOW, &raw_termios) == 0;
        }
        while (rclcpp::ok() && !stop_keyboard_thread_.load()) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(terminal_fd, &read_fds);
            timeval timeout{0, 10000};
            const int ready = select(
                terminal_fd + 1, &read_fds, nullptr, nullptr, &timeout);
            if (ready <= 0 || !FD_ISSET(terminal_fd, &read_fds)) {
                continue;
            }
            char input = '\0';
            if (read(terminal_fd, &input, 1) == 1 &&
                input == emergency_stop_key_.front())
            {
                trigger_emergency_stop();
            }
        }
        if (restore_terminal) {
            tcsetattr(terminal_fd, TCSANOW, &original_termios);
        }
        close(terminal_fd);
    }

    std::uint16_t axis_pwm(const double value, const bool invert) const
    {
        const double axis = invert ? -value : value;
        const int pwm = static_cast<int>(std::llround(
            static_cast<double>(RC_NEUTRAL) +
            std::clamp(axis, -1.0, 1.0) * rc_pwm_span_));
        return static_cast<std::uint16_t>(std::clamp(pwm, 1100, 1900));
    }

    static double wrap_pi(const double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    static const char * state_name(const State state)
    {
        switch (state) {
            case State::MOVE_TO_LINE_CENTER: return "MOVE_TO_LINE_CENTER";
            case State::LINE_SEARCH: return "LINE_SEARCH";
            case State::RETURN_TO_PEAK: return "RETURN_TO_PEAK";
            case State::HANDOFF_NEUTRAL: return "HANDOFF_NEUTRAL";
            case State::HANDOFF_COMPLETE: return "HANDOFF_COMPLETE";
        }
        return "UNKNOWN";
    }

    double arena_length_m_ = 15.0;
    double arena_width_m_ = 16.0;
    double arena_offset_x_m_ = 0.0;
    double arena_offset_y_m_ = 0.0;
    double arena_safety_margin_m_ = 0.5;
    double vision_near_zone_width_m_ = 2.0;
    double acoustic_timeout_s_ = 90.0;
    double target_depth_z_m_ = -8.0;
    double line_search_start_depth_tolerance_m_ = 0.2;
    double waypoint_reach_tolerance_m_ = 0.15;
    double snr_sample_spacing_m_ = 0.15;
    double snr_drop_from_peak_db_ = 2.0;
    double snr_timeout_s_ = 1.0;
    double max_snr_odom_skew_s_ = 0.15;
    double odometry_timeout_s_ = 0.5;
    double rate_hz_ = 30.0;
    double forward_cruise_ = 0.5;
    double yaw_kp_ = 1.15;
    double yaw_ki_ = 0.15;
    double yaw_kd_ = 0.08;
    double yaw_integral_limit_ = 2.0;
    double yaw_limit_ = 0.72;
    double move_heading_tolerance_rad_ = 0.1745;
    double rc_pwm_span_ = 400.0;
    std::size_t snr_decline_count_limit_ = 5;
    std::size_t snr_median_window_size_ = 3;
    std::size_t decline_count_ = 0;
    int line_search_direction_ = 1;
    std::string arena_start_corner_ = "bottom_left";
    std::string emergency_stop_key_ = "s";
    std::string arena_frame_id_ = "arena";
    State state_ = State::MOVE_TO_LINE_CENTER;
    bool have_odometry_ = false;
    bool have_snr_ = false;
    bool have_peak_ = false;
    bool heading_aligned_ = false;
    bool return_to_peak_requested_ = false;
    bool vision_search_requested_ = false;
    bool handoff_neutral_sent_ = false;
    bool acoustic_deadline_started_ = false;
    bool invert_rc_yaw_ = true;
    bool yaw_pid_initialized_ = false;
    bool depth_pid_initialized_ = false;
    rclcpp::Time last_odometry_receive_time_;
    rclcpp::Time last_snr_receive_time_;
    rclcpp::Time last_yaw_control_time_;
    rclcpp::Time last_depth_control_time_;
    rclcpp::Time handoff_neutral_time_;
    rclcpp::Time acoustic_start_time_;
    double current_z_m_ = 0.0;
    double initial_z_m_ = 0.0;
    double current_yaw_rad_ = 0.0;
    double peak_snr_db_ = 0.0;
    double previous_yaw_error_ = 0.0;
    double yaw_error_integral_ = 0.0;
    double yaw_error_derivative_ = 0.0;
    double depth_error_integral_ = 0.0;
    Eigen::Vector2d current_position_{0.0, 0.0};
    Eigen::Vector2d current_waypoint_{0.0, 0.0};
    Eigen::Vector2d line_center_{0.0, 0.0};
    Eigen::Vector2d line_endpoint_{0.0, 0.0};
    Eigen::Vector2d peak_position_{0.0, 0.0};
    std::optional<Eigen::Vector2d> last_sample_position_;
    std::deque<SnrSample> snr_filter_window_;
    std::deque<PoseSample> odometry_history_;
    hydrophone_ctrl::ArenaFrameTransform arena_transform_;
    std::atomic_bool emergency_stop_active_{false};
    std::atomic_bool stop_keyboard_thread_{false};
    std::thread keyboard_thread_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        start_frame_sub_;
    rclcpp::Subscription<audio_common_msgs::msg::Float64Stamped>::SharedPtr snr_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_confirmed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr peak_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        vision_search_request_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        vision_control_granted_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::NearZoneLineSearchControllerNode)
