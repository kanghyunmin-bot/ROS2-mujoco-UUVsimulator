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

#include <audio_capture/arena_frame_transform.hpp>
#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
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
        const auto snr_topic = declare_parameter<std::string>(
            "snr_topic", "/audio_frequency_detector/snr_db_stamped");
        const auto state_topic = declare_parameter<std::string>(
            "state_topic", "/homing/control_state");
        const auto waypoint_topic = declare_parameter<std::string>(
            "waypoint_topic", "/waypoint");
        const auto arena_start_frame_topic = declare_parameter<std::string>(
            "arena_start_frame_topic", "/guided/start_frame");
        const auto peak_topic = declare_parameter<std::string>(
            "peak_topic", "/homing/snr_peak_position");
        const auto vision_search_request_topic = declare_parameter<std::string>(
            "vision_search_request_topic", "/homing/vision_search_active");
        const auto target_confirmed_topic = declare_parameter<std::string>(
            "target_confirmed_topic", "/vision/target_confirmed");
        const auto vision_control_granted_topic = declare_parameter<std::string>(
            "vision_control_granted_topic", "/homing/vision_control_granted");
        const auto guided_waypoint_enable_topic =
            declare_parameter<std::string>(
                "guided_waypoint_enable_topic", "/guided/waypoint_enable");
        const auto guided_status_topic = declare_parameter<std::string>(
            "guided_status_topic", "/guided/status");
        const auto fcu_state_topic = declare_parameter<std::string>(
            "fcu_state_topic", "/mavros/state");
        const auto set_mode_service = declare_parameter<std::string>(
            "set_mode_service", "/mavros/set_mode");
        vision_mode_name_ = declare_parameter<std::string>(
            "vision_mode_name", "STABILIZE");
        if (vision_mode_name_.empty()) {
            throw std::invalid_argument("vision_mode_name must not be empty");
        }
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
        fcu_state_timeout_s_ = std::max(
            0.05, declare_parameter<double>("fcu_state_timeout_s", 1.0));
        handoff_hold_sec_ = std::max(
            0.0, declare_parameter<double>("handoff_hold_sec", 0.7));
        handoff_max_speed_mps_ = std::max(
            0.0, declare_parameter<double>("handoff_max_speed_mps", 0.2));
        mode_request_interval_s_ = std::max(
            0.1, declare_parameter<double>("mode_request_interval_s", 1.0));
        rate_hz_ = std::clamp(
            declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);

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
        arena_start_frame_sub_ =
            create_subscription<geometry_msgs::msg::PoseStamped>(
                arena_start_frame_topic,
                rclcpp::QoS(1).reliable().transient_local(),
                std::bind(
                    &NearZoneLineSearchControllerNode::arena_start_frame_callback,
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
        guided_status_sub_ = create_subscription<std_msgs::msg::String>(
            guided_status_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(
                &NearZoneLineSearchControllerNode::guided_status_callback,
                this, std::placeholders::_1));
        fcu_state_sub_ = create_subscription<mavros_msgs::msg::State>(
            fcu_state_topic, rclcpp::QoS(10).reliable(),
            std::bind(
                &NearZoneLineSearchControllerNode::fcu_state_callback,
                this, std::placeholders::_1));
        emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            emergency_stop_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(
                &NearZoneLineSearchControllerNode::emergency_stop_callback,
                this, std::placeholders::_1));

        state_pub_ = create_publisher<std_msgs::msg::String>(
            state_topic, rclcpp::QoS(1).reliable().transient_local());
        waypoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
            waypoint_topic, rclcpp::QoS(10).reliable());
        peak_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            peak_topic, rclcpp::QoS(1).reliable().transient_local());
        vision_search_request_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_search_request_topic,
            rclcpp::QoS(1).reliable().transient_local());
        vision_control_granted_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_control_granted_topic,
            rclcpp::QoS(1).reliable().transient_local());
        guided_waypoint_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
            guided_waypoint_enable_topic, rclcpp::QoS(10).reliable());
        set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(
            set_mode_service);
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
            "Near-zone line search ready: drop=%.1f dB count=%zu spacing=%.2f m",
            snr_drop_from_peak_db_, snr_decline_count_limit_,
            snr_sample_spacing_m_);
    }

    ~NearZoneLineSearchControllerNode() override
    {
        stop_keyboard_thread_.store(true);
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    static constexpr double DEPTH_TOLERANCE_M = 0.10;

    enum class State
    {
        MOVE_TO_LINE_CENTER,
        LINE_SEARCH,
        RETURN_TO_PEAK,
        WAIT_VISION_TARGET,
        HANDOFF_PREPARE,
        HANDOFF_COMPLETE
    };

    struct ArenaBounds
    {
        double x_min = 0.0;
        double x_max = 0.0;
        double y_min = 0.0;
        double y_max = 0.0;
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

    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        const Eigen::Vector2d position(
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        const double z_m = msg->pose.pose.position.z;
        const auto & linear_velocity = msg->twist.twist.linear;
        const double speed_mps = std::sqrt(
            linear_velocity.x * linear_velocity.x +
            linear_velocity.y * linear_velocity.y +
            linear_velocity.z * linear_velocity.z);
        if (!position.allFinite() || !std::isfinite(z_m) ||
            !std::isfinite(speed_mps))
        {
            return;
        }
        current_position_ = position;
        current_z_m_ = z_m;
        current_speed_mps_ = speed_mps;
        odometry_frame_ =
            msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;
        last_odometry_receive_time_ = now();
        have_odometry_ = true;
        const rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() > 0) {
            if (!odometry_history_.empty() &&
                stamp < odometry_history_.back().stamp)
            {
                odometry_history_.clear();
            }
            odometry_history_.push_back({stamp, position});
            while (odometry_history_.size() > 300) {
                odometry_history_.pop_front();
            }
        }

        try_start_mission();
    }

    void arena_start_frame_callback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        if (mission_started_) {
            RCLCPP_WARN(
                get_logger(),
                "[ARENA] ignored start-frame update after acoustic mission start");
            return;
        }
        std::string error;
        if (!arena_frame_.update(*msg, error)) {
            RCLCPP_WARN(
                get_logger(), "[ARENA] invalid start frame: %s", error.c_str());
            return;
        }
        RCLCPP_INFO(
            get_logger(),
            "[ARENA] start frame received: parent=%s origin=(%.3f, %.3f) "
            "yaw=%.2f deg",
            arena_frame_.parent_frame().c_str(),
            arena_frame_.origin().x(), arena_frame_.origin().y(),
            arena_frame_.yaw_rad() * 180.0 / 3.14159265358979323846);
        try_start_mission();
    }

    void try_start_mission()
    {
        if (mission_started_ || !have_odometry_ || !arena_frame_.ready()) {
            return;
        }
        if (arena_frame_.parent_frame() !=
            ArenaFrameTransform2D::normalized_frame(odometry_frame_))
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[ARENA] start-frame parent '%s' does not match odometry frame '%s'",
                arena_frame_.parent_frame().c_str(), odometry_frame_.c_str());
            return;
        }
        mission_started_ = true;
        line_center_ = nearest_near_zone_center(current_position_);
        set_current_waypoint(line_center_);
        publish_state();
        RCLCPP_INFO(
            get_logger(),
            "[LINE] moving to rotated near-zone center odom=(%.2f, %.2f), "
            "depth=%.2f m",
            line_center_.x(), line_center_.y(), target_depth_z_m_);
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
        if (!msg->data || !vision_search_requested_ ||
            state_ != State::WAIT_VISION_TARGET)
        {
            return;
        }
        set_current_waypoint(current_position_);
        handoff_stable_since_.reset();
        guided_disable_sent_ = false;
        guided_controller_idle_ = false;
        mode_request_pending_ = false;
        publish_vision_control_granted(false);
        transition_to(State::HANDOFF_PREPARE);
        RCLCPP_INFO(
            get_logger(),
            "[HANDOFF] target confirmed; stabilizing before releasing acoustic control");
    }

    void guided_status_callback(
        const std_msgs::msg::String::ConstSharedPtr msg)
    {
        const bool was_idle = guided_controller_idle_;
        guided_controller_idle_ = msg->data.rfind("IDLE:", 0) == 0;
        if (!was_idle && guided_controller_idle_ &&
            state_ == State::HANDOFF_PREPARE)
        {
            RCLCPP_INFO(
                get_logger(), "[HANDOFF] external waypoint controller is IDLE");
        }
    }

    void fcu_state_callback(
        const mavros_msgs::msg::State::ConstSharedPtr msg)
    {
        have_fcu_state_ = true;
        fcu_connected_ = msg->connected;
        current_fcu_mode_ = msg->mode;
        last_fcu_state_receive_time_ = now();
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
        if (have_current_waypoint_ && state_ != State::HANDOFF_COMPLETE) {
            publish_current_waypoint();
        }
        if (emergency_stop_active_.load()) {
            if (!emergency_hold_published_ && have_odometry_) {
                set_current_waypoint(current_position_);
                emergency_hold_published_ = true;
            }
            return;
        }
        if (state_ == State::WAIT_VISION_TARGET ||
            state_ == State::HANDOFF_COMPLETE)
        {
            return;
        }
        if (!mission_started_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[ARENA] waiting for odometry and /guided/start_frame");
            return;
        }
        if (!odometry_is_fresh(current_time)) {
            return;
        }

        switch (state_) {
            case State::MOVE_TO_LINE_CENTER:
                if (waypoint_reached() && depth_target_reached()) {
                    begin_line_search();
                }
                return;
            case State::LINE_SEARCH:
                if (return_to_peak_requested_) {
                    begin_return_to_peak("sustained_snr_decline");
                    return;
                }
                if (!snr_is_fresh(current_time)) {
                    if (!line_search_paused_) {
                        set_current_waypoint(current_position_);
                        line_search_paused_ = true;
                    }
                    return;
                }
                if (line_search_paused_) {
                    set_current_waypoint(line_endpoint_);
                    line_search_paused_ = false;
                }
                if (waypoint_reached()) {
                    begin_return_to_peak("line_boundary");
                }
                return;
            case State::RETURN_TO_PEAK:
                if (waypoint_reached()) {
                    begin_vision_wait();
                }
                return;
            case State::HANDOFF_PREPARE:
                run_handoff_prepare(current_time);
                return;
            case State::WAIT_VISION_TARGET:
            case State::HANDOFF_COMPLETE:
                return;
        }
    }

    void run_handoff_prepare(const rclcpp::Time & current_time)
    {
        if (!guided_disable_sent_) {
            const bool stable =
                waypoint_reached() && depth_target_reached() &&
                current_speed_mps_ <= handoff_max_speed_mps_;
            if (!stable) {
                handoff_stable_since_.reset();
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "[HANDOFF] holding position: distance=%.2f m speed=%.2f m/s",
                    (current_waypoint_ - current_position_).norm(),
                    current_speed_mps_);
                return;
            }
            if (!handoff_stable_since_) {
                handoff_stable_since_ = current_time;
                return;
            }
            if ((current_time - *handoff_stable_since_).seconds() <
                handoff_hold_sec_)
            {
                return;
            }
            if (!fcu_state_is_fresh(current_time) || !fcu_connected_ ||
                !set_mode_client_->service_is_ready())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "[HANDOFF] waiting for a connected FCU and set-mode service");
                return;
            }

            publish_guided_waypoint_enabled(false);
            guided_disable_sent_ = true;
            RCLCPP_INFO(
                get_logger(),
                "[HANDOFF] external waypoint control disable requested");
            return;
        }

        if (!guided_controller_idle_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[HANDOFF] waiting for /guided/status IDLE confirmation");
            return;
        }

        if (fcu_state_is_fresh(current_time) && fcu_connected_ &&
            current_fcu_mode_ == vision_mode_name_)
        {
            publish_vision_control_granted(true);
            transition_to(State::HANDOFF_COMPLETE);
            RCLCPP_INFO(
                get_logger(),
                "[HANDOFF] FCU mode %s confirmed; vision control granted",
                vision_mode_name_.c_str());
            return;
        }
        request_vision_mode(current_time);
    }

    void request_vision_mode(const rclcpp::Time & current_time)
    {
        if (!fcu_state_is_fresh(current_time) || !fcu_connected_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[HANDOFF] waiting for fresh connected FCU state");
            return;
        }
        if (mode_request_pending_ || !set_mode_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[HANDOFF] waiting for set-mode service");
            return;
        }

        const auto steady_now = std::chrono::steady_clock::now();
        if (last_mode_request_time_.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(
                steady_now - last_mode_request_time_).count() <
            mode_request_interval_s_)
        {
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->base_mode = 0;
        request->custom_mode = vision_mode_name_;
        mode_request_pending_ = true;
        last_mode_request_time_ = steady_now;
        set_mode_client_->async_send_request(
            request,
            [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                mode_request_pending_ = false;
                if (!future.get()->mode_sent) {
                    RCLCPP_WARN(
                        get_logger(),
                        "[HANDOFF] FCU rejected mode request for %s",
                        vision_mode_name_.c_str());
                }
            });
        RCLCPP_INFO(
            get_logger(), "[HANDOFF] requested FCU mode %s",
            vision_mode_name_.c_str());
    }

    void begin_line_search()
    {
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        Eigen::Vector2d endpoint_arena =
            arena_frame_.odom_to_arena(line_center_);
        endpoint_arena.x() =
            line_search_direction_ > 0 ? bounds.x_max : bounds.x_min;
        line_endpoint_ = arena_frame_.arena_to_odom(endpoint_arena);
        have_snr_ = false;
        have_peak_ = false;
        return_to_peak_requested_ = false;
        line_search_paused_ = false;
        decline_count_ = 0;
        last_sample_position_.reset();
        snr_filter_window_.clear();
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
            set_current_waypoint(current_position_);
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

    void begin_vision_wait()
    {
        vision_search_requested_ = true;
        publish_vision_control_granted(false);
        publish_vision_search_request(true);
        set_current_waypoint(current_position_);
        transition_to(State::WAIT_VISION_TARGET);
        RCLCPP_INFO(
            get_logger(),
            "[VISION] peak reached; waiting for target confirmation");
    }

    bool waypoint_reached() const
    {
        return (current_waypoint_ - current_position_).norm() <=
            waypoint_reach_tolerance_m_;
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
        Eigen::Vector2d arena_position =
            arena_frame_.odom_to_arena(position);
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        const double width = std::min(
            vision_near_zone_width_m_, bounds.y_max - bounds.y_min);
        const double y = arena_start_corner_ == "bottom_left" ?
            bounds.y_min + 0.5 * width :
            bounds.y_max - 0.5 * width;
        arena_position.x() =
            std::clamp(arena_position.x(), bounds.x_min, bounds.x_max);
        arena_position.y() = y;
        return arena_frame_.arena_to_odom(arena_position);
    }

    bool odometry_is_fresh(const rclcpp::Time & current_time) const
    {
        return have_odometry_ &&
            (current_time - last_odometry_receive_time_).seconds() <=
            odometry_timeout_s_;
    }

    bool fcu_state_is_fresh(const rclcpp::Time & current_time) const
    {
        return have_fcu_state_ &&
            (current_time - last_fcu_state_receive_time_).seconds() <=
            fcu_state_timeout_s_;
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

    bool depth_target_reached() const
    {
        return std::abs(target_depth_z_m_ - current_z_m_) <= DEPTH_TOLERANCE_M;
    }

    void set_current_waypoint(const Eigen::Vector2d & waypoint)
    {
        current_waypoint_ = waypoint;
        have_current_waypoint_ = true;
        publish_current_waypoint();
    }

    void publish_current_waypoint()
    {
        mavros_msgs::msg::PositionTarget msg;
        msg.header.stamp = now();
        msg.header.frame_id = odometry_frame_;
        msg.coordinate_frame =
            mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        msg.position.x = current_waypoint_.x();
        msg.position.y = current_waypoint_.y();
        msg.position.z = target_depth_z_m_;
        waypoint_pub_->publish(msg);
    }

    void publish_peak_position()
    {
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = odometry_frame_;
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

    void publish_guided_waypoint_enabled(const bool enabled)
    {
        std_msgs::msg::Bool msg;
        msg.data = enabled;
        guided_waypoint_enable_pub_->publish(msg);
    }

    void trigger_emergency_stop()
    {
        if (emergency_stop_active_.exchange(true)) {
            return;
        }
        std_msgs::msg::Bool msg;
        msg.data = true;
        emergency_stop_pub_->publish(msg);
        RCLCPP_ERROR(
            get_logger(), "[EMERGENCY] key '%c' pressed; hold waypoint requested",
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

    static const char * state_name(const State state)
    {
        switch (state) {
            case State::MOVE_TO_LINE_CENTER: return "MOVE_TO_LINE_CENTER";
            case State::LINE_SEARCH: return "LINE_SEARCH";
            case State::RETURN_TO_PEAK: return "RETURN_TO_PEAK";
            case State::WAIT_VISION_TARGET: return "WAIT_VISION_TARGET";
            case State::HANDOFF_PREPARE: return "HANDOFF_PREPARE";
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
    double target_depth_z_m_ = -8.0;
    double waypoint_reach_tolerance_m_ = 0.15;
    double snr_sample_spacing_m_ = 0.15;
    double snr_drop_from_peak_db_ = 2.0;
    double snr_timeout_s_ = 1.0;
    double max_snr_odom_skew_s_ = 0.15;
    double odometry_timeout_s_ = 0.5;
    double fcu_state_timeout_s_ = 1.0;
    double handoff_hold_sec_ = 0.7;
    double handoff_max_speed_mps_ = 0.2;
    double mode_request_interval_s_ = 1.0;
    double rate_hz_ = 30.0;
    std::size_t snr_decline_count_limit_ = 5;
    std::size_t snr_median_window_size_ = 3;
    std::size_t decline_count_ = 0;
    int line_search_direction_ = 1;
    std::string arena_start_corner_ = "bottom_left";
    std::string emergency_stop_key_ = "s";
    std::string odometry_frame_ = "odom";
    std::string vision_mode_name_ = "STABILIZE";
    State state_ = State::MOVE_TO_LINE_CENTER;
    bool have_odometry_ = false;
    bool have_snr_ = false;
    bool have_peak_ = false;
    bool return_to_peak_requested_ = false;
    bool line_search_paused_ = false;
    bool vision_search_requested_ = false;
    bool emergency_hold_published_ = false;
    bool have_current_waypoint_ = false;
    bool mission_started_ = false;
    bool guided_controller_idle_ = false;
    bool guided_disable_sent_ = false;
    bool have_fcu_state_ = false;
    bool fcu_connected_ = false;
    bool mode_request_pending_ = false;
    rclcpp::Time last_odometry_receive_time_;
    rclcpp::Time last_snr_receive_time_;
    rclcpp::Time last_fcu_state_receive_time_;
    double current_z_m_ = 0.0;
    double current_speed_mps_ = 0.0;
    double peak_snr_db_ = 0.0;
    Eigen::Vector2d current_position_{0.0, 0.0};
    Eigen::Vector2d current_waypoint_{0.0, 0.0};
    Eigen::Vector2d line_center_{0.0, 0.0};
    Eigen::Vector2d line_endpoint_{0.0, 0.0};
    Eigen::Vector2d peak_position_{0.0, 0.0};
    std::optional<Eigen::Vector2d> last_sample_position_;
    std::optional<rclcpp::Time> handoff_stable_since_;
    std::deque<SnrSample> snr_filter_window_;
    std::deque<PoseSample> odometry_history_;
    std::chrono::steady_clock::time_point last_mode_request_time_;
    std::string current_fcu_mode_;
    ArenaFrameTransform2D arena_frame_;
    std::atomic_bool emergency_stop_active_{false};
    std::atomic_bool stop_keyboard_thread_{false};
    std::thread keyboard_thread_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        arena_start_frame_sub_;
    rclcpp::Subscription<audio_common_msgs::msg::Float64Stamped>::SharedPtr snr_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_confirmed_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr guided_status_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr fcu_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr peak_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        vision_search_request_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        vision_control_granted_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        guided_waypoint_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::NearZoneLineSearchControllerNode)
