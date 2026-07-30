#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
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
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace audio_capture
{
// 초기 pose를 arena (0,0,+X)로 고정하고 모든 scan/homing 이동을 절대 odometry
// waypoint로 수행하고 odometry z로 목표 depth를 동시에 유지한다.
class WaypointHomingControllerNode : public rclcpp::Node
{
public:
    explicit WaypointHomingControllerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("waypoint_homing_controller", options)
    {
        const auto odometry_topic = declare_parameter<std::string>(
            "odometry_topic", "/odometry/filtered");
        const auto start_frame_topic = declare_parameter<std::string>(
            "start_frame_topic", "/start_frame");
        const auto snr_topic = declare_parameter<std::string>(
            "snr_topic", "/audio_frequency_detector/snr_db_stamped");
        const auto region_gradient_topic = declare_parameter<std::string>(
            "region_gradient_topic", "/homing/region_gradient");
        const auto rolling_gradient_topic = declare_parameter<std::string>(
            "rolling_gradient_topic", "/homing/rolling_gradient");
        const auto homing_direction_topic = declare_parameter<std::string>(
            "homing_direction_topic", "/homing/homing_direction");
        const auto direction_body_topic = declare_parameter<std::string>(
            "direction_body_topic", "/mission/hydrophone/direction_body");
        const auto state_topic = declare_parameter<std::string>(
            "state_topic", "/homing/control_state");
        const auto waypoint_topic = declare_parameter<std::string>(
            "waypoint_topic", "/homing/current_waypoint");
        arena_frame_id_ = declare_parameter<std::string>(
            "arena_frame_id", "arena");
        const auto scan_center_topic = declare_parameter<std::string>(
            "scan_center_topic", "/homing/scan_center");
        const auto vision_search_request_topic = declare_parameter<std::string>(
            "vision_search_request_topic", "/homing/vision_search_active");
        const auto target_confirmed_topic = declare_parameter<std::string>(
            "target_confirmed_topic", "/vision/target_confirmed");
        // [ACOUSTIC-VISION HANDSHAKE] Vision은 이 승인 후에만 RC를 발행한다.
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
        if (!std::isfinite(arena_offset_x_m_) || !std::isfinite(arena_offset_y_m_)) {
            throw std::invalid_argument("arena offsets must be finite");
        }
        arena_start_corner_ = declare_parameter<std::string>(
            "arena_start_corner", "bottom_left");
        if (arena_start_corner_ != "bottom_left" &&
            arena_start_corner_ != "bottom_right")
        {
            throw std::invalid_argument(
                "arena_start_corner must be bottom_left or bottom_right");
        }
        arena_safety_margin_m_ = std::max(
            0.0, declare_parameter<double>("arena_safety_margin_m", 0.5));
        initial_scan_radius_m_ = std::max(
            0.01, declare_parameter<double>("initial_scan_radius_m", 1.5));
        rescan_radius_m_ = std::max(
            0.01, declare_parameter<double>("rescan_radius_m", 0.7));
        use_explicit_initial_scan_center_ = declare_parameter<bool>(
            "use_explicit_initial_scan_center", false);
        initial_scan_center_x_m_ = declare_parameter<double>(
            "initial_scan_center_x_m", 0.0);
        initial_scan_center_y_m_ = declare_parameter<double>(
            "initial_scan_center_y_m", 0.0);
        if (2.0 * (arena_safety_margin_m_ +
            std::max(initial_scan_radius_m_, rescan_radius_m_)) >
            std::min(arena_length_m_, arena_width_m_))
        {
            throw std::invalid_argument(
                "arena is too small for scan radius and arena_safety_margin_m");
        }

        homing_waypoint_step_m_ = std::max(
            0.05, declare_parameter<double>("homing_waypoint_step_m", 0.8));
        homing_zigzag_offset_m_ = std::max(
            0.0, declare_parameter<double>("homing_zigzag_offset_m", 0.2));
        rolling_gradient_alpha_ = std::clamp(
            declare_parameter<double>("rolling_gradient_alpha", 0.15), 0.0, 1.0);
        rolling_gradient_conflict_angle_rad_ = std::clamp(
            declare_parameter<double>(
                "rolling_gradient_conflict_angle_rad", PI / 3.0),
            0.0, PI);
        rolling_gradient_conflict_limit_ =
            static_cast<std::size_t>(std::max<std::int64_t>(
                1,
                declare_parameter<std::int64_t>(
                    "rolling_gradient_conflict_limit", 3)));
        waypoint_reach_tolerance_m_ = std::max(
            0.01, declare_parameter<double>("waypoint_reach_tolerance_m", 0.15));
        scan_radial_kp_ = std::max(
            0.0, declare_parameter<double>("scan_radial_kp", 1.5));
        scan_radial_ki_ = std::max(
            0.0, declare_parameter<double>("scan_radial_ki", 0.05));
        scan_radial_kd_ = std::max(
            0.0, declare_parameter<double>("scan_radial_kd", 0.3));
        scan_radial_integral_limit_ = std::max(
            0.0, declare_parameter<double>(
                "scan_radial_integral_limit", 1.0));
        scan_lookahead_rad_ = std::clamp(
            declare_parameter<double>("scan_lookahead_rad", 0.30),
            0.08, 1.0);
        scan_xy_gain_ = std::clamp(
            declare_parameter<double>("scan_xy_gain", 0.85), 0.1, 3.0);
        scan_sway_gain_ = std::clamp(
            declare_parameter<double>("scan_sway_gain", 1.10), 0.0, 4.0);
        scan_sway_limit_ = std::clamp(
            declare_parameter<double>("scan_sway_limit", 0.45), 0.0, 1.0);
        scan_heading_gate_rad_ = std::clamp(
            declare_parameter<double>("scan_heading_gate_rad", 0.50),
            0.05, PI);
        scan_completion_radius_tolerance_m_ = std::clamp(
            declare_parameter<double>(
                "scan_completion_radius_tolerance_m", 0.30),
            0.05, 1.0);
        vision_near_zone_width_m_ = std::clamp(
            declare_parameter<double>("vision_near_zone_width_m", 2.0),
            0.0, arena_width_m_);
        vision_handoff_enabled_ = declare_parameter<bool>(
            "vision_handoff_enabled", true);
        // 0 이하면 Acoustic 타임아웃 핸드오프를 끈다. near zone 진입 시에는 즉시 인계한다.
        acoustic_timeout_s_ = declare_parameter<double>("acoustic_timeout_s", 90.0);
        if (!std::isfinite(acoustic_timeout_s_)) {
            throw std::invalid_argument("acoustic_timeout_s must be finite");
        }
        success_snr_db_ = declare_parameter<double>("success_snr_db", 70.0);
        success_hold_s_ = std::max(
            0.1, declare_parameter<double>("success_hold_s", 1.0));
        success_snr_timeout_s_ = std::max(
            0.1, declare_parameter<double>("success_snr_timeout_s", 1.0));
        success_snr_window_size_ =
            static_cast<std::size_t>(std::max<std::int64_t>(
                3,
                declare_parameter<std::int64_t>(
                    "success_snr_window_size", 15)));
        target_depth_z_m_ = declare_parameter<double>("target_depth_z_m", -0.65);
        if (!std::isfinite(target_depth_z_m_)) {
            throw std::invalid_argument("target_depth_z_m must be finite");
        }
        depth_tolerance_m_ = std::max(
            0.0, declare_parameter<double>("depth_tolerance_m", 0.10));
        depth_kp_ = std::max(
            0.0, declare_parameter<double>("depth_kp", 0.8));
        depth_ki_ = std::max(
            0.0, declare_parameter<double>("depth_ki", 0.15));

        rate_hz_ = std::clamp(
            declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
        odometry_timeout_s_ = std::max(
            0.05, declare_parameter<double>("odometry_timeout_s", 0.5));
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
            declare_parameter<double>("move_heading_tolerance_rad", 0.1745), 0.01, PI);
        vision_heading_tolerance_rad_ = std::clamp(
            declare_parameter<double>("vision_heading_tolerance_rad", 0.12), 0.01, PI);
        rc_pwm_span_ = std::clamp(
            declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);
        invert_rc_yaw_ = declare_parameter<bool>("invert_rc_yaw", true);
        invert_rc_lateral_ = declare_parameter<bool>("invert_rc_lateral", false);

        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 30,
            std::bind(&WaypointHomingControllerNode::odometry_callback, this,
                std::placeholders::_1));
        start_frame_sub_ =
            create_subscription<geometry_msgs::msg::PoseStamped>(
                start_frame_topic,
                rclcpp::QoS(1).reliable().transient_local(),
                std::bind(
                    &WaypointHomingControllerNode::start_frame_callback,
                    this, std::placeholders::_1));
        snr_sub_ =
            create_subscription<audio_common_msgs::msg::Float64Stamped>(
                snr_topic, 30,
                std::bind(
                    &WaypointHomingControllerNode::snr_callback,
                    this, std::placeholders::_1));
        region_gradient_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            region_gradient_topic, 10,
            std::bind(&WaypointHomingControllerNode::region_gradient_callback, this,
                std::placeholders::_1));
        rolling_gradient_sub_ =
            create_subscription<geometry_msgs::msg::Vector3Stamped>(
                rolling_gradient_topic, 10,
                std::bind(&WaypointHomingControllerNode::rolling_gradient_callback,
                    this, std::placeholders::_1));
        target_confirmed_sub_ = create_subscription<std_msgs::msg::Bool>(
            target_confirmed_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&WaypointHomingControllerNode::target_confirmed_callback, this,
                std::placeholders::_1));
        state_pub_ = create_publisher<std_msgs::msg::String>(
            state_topic, rclcpp::QoS(1).reliable().transient_local());
        waypoint_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            waypoint_topic, rclcpp::QoS(1).reliable().transient_local());
        scan_center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            scan_center_topic, rclcpp::QoS(1).reliable().transient_local());
        vision_search_request_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_search_request_topic,
            rclcpp::QoS(1).reliable().transient_local());
        vision_control_granted_pub_ = create_publisher<std_msgs::msg::Bool>(
            vision_control_granted_topic,
            rclcpp::QoS(1).reliable().transient_local());
        homing_direction_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>(
                homing_direction_topic,
                rclcpp::QoS(1).reliable().transient_local());
        direction_body_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>(
                direction_body_topic, 10);
        rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(
            rc_override_topic, 10);
        emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
            emergency_stop_topic, rclcpp::QoS(1).reliable().transient_local());
        success_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/homing/success", rclcpp::QoS(1).reliable().transient_local());
        emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            emergency_stop_topic, rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&WaypointHomingControllerNode::emergency_stop_callback, this,
                std::placeholders::_1));

        publish_state();
        publish_success(false);
        publish_vision_search_request(false);
        publish_vision_control_granted(false);
        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&WaypointHomingControllerNode::control_loop, this));

        if (enable_keyboard_emergency_stop && !emergency_stop_key_.empty()) {
            keyboard_thread_ = std::thread(
                &WaypointHomingControllerNode::keyboard_loop, this);
            RCLCPP_INFO(
                get_logger(),
                "Emergency neutral enabled. Press '%c' to latch neutral RC.",
                emergency_stop_key_.front());
        }

        const ArenaBounds bounds = arena_bounds(0.0);
        RCLCPP_INFO(
            get_logger(),
            "Waypoint controller ready: arena x=[%.3f, %.3f] y=[%.3f, %.3f], "
            "depth_pi=(kp=%.3f, ki=%.3f), acoustic_timeout_s=%.1f "
            "(near-zone request; boundary/confirm/timeout -> grant)",
            bounds.x_min, bounds.x_max, bounds.y_min, bounds.y_max,
            depth_kp_, depth_ki_,
            acoustic_timeout_s_);
    }

    ~WaypointHomingControllerNode() override
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
    static constexpr double HEAVE_LIMIT = 0.2;
    static constexpr double REALIGN_HEADING_ERROR_RAD = PI / 3.0;
    static constexpr double YAW_DERIVATIVE_ALPHA = 0.2;
    static constexpr double SCAN_RADIAL_DERIVATIVE_ALPHA = 0.2;
    static constexpr double SCAN_RADIAL_CORRECTION_LIMIT = 1.0;

    enum class State
    {
        MOVE_TO_SCAN_CENTER,    //원형 탐색 중심 위치로 이동 상태
        MOVE_TO_SCAN_START,     //원형 궤도 시작점으로 이동 상태
        REGION_SCAN,    //원형 탐색 상태
        REGION_HOMING,    //원형 탐색 결과 그래디언트 매칭 상태
        SUCCESS,         // SNR 기준으로 핑거 접근 성공
        HANDOFF_NEUTRAL,  // [ACOUSTIC-VISION HANDSHAKE] 중립 RC를 딱 한 번 보낸다.
        HANDOFF_COMPLETE  // [ACOUSTIC-VISION HANDSHAKE] Acoustic RC를 영구 종료한다.
    };

    enum class HomingWaypointResult
    {
        CREATED,
        VISION_ZONE,
        BOUNDARY
    };

    enum class RegionResultState
    {
        WAITING,
        INVALID,
        VALID
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
        double sway = 0.0;
        double yaw = 0.0;
        double heave = 0.0;
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
        // 현재 위치와 yaw를 업데이트한다.
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
        current_position_ = arena_transform_.position_from_odom(odom_position);
        current_z_m_ = z_m;
        current_yaw_rad_ = arena_transform_.yaw_from_odom(odom_yaw);
        last_odometry_receive_time_ = now(); // 마지막 오도메트리 수신 시간을 업데이트한다.
        have_odometry_ = true; // 오도메트리 수신 여부를 업데이트한다.

        if (first_odometry) { // 최초 odometry 수신 시 미션을 시작한다.
            start_acoustic_deadline_if_needed(now());
            const Eigen::Vector2d requested_center =
                use_explicit_initial_scan_center_ ?
                Eigen::Vector2d(
                    initial_scan_center_x_m_, initial_scan_center_y_m_) :
                arena_center();
            start_new_region_scan(requested_center, true);
        }
    }

    void snr_callback(
        const audio_common_msgs::msg::Float64Stamped::ConstSharedPtr msg)
    {
        if (!std::isfinite(msg->data)) {
            return;
        }
        latest_snr_db_ = msg->data;
        last_snr_receive_time_ = now();
        have_snr_ = true;
        if (state_ != State::REGION_HOMING) {
            return;
        }
        success_snr_window_.push_back(msg->data);
        while (success_snr_window_.size() > success_snr_window_size_) {
            success_snr_window_.pop_front();
        }
    }


    // REGION_SCAN 중에 estimator가 보낸 원형 스캔 그래디언트를 받아 저장하는 콜백
    void region_gradient_callback(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        if (state_ != State::REGION_SCAN) {
            return;
        }
        const Eigen::Vector2d gradient(msg->vector.x, msg->vector.y);
        const double magnitude = gradient.norm(); // 그래디언트 크기
        const bool valid = gradient.allFinite() && std::isfinite(magnitude) && //유효성 검사
            magnitude > 1.0e-6;

        region_result_state_ = valid ?
            RegionResultState::VALID : RegionResultState::INVALID;
        if (valid) {
            region_gradient_ = gradient / magnitude; // 유효하면 그래디언트 방향 저장
        }
        RCLCPP_DEBUG(
            get_logger(), "REGION_SCAN gradient: valid=%s raw=(%.3f, %.3f) "
            "G_ref=(%.3f, %.3f)",
            valid ? "true" : "false", gradient.x(), gradient.y(),
            region_gradient_.x(), region_gradient_.y());
    }

    void rolling_gradient_callback(
        const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        if (state_ != State::REGION_HOMING || vision_search_requested_ ||
            rescan_requested_)
        {
            return;
        }
        const Eigen::Vector2d rolling(msg->vector.x, msg->vector.y);
        const double magnitude = rolling.norm();
        if (!rolling.allFinite() || magnitude <= 1.0e-6) {
            return;
        }
        const Eigen::Vector2d candidate = rolling / magnitude;
        const double angle = std::acos(std::clamp(
            homing_direction_.dot(candidate), -1.0, 1.0));
        if (angle > rolling_gradient_conflict_angle_rad_) {
            ++rolling_gradient_conflict_count_;
            RCLCPP_WARN(
                get_logger(),
                "[HOMING] rolling gradient conflict angle=%.1f deg (%zu/%zu)",
                angle * 180.0 / PI,
                rolling_gradient_conflict_count_,
                rolling_gradient_conflict_limit_);
            if (rolling_gradient_conflict_count_ >=
                rolling_gradient_conflict_limit_)
            {
                rescan_requested_ = true;
                RCLCPP_WARN(
                    get_logger(),
                    "[RESCAN] reason=rolling_gradient_conflict");
            }
            return;
        }

        rolling_gradient_conflict_count_ = 0;
        const Eigen::Vector2d blended =
            (1.0 - rolling_gradient_alpha_) * homing_direction_ +
            rolling_gradient_alpha_ * candidate;
        if (blended.norm() > 1.0e-6) {
            homing_direction_ = blended.normalized();
            publish_homing_direction();
        }
    }

    void target_confirmed_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (!msg->data || !vision_handoff_enabled_ ||
            !vision_search_requested_ || handoff_in_progress())
        {
            return;
        }
        begin_vision_handoff("target_confirmed");
    }

    bool handoff_in_progress() const
    {
        return state_ == State::HANDOFF_NEUTRAL ||
            state_ == State::HANDOFF_COMPLETE;
    }

    bool inside_vision_zone(const Eigen::Vector2d & position) const
    {
        if (vision_near_zone_width_m_ <= 0.0) {
            return false;
        }
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        const double width = std::min(
            vision_near_zone_width_m_, bounds.y_max - bounds.y_min);
        if (arena_start_corner_ == "bottom_left") {
            return position.y() <= bounds.y_min + width;
        }
        return position.y() >= bounds.y_max - width;
    }

    bool inside_vision_zone() const
    {
        return inside_vision_zone(current_position_);
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

    // near zone 진입 시 Vision에게 탐색만 요청하고 Acoustic 제어는 유지한다.
    void request_vision_confirmation()
    {
        if (!vision_handoff_enabled_ || vision_search_requested_ ||
            handoff_in_progress())
        {
            return;
        }
        vision_search_requested_ = true;
        publish_vision_control_granted(false);
        publish_vision_search_request(true);
        rolling_gradient_conflict_count_ = 0;
        publish_homing_direction();
        RCLCPP_INFO(
            get_logger(),
            "[VISION] search requested at position=(%.2f, %.2f); acoustic control kept",
            current_position_.x(), current_position_.y());
    }

    // 경계 직전 / confirm / timeout 시 중립 후 Vision에 제어권을 넘긴다.
    void begin_vision_handoff(const char * reason)
    {
        if (!vision_handoff_enabled_ || handoff_in_progress()) {
            return;
        }
        if (!vision_search_requested_) {
            vision_search_requested_ = true;
            publish_vision_control_granted(false);
            publish_vision_search_request(true);
            rolling_gradient_conflict_count_ = 0;
            publish_homing_direction();
        }
        handoff_neutral_sent_ = false;
        reset_yaw_pid();
        publish_rc(Command{});
        transition_to(State::HANDOFF_NEUTRAL);
        RCLCPP_INFO(
            get_logger(),
            "[VISION] acoustic handoff started reason=%s position=(%.2f, %.2f)",
            reason, current_position_.x(), current_position_.y());
    }

    void control_loop()
    {
        const rclcpp::Time current_time = now(); // 현재 시간을 가져온다.
        // [EMERGENCY NEUTRAL] 인계 상태와 무관하게 제어 채널 중립을 계속 발행한다.
        if (emergency_stop_active_.load()) {
            reset_yaw_pid();
            publish_rc(Command{});
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[EMERGENCY] neutral RC latched; restart required to resume");
            return;
        }
        // [ACOUSTIC-VISION HANDSHAKE] 중립을 한 주기 먼저 보낸 다음 Vision에 RC 소유권을 승인한다.
        if (state_ == State::HANDOFF_NEUTRAL) {
            if (!handoff_neutral_sent_) {
                publish_rc(Command{});
                handoff_neutral_sent_ = true;
                handoff_neutral_time_ = current_time;
                return;
            }
            if ((current_time - handoff_neutral_time_).seconds() >= 1.0 / rate_hz_) {
                transition_to(State::HANDOFF_COMPLETE);
                publish_vision_control_granted(true);
                RCLCPP_INFO(get_logger(), "[VISION] acoustic RC stopped; vision control granted");
            }
            return;
        }
        if (state_ == State::HANDOFF_COMPLETE) {
            depth_control_time_initialized_ = false;
            reset_yaw_pid();
            log_controller_status();
            return;
        }
        if (state_ == State::SUCCESS) {
            publish_rc(Command{});
            log_controller_status();
            return;
        }
        if (!odometry_is_fresh(current_time)) {
            depth_control_time_initialized_ = false;
            reset_yaw_pid();
            publish_rc(Command{});
            return;
        }
        log_controller_status();

        if (state_ == State::REGION_HOMING && acoustic_success_waiting_for_vision_) {
            publish_rc(depth_hold_command(current_time));
            return;
        }
        if (state_ == State::REGION_HOMING && success_reached(current_time)) {
            if (vision_handoff_enabled_) {
                acoustic_success_waiting_for_vision_ = true;
                request_vision_confirmation();
                // Acoustic still owns RC until target_confirmed. Hold position
                // and depth; begin_vision_handoff() sends the ordered neutral.
                publish_rc(depth_hold_command(current_time));
                RCLCPP_INFO(
                    get_logger(),
                    "[VISION] pinger SNR reached (median=%.2f dB); "
                    "waiting for visual target confirmation",
                    median_success_snr());
            } else {
                publish_rc(Command{});
                publish_success(true);
                transition_to(State::SUCCESS);
                RCLCPP_INFO(
                    get_logger(),
                    "[SUCCESS] pinger reached: median SNR=%.2f dB threshold=%.2f dB",
                    median_success_snr(), success_snr_db_);
            }
            return;
        }
        if (acoustic_deadline_expired(current_time)) {
            begin_vision_handoff("acoustic_timeout");
            return;
        }
        if (state_ == State::REGION_HOMING && vision_handoff_enabled_ &&
            inside_vision_zone())
        {
            request_vision_confirmation();
        }

        switch (state_) {
            case State::MOVE_TO_SCAN_CENTER:    //원형 탐색 중심 위치로 이동하기 위한 준비 상태
                if (follow_waypoint(current_time) && depth_target_reached()) {
                    begin_move_to_scan_start();
                }
                return;

            case State::MOVE_TO_SCAN_START:
                if (follow_waypoint(current_time)) {
                    begin_region_scan();
                }
                return;

            case State::REGION_SCAN:
                if (accumulated_scan_angle_rad_ < 2.0 * PI &&
                    !follow_circular_scan(current_time))
                {
                    return;
                }
                if (region_result_state_ == RegionResultState::WAITING) {
                    publish_rc(depth_hold_command(current_time));
                    return;
                }
                if (region_result_state_ == RegionResultState::INVALID) {
                    publish_rc(depth_hold_command(current_time));
                    RCLCPP_WARN(
                        get_logger(), "[RESCAN] reason=invalid_region_gradient");
                    start_new_region_scan(current_position_); // 현재 위치 기준으로 새로운 원형 탐색을 요청.
                    return;
                }
                begin_region_homing(); //원형 탐색 결과 그래디언트 유효성 판정 성공이므로 원형 탐색 결과 그래디언트 매칭 상태로 전이.
                return;

            case State::REGION_HOMING:
                if (rescan_requested_) {
                    start_new_region_scan(current_position_);
                    return;
                }
                if (!follow_waypoint(current_time)) {
                    return;
                }
                handle_homing_waypoint_result(make_next_homing_waypoint());
                return;

            case State::SUCCESS:
            case State::HANDOFF_NEUTRAL:
            case State::HANDOFF_COMPLETE:
                return;
        }
    }

    void start_new_region_scan(const Eigen::Vector2d & requested_center, const bool force = false)
    {
        if (handoff_in_progress()) {
            return;
        }
        if (vision_search_requested_) {
            publish_vision_search_request(false);
            vision_search_requested_ = false;
        }
        active_scan_radius_m_ = first_region_scan_ ?
            initial_scan_radius_m_ : rescan_radius_m_;
        first_region_scan_ = false;
        scan_center_ = adjusted_scan_center(requested_center, active_scan_radius_m_);
        publish_scan_center();
        rescan_requested_ = false;
        rolling_gradient_conflict_count_ = 0;
        waypoints_.clear();
        transition_to(State::MOVE_TO_SCAN_CENTER, force); // 원형 탐색 중심 위치로 이동 상태로 전이.
        set_current_waypoint(scan_center_); // 원형 탐색 중심 위치를 waypoint로 설정.
    }

    void begin_move_to_scan_start()
    {
        const Eigen::Vector2d scan_start =
            scan_center_ + active_scan_radius_m_ * Eigen::Vector2d::UnitX();
        if (!waypoint_is_safe(scan_start)) {
            throw std::logic_error(
                "adjusted region scan start left arena safety bounds");
        }
        transition_to(State::MOVE_TO_SCAN_START);
        set_current_waypoint(scan_start);
    }

    void begin_region_scan()
    {
        const Eigen::Vector2d radial = current_position_ - scan_center_;
        previous_scan_angle_rad_ = std::atan2(radial.y(), radial.x());
        accumulated_scan_angle_rad_ = 0.0;
        scan_radial_error_integral_ = 0.0;
        scan_radial_error_derivative_ = 0.0;
        scan_radial_pid_initialized_ = false;
        region_result_state_ = RegionResultState::WAITING;
        waypoint_heading_aligned_ = false;
        reset_yaw_pid();
        transition_to(State::REGION_SCAN);
        RCLCPP_INFO(
            get_logger(),
            "[SCAN] continuous circle started center=(%.2f, %.2f) radius=%.2f m",
            scan_center_.x(), scan_center_.y(), active_scan_radius_m_);
    }

    void begin_region_homing() // 원형 탐색 결과 그래디언트 매칭 상태로 전이
    {
        rescan_requested_ = false;
        rolling_gradient_conflict_count_ = 0;
        homing_direction_ = region_gradient_;
        homing_centerline_ = current_position_;
        zigzag_sign_ = 1.0;
        success_snr_window_.clear();
        success_candidate_active_ = false;
        transition_to(State::REGION_HOMING);
        publish_homing_direction();
        RCLCPP_INFO(
            get_logger(), "[SCAN] complete G_ref=(%.3f, %.3f)",
            region_gradient_.x(), region_gradient_.y());
        handle_homing_waypoint_result(make_next_homing_waypoint());
    }

    HomingWaypointResult make_next_homing_waypoint()
    {
        if (vision_search_requested_) {
            const Eigen::Vector2d waypoint =
                current_position_ + homing_waypoint_step_m_ * homing_direction_;
            if (!waypoint_is_safe(waypoint)) {
                return HomingWaypointResult::BOUNDARY;
            }
            waypoints_.assign(1, waypoint);
            return HomingWaypointResult::CREATED;
        }

        const Eigen::Vector2d next_centerline =
            homing_centerline_ + homing_waypoint_step_m_ * homing_direction_;
        const Eigen::Vector2d normal(
            -homing_direction_.y(), homing_direction_.x());
        const Eigen::Vector2d waypoint =
            next_centerline + zigzag_sign_ * homing_zigzag_offset_m_ * normal;

        if (vision_handoff_enabled_ && inside_vision_zone(waypoint)) {
            // near zone 안으로는 계속 들어가고, 안전경계 직전에서만 handoff 한다.
            request_vision_confirmation();
        }
        if (!waypoint_is_safe(waypoint)) {
            return HomingWaypointResult::BOUNDARY;
        }

        homing_centerline_ = next_centerline;
        waypoints_.assign(1, waypoint);
        zigzag_sign_ = -zigzag_sign_;
        return HomingWaypointResult::CREATED;
    }

    void handle_homing_waypoint_result(const HomingWaypointResult result)
    {
        if (result == HomingWaypointResult::CREATED) {
            set_current_waypoint(waypoints_.front());
            return;
        }
        if (result == HomingWaypointResult::VISION_ZONE) {
            request_vision_confirmation();
            handle_homing_boundary();
            return;
        }
        handle_homing_boundary();
    }

    void handle_homing_boundary()
    {
        // arena_safety_margin 안쪽 한계 = 경계 직전. near zone/handshake 중이면 Vision에 넘긴다.
        if (vision_handoff_enabled_ &&
            (vision_search_requested_ || inside_vision_zone()))
        {
            begin_vision_handoff("arena_boundary");
            return;
        }
        publish_rc(depth_hold_command(now()));
        RCLCPP_WARN(get_logger(), "[RESCAN] reason=arena_boundary");
        start_new_region_scan(current_position_);
    }

    bool follow_waypoint(const rclcpp::Time & current_time)
    {
        const Eigen::Vector2d delta = current_waypoint_ - current_position_; // 현재 위치와 waypoint 사이의 벡터
        const double distance = delta.norm(); // 현재 위치와 waypoint 사이의 거리
        if (distance <= waypoint_reach_tolerance_m_) {
            publish_rc(depth_hold_command(current_time));
            return true;
        }
        const double desired_yaw = std::atan2(delta.y(), delta.x()); // 현재 위치와 waypoint 사이의 벡터를 이용하여 원하는 yaw 값을 계산한다.
        const double yaw_error = wrap_pi(desired_yaw - current_yaw_rad_); // 현재 yaw와 원하는 yaw 사이의 오차.
        Command command = depth_hold_command(current_time); // 수평 이동과 목표 depth 제어를 동시에 수행한다.
        command.yaw = yaw_pid_command(yaw_error, current_time);
        const double heading_tolerance = vision_search_requested_ ?
            vision_heading_tolerance_rad_ : move_heading_tolerance_rad_;
        if (!waypoint_heading_aligned_ &&
            std::abs(yaw_error) <= heading_tolerance)
        {
            waypoint_heading_aligned_ = true;
        }
        if (waypoint_heading_aligned_ &&
            std::abs(yaw_error) >= REALIGN_HEADING_ERROR_RAD)
        {
            waypoint_heading_aligned_ = false;
        }
        if (waypoint_heading_aligned_) {
            command.forward = forward_cruise_;
        }
        publish_rc(command); // 명령을 보낸다.
        return false; // 아직 도착하지 않았으므로 false를 반환.
    }

    bool follow_circular_scan(const rclcpp::Time & current_time)
    {
        const Eigen::Vector2d radial = current_position_ - scan_center_;
        const double radius = radial.norm();
        if (radius <= 1.0e-6) {
            publish_rc(depth_hold_command(current_time));
            return false;
        }

        const double angle = std::atan2(radial.y(), radial.x());
        const double angle_delta = wrap_pi(angle - previous_scan_angle_rad_);
        // Only count forward CCW motion; brief reverse corrections must not
        // fake progress or subtract a large part of an otherwise clean lap.
        if (angle_delta >= 0.0 && angle_delta < 0.40) {
            accumulated_scan_angle_rad_ += angle_delta;
        }
        previous_scan_angle_rad_ = angle;
        if (accumulated_scan_angle_rad_ >= 2.0 * PI &&
            std::abs(radius - active_scan_radius_m_) <=
                scan_completion_radius_tolerance_m_)
        {
            RCLCPP_INFO(
                get_logger(),
                "[SCAN] continuous circle complete radius=%.2f/%.2f m",
                radius, active_scan_radius_m_);
            return true;
        }
        if (accumulated_scan_angle_rad_ >= 2.0 * PI) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "[SCAN] lap complete but radius=%.2f/%.2f m; correcting before homing",
                radius, active_scan_radius_m_);
        }

        // Track a short look-ahead point on the requested circle.  The
        // lateral RC term directly removes radial error while yaw aligns the
        // vehicle with the arc tangent, avoiding the outward spiral produced
        // by forward+yaw-only control.
        const double target_angle = angle + scan_lookahead_rad_;
        const Eigen::Vector2d target =
            scan_center_ + active_scan_radius_m_ *
            Eigen::Vector2d(std::cos(target_angle), std::sin(target_angle));
        const Eigen::Vector2d error_world = target - current_position_;
        const double desired_yaw = std::atan2(error_world.y(), error_world.x());
        const double yaw_error = wrap_pi(desired_yaw - current_yaw_rad_);
        const double cosine = std::cos(current_yaw_rad_);
        const double sine = std::sin(current_yaw_rad_);
        const double error_body_x =
            cosine * error_world.x() + sine * error_world.y();
        const double error_body_y =
            -sine * error_world.x() + cosine * error_world.y();

        Command command = depth_hold_command(current_time);
        command.yaw = yaw_pid_command(yaw_error, current_time);
        command.sway = std::clamp(
            scan_sway_gain_ * error_body_y,
            -scan_sway_limit_, scan_sway_limit_);
        if (std::abs(yaw_error) <= scan_heading_gate_rad_) {
            command.forward = std::clamp(
                scan_xy_gain_ * error_body_x, 0.08, forward_cruise_);
        }
        publish_rc(command);
        return false;
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
            yaw_kp_ * yaw_error +
            yaw_ki_ * candidate_integral +
            yaw_kd_ * yaw_error_derivative_;
        if (std::abs(candidate_command) <= yaw_limit_ ||
            candidate_command * yaw_error < 0.0)
        {
            yaw_error_integral_ = candidate_integral;
        }

        last_yaw_error_ = yaw_error;
        last_yaw_command_ = std::clamp(
            yaw_kp_ * yaw_error +
            yaw_ki_ * yaw_error_integral_ +
            yaw_kd_ * yaw_error_derivative_,
            -yaw_limit_, yaw_limit_);
        return last_yaw_command_;
    }

    void reset_yaw_pid()
    {
        yaw_pid_initialized_ = false;
        previous_yaw_error_ = 0.0;
        yaw_error_integral_ = 0.0;
        yaw_error_derivative_ = 0.0;
        last_yaw_error_ = 0.0;
        last_yaw_command_ = 0.0;
    }

    Command depth_hold_command(const rclcpp::Time & current_time)
    {
        Command command;
        const double error = target_depth_z_m_ - current_z_m_;

        double dt = 0.0;
        if (depth_control_time_initialized_) {
            dt = std::clamp(
                (current_time - last_depth_control_time_).seconds(), 0.0, 0.2);
        }
        last_depth_control_time_ = current_time;
        depth_control_time_initialized_ = true;

        const double candidate_integral = depth_error_integral_ + error * dt;
        const double candidate_heave = -(
            depth_kp_ * error + depth_ki_ * candidate_integral);
        if (std::abs(candidate_heave) <= HEAVE_LIMIT ||
            (candidate_heave > HEAVE_LIMIT && error > 0.0) ||
            (candidate_heave < -HEAVE_LIMIT && error < 0.0))
        {
            depth_error_integral_ = candidate_integral;
        }
        command.heave = std::clamp(
            -(depth_kp_ * error + depth_ki_ * depth_error_integral_),
            -HEAVE_LIMIT, HEAVE_LIMIT);
        return command;
    }

    bool depth_target_reached() const
    {
        return std::abs(target_depth_z_m_ - current_z_m_) <= depth_tolerance_m_;
    }

    double median_success_snr() const
    {
        if (success_snr_window_.empty()) {
            return -std::numeric_limits<double>::infinity();
        }
        std::vector<double> values(
            success_snr_window_.begin(), success_snr_window_.end());
        const std::size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        double value = values[middle];
        if (values.size() % 2 == 0) {
            const auto lower =
                std::max_element(values.begin(), values.begin() + middle);
            value = 0.5 * (value + *lower);
        }
        return value;
    }

    bool success_reached(const rclcpp::Time & current_time)
    {
        if (!have_snr_ ||
            (current_time - last_snr_receive_time_).seconds() >
                success_snr_timeout_s_ ||
            success_snr_window_.size() < 5 ||
            median_success_snr() < success_snr_db_)
        {
            success_candidate_active_ = false;
            return false;
        }
        if (!success_candidate_active_) {
            success_candidate_active_ = true;
            success_candidate_start_time_ = current_time;
            return false;
        }
        return (current_time - success_candidate_start_time_).seconds() >=
            success_hold_s_;
    }

    void log_controller_status()
    {
        if (state_ == State::SUCCESS) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[CONTROL] state=SUCCESS median_snr=%.2f dB position=(%.2f, %.2f)",
                median_success_snr(), current_position_.x(), current_position_.y());
            return;
        }
        if (state_ == State::HANDOFF_NEUTRAL ||
            state_ == State::HANDOFF_COMPLETE)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[CONTROL] state=%s acoustic_control=inactive",
                state_name(state_));
            return;
        }
        if (state_ == State::REGION_SCAN) {
            const double radius = (current_position_ - scan_center_).norm();
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[CONTROL] state=REGION_SCAN progress=%.0f/360 deg "
                "radius=%.2f/%.2f m yaw_error=%.1f deg yaw_cmd=%.2f "
                "z=%.2f/%.2f m",
                std::clamp(
                    accumulated_scan_angle_rad_ * 180.0 / PI, 0.0, 360.0),
                radius, active_scan_radius_m_,
                last_yaw_error_ * 180.0 / PI, last_yaw_command_,
                current_z_m_, target_depth_z_m_);
            return;
        }
        const double distance = (current_waypoint_ - current_position_).norm();
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[CONTROL] state=%s distance=%.2f m "
            "yaw_error=%.1f deg yaw_cmd=%.2f z=%.2f/%.2f m",
            state_name(state_), distance,
            last_yaw_error_ * 180.0 / PI, last_yaw_command_,
            current_z_m_, target_depth_z_m_);
    }

    Eigen::Vector2d adjusted_scan_center(
        const Eigen::Vector2d & position, const double radius) const // 원형 탐색 중심 위치를 실험장 안쪽으로 조정하는 함수
    {
        Eigen::Vector2d center = position;
        const double inset = arena_safety_margin_m_ + radius; // 중심이 벽에서 최소 이만큼 떨어져야 반경 원이 여유공간을 안 뚫음
        const ArenaBounds bounds = arena_bounds(inset);
        center.x() = std::clamp(center.x(), bounds.x_min, bounds.x_max);
        center.y() = std::clamp(center.y(), bounds.y_min, bounds.y_max);
        return center; //보정된 중심 위치를 반환.
    }

    Eigen::Vector2d arena_center() const
    {
        const ArenaBounds bounds = arena_bounds(0.0);
        return {
            0.5 * (bounds.x_min + bounds.x_max),
            0.5 * (bounds.y_min + bounds.y_max)};
    }


    // 원형 탐색 경로가 실험장 안쪽에 있는지 판정하는 함수
    bool waypoint_is_safe(const Eigen::Vector2d & waypoint) const
    {
        const ArenaBounds bounds = arena_bounds(arena_safety_margin_m_);
        return waypoint.x() >= bounds.x_min && waypoint.x() <= bounds.x_max &&
            waypoint.y() >= bounds.y_min && waypoint.y() <= bounds.y_max;
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

    void set_current_waypoint(const Eigen::Vector2d & waypoint)
    {
        current_waypoint_ = waypoint;
        waypoint_heading_aligned_ = false;
        reset_yaw_pid();
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = arena_frame_id_;
        msg.point.x = waypoint.x();
        msg.point.y = waypoint.y();
        waypoint_pub_->publish(msg);
    }

    void publish_scan_center()
    {
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = arena_frame_id_;
        msg.point.x = scan_center_.x();
        msg.point.y = scan_center_.y();
        scan_center_pub_->publish(msg);
    }

    void transition_to(const State next_state, const bool force = false)
    {
        if (!force && state_ == next_state) {
            return;
        }
        const State previous_state = state_;
        state_ = next_state;
        publish_state();
        RCLCPP_INFO(
            get_logger(), "[STATE] %s -> %s",
            state_name(previous_state), state_name(state_));
    }

    void publish_state()
    {
        std_msgs::msg::String msg;
        msg.data = state_name(state_);
        state_pub_->publish(msg);
    }

    void publish_success(const bool success)
    {
        std_msgs::msg::Bool msg;
        msg.data = success;
        success_pub_->publish(msg);
    }

    void publish_vision_search_request(const bool active)
    {
        std_msgs::msg::Bool msg;
        msg.data = active;
        vision_search_request_pub_->publish(msg);
    }

    // [ACOUSTIC-VISION HANDSHAKE] Vision RC 발행을 허용하는 최종 승인이다.
    void publish_vision_control_granted(const bool granted)
    {
        std_msgs::msg::Bool msg;
        msg.data = granted;
        vision_control_granted_pub_->publish(msg);
    }

    void publish_homing_direction()
    {
        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = arena_frame_id_;
        msg.vector.x = homing_direction_.x();
        msg.vector.y = homing_direction_.y();
        homing_direction_pub_->publish(msg);

        const double cosine = std::cos(current_yaw_rad_);
        const double sine = std::sin(current_yaw_rad_);
        geometry_msgs::msg::Vector3Stamped body;
        body.header.stamp = msg.header.stamp;
        body.header.frame_id = "base_link";
        body.vector.x =
            cosine * homing_direction_.x() + sine * homing_direction_.y();
        body.vector.y =
            -sine * homing_direction_.x() + cosine * homing_direction_.y();
        direction_body_pub_->publish(body);
    }

    void publish_rc(const Command & command)
    {
        mavros_msgs::msg::OverrideRCIn msg;
        msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        // [RC OWNERSHIP] 이 제어기가 실제 사용하는 축만 override한다.
        msg.channels[VERTICAL_CHANNEL_INDEX] = axis_pwm(command.heave, true);
        msg.channels[YAW_CHANNEL_INDEX] = axis_pwm(command.yaw, invert_rc_yaw_);
        msg.channels[FORWARD_CHANNEL_INDEX] = axis_pwm(command.forward, false);
        msg.channels[LATERAL_CHANNEL_INDEX] =
            axis_pwm(command.sway, invert_rc_lateral_);
        rc_pub_->publish(msg);
    }

    void emergency_stop_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (msg->data) {
            emergency_stop_active_.store(true);
        }
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
        if (tcgetattr(terminal_fd, &original_termios) == 0)
        {
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
            timeval timeout{0, 100000};
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

    bool odometry_is_fresh(const rclcpp::Time & current_time) const
    {
        return have_odometry_ &&
            (current_time - last_odometry_receive_time_).seconds() <= odometry_timeout_s_;
    }

    static double wrap_pi(const double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    static const char * state_name(const State state)
    {
        switch (state) {
            case State::MOVE_TO_SCAN_CENTER:
                return "MOVE_TO_SCAN_CENTER";
            case State::MOVE_TO_SCAN_START:
                return "MOVE_TO_SCAN_START";
            case State::REGION_SCAN:
                return "REGION_SCAN";
            case State::REGION_HOMING:
                return "REGION_HOMING";
            case State::SUCCESS:
                return "SUCCESS";
            case State::HANDOFF_NEUTRAL:
                return "HANDOFF_NEUTRAL";
            case State::HANDOFF_COMPLETE:
                return "HANDOFF_COMPLETE";
        }
        return "MOVE_TO_SCAN_CENTER";
    }

    double arena_length_m_ = 15.0;
    double arena_width_m_ = 16.0;
    double arena_offset_x_m_ = 0.0;
    double arena_offset_y_m_ = 0.0;
    double arena_safety_margin_m_ = 0.5;
    double initial_scan_radius_m_ = 1.5;
    double rescan_radius_m_ = 0.7;
    double active_scan_radius_m_ = 1.5;
    double homing_waypoint_step_m_ = 0.8;
    double homing_zigzag_offset_m_ = 0.2;
    double rolling_gradient_alpha_ = 0.15;
    double rolling_gradient_conflict_angle_rad_ = PI / 3.0;
    double waypoint_reach_tolerance_m_ = 0.15;
    double scan_radial_kp_ = 1.5;
    double scan_radial_ki_ = 0.05;
    double scan_radial_kd_ = 0.3;
    double scan_radial_integral_limit_ = 1.0;
    double scan_lookahead_rad_ = 0.30;
    double scan_xy_gain_ = 0.85;
    double scan_sway_gain_ = 1.10;
    double scan_sway_limit_ = 0.45;
    double scan_heading_gate_rad_ = 0.50;
    double scan_completion_radius_tolerance_m_ = 0.30;
    double vision_near_zone_width_m_ = 2.0;
    double acoustic_timeout_s_ = 90.0;
    double success_snr_db_ = 70.0;
    double success_hold_s_ = 1.0;
    double success_snr_timeout_s_ = 1.0;
    double target_depth_z_m_ = -0.65;
    double depth_tolerance_m_ = 0.10;
    double depth_kp_ = 0.8;
    double depth_ki_ = 0.15;
    double rate_hz_ = 30.0;
    double odometry_timeout_s_ = 0.5;
    double forward_cruise_ = 0.5;
    double yaw_kp_ = 1.15;
    double yaw_ki_ = 0.15;
    double yaw_kd_ = 0.08;
    double yaw_integral_limit_ = 2.0;
    double yaw_limit_ = 0.72;
    double move_heading_tolerance_rad_ = 0.1745;
    double vision_heading_tolerance_rad_ = 0.12;
    double rc_pwm_span_ = 400.0;
    std::size_t rolling_gradient_conflict_limit_ = 3;
    std::size_t rolling_gradient_conflict_count_ = 0;
    std::size_t success_snr_window_size_ = 15;
    double zigzag_sign_ = 1.0;
    std::string arena_start_corner_ = "bottom_left";
    std::string arena_frame_id_ = "arena";
    bool invert_rc_yaw_ = true;
    bool invert_rc_lateral_ = false;
    bool vision_handoff_enabled_ = true;
    bool use_explicit_initial_scan_center_ = false;
    bool acoustic_deadline_started_ = false;
    bool have_odometry_ = false;
    bool waypoint_heading_aligned_ = false;
    bool rescan_requested_ = false;
    bool vision_search_requested_ = false;
    bool acoustic_success_waiting_for_vision_ = false;
    bool first_region_scan_ = true;
    bool depth_control_time_initialized_ = false;
    bool yaw_pid_initialized_ = false;
    bool scan_radial_pid_initialized_ = false;
    bool handoff_neutral_sent_ = false;
    bool have_snr_ = false;
    bool success_candidate_active_ = false;
    std::string emergency_stop_key_ = "s";
    State state_ = State::MOVE_TO_SCAN_CENTER;
    RegionResultState region_result_state_ = RegionResultState::WAITING;
    rclcpp::Time last_odometry_receive_time_;
    rclcpp::Time last_depth_control_time_;
    rclcpp::Time last_yaw_control_time_;
    rclcpp::Time last_scan_radial_control_time_;
    rclcpp::Time handoff_neutral_time_;
    rclcpp::Time acoustic_start_time_;
    rclcpp::Time last_snr_receive_time_;
    rclcpp::Time success_candidate_start_time_;
    double current_yaw_rad_ = 0.0;
    double current_z_m_ = 0.0;
    double depth_error_integral_ = 0.0;
    double previous_yaw_error_ = 0.0;
    double yaw_error_integral_ = 0.0;
    double yaw_error_derivative_ = 0.0;
    double last_yaw_error_ = 0.0;
    double last_yaw_command_ = 0.0;
    double previous_scan_angle_rad_ = 0.0;
    double accumulated_scan_angle_rad_ = 0.0;
    double previous_scan_radial_error_ = 0.0;
    double scan_radial_error_integral_ = 0.0;
    double scan_radial_error_derivative_ = 0.0;
    double initial_scan_center_x_m_ = 0.0;
    double initial_scan_center_y_m_ = 0.0;
    double latest_snr_db_ = -std::numeric_limits<double>::infinity();
    Eigen::Vector2d current_position_{0.0, 0.0};
    Eigen::Vector2d scan_center_{0.0, 0.0};
    Eigen::Vector2d current_waypoint_{0.0, 0.0};
    Eigen::Vector2d homing_centerline_{0.0, 0.0};
    Eigen::Vector2d region_gradient_{1.0, 0.0};
    Eigen::Vector2d homing_direction_{1.0, 0.0};
    std::vector<Eigen::Vector2d> waypoints_;
    std::deque<double> success_snr_window_;
    hydrophone_ctrl::ArenaFrameTransform arena_transform_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        start_frame_sub_;
    rclcpp::Subscription<audio_common_msgs::msg::Float64Stamped>::SharedPtr
        snr_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        region_gradient_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        rolling_gradient_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_confirmed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr scan_center_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_search_request_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_control_granted_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        homing_direction_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        direction_body_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic_bool emergency_stop_active_{false};
    std::atomic_bool stop_keyboard_thread_{false};
    std::thread keyboard_thread_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::WaypointHomingControllerNode)
