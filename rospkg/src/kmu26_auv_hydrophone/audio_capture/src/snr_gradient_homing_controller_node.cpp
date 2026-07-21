#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <deque>
#include <string>
#include <utility>

#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace audio_capture
{
// SNR gradient 전용 제어 흐름:
//   DISABLED
//      enable=true
//          -> INITIAL_DIAGONAL: 모서리에서 경기장 중심 방향으로 대각선 이동
//          -> INITIAL_SEARCH: 전진+yaw 명령으로 원형/호 궤적 생성
//          -> VERTICAL_SEARCH: SNR 10 dB 지속 시 위·아래 sweep 후 최고 SNR z로 이동
//          -> HOMING: 유효한 방향과 신뢰도가 일정 시간 유지되면 음원 방향 추종
//          -> DISABLED: 방향을 일정 시간 잃으면 자동 재탐색 없이 안전하게 제어 해제
class SnrGradientHomingControllerNode : public rclcpp::Node
{
public:
    // [제어 노드 초기화] homing 입력, 안전 파라미터, 상태기계, RC 출력과 주기 timer를 구성한다.
    explicit SnrGradientHomingControllerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("snr_gradient_homing_controller", options)
    {
        // ROS 입출력 설정.
        direction_topic_ =
            declare_parameter<std::string>("direction_topic", "/homing/direction");
        confidence_topic_ =
            declare_parameter<std::string>("confidence_topic", "/homing/snr_confidence");
        enable_topic_ =
            declare_parameter<std::string>("enable_topic", "/homing/control_enable");
        state_topic_ =
            declare_parameter<std::string>("state_topic", "/homing/control_state");
        estimator_ready_topic_ =
            declare_parameter<std::string>("estimator_ready_topic", "/homing/estimator_ready");
        reset_topic_ = declare_parameter<std::string>("reset_topic", "/homing/reset_estimator");
        odometry_topic_ =
            declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
        rc_override_topic_ =
            declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
        rc_preview_topic_ =
            declare_parameter<std::string>("rc_preview_topic", "/homing/rc_override_preview");
        required_direction_frame_ =
            declare_parameter<std::string>("required_direction_frame", "base_link");

        // 상태 전환과 방향 유효성 설정.
        control_enabled_ = declare_parameter<bool>("control_enabled", false);
        dry_run_ = declare_parameter<bool>("dry_run", true);
        rate_hz_ = clamp(declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
        direction_timeout_s_ =
            clamp(declare_parameter<double>("direction_timeout_s", 0.8), 0.05, 10.0);
        confidence_timeout_s_ =
            clamp(declare_parameter<double>("confidence_timeout_s", 0.8), 0.05, 10.0);
        min_direction_confidence_ =
            clamp(declare_parameter<double>("min_direction_confidence", 0.15), 0.0, 1.0);
        acquire_hold_s_ =
            clamp(declare_parameter<double>("acquire_hold_s", 1.0), 0.0, 30.0);
        search_min_duration_s_ =
            clamp(declare_parameter<double>("search_min_duration_s", 6.0), 0.0, 120.0);
        direction_loss_hold_s_ =
            clamp(declare_parameter<double>("direction_loss_hold_s", 1.0), 0.0, 30.0);
        direction_stability_window_s_ = clamp(
            declare_parameter<double>("direction_stability_window_s", 1.0), 0.1, 10.0);
        max_direction_std_rad_ = clamp(
            declare_parameter<double>("max_direction_std_rad", 0.30), 0.01, PI);
        estimator_ready_timeout_s_ = clamp(
            declare_parameter<double>("estimator_ready_timeout_s", 1.0), 0.05, 10.0);

        // 경기장 좌표와 모서리 이탈 대각선 이동 설정.
        arena_width_m_ = std::max(
            0.1, declare_parameter<double>("arena_width_m", 15.0));
        arena_height_m_ = std::max(
            0.1, declare_parameter<double>("arena_height_m", 16.0));
        arena_start_corner_ =
            declare_parameter<std::string>("arena_start_corner", "bottom_left");
        if (arena_start_corner_ != "bottom_left" &&
            arena_start_corner_ != "bottom_right")
        {
            RCLCPP_WARN(
                get_logger(),
                "Unknown arena_start_corner '%s'; using bottom_left.",
                arena_start_corner_.c_str());
            arena_start_corner_ = "bottom_left";
        }
        arena_yaw_rad_ = declare_parameter<double>("arena_yaw_rad", 0.0);
        arena_start_inset_m_ = std::clamp(
            declare_parameter<double>("arena_start_inset_m", 0.12),
            0.0,
            0.49 * std::min(arena_width_m_, arena_height_m_));
        geofence_margin_m_ = std::clamp(
            declare_parameter<double>("geofence_margin_m", 0.20),
            0.0,
            0.49 * std::min(arena_width_m_, arena_height_m_));
        initial_diagonal_distance_m_ = clamp(
            declare_parameter<double>("initial_diagonal_distance_m", 0.70), 0.0, 20.0);
        initial_diagonal_command_ = clamp(
            declare_parameter<double>("initial_diagonal_command", 0.30), 0.0, 1.0);
        initial_diagonal_timeout_s_ = clamp(
            declare_parameter<double>("initial_diagonal_timeout_s", 10.0), 0.1, 120.0);
        odometry_timeout_s_ = clamp(
            declare_parameter<double>("odometry_timeout_s", 0.50), 0.05, 10.0);
        vertical_search_distance_m_ = clamp(
            declare_parameter<double>("vertical_search_distance_m", 0.50), 0.05, 5.0);
        vertical_search_min_z_m_ =
            declare_parameter<double>("vertical_search_min_z_m", -1.30);
        vertical_search_max_z_m_ =
            declare_parameter<double>("vertical_search_max_z_m", -0.20);
        if (vertical_search_min_z_m_ > vertical_search_max_z_m_) {
            std::swap(vertical_search_min_z_m_, vertical_search_max_z_m_);
        }

        // 초기 원형 탐색 설정. 부호는 실제 기체 yaw 채널 방향에 맞춘다.
        search_forward_ =
            clamp(declare_parameter<double>("search_forward", 0.30), 0.0, 1.0);
        search_yaw_ =
            clamp(declare_parameter<double>("search_yaw", 0.30), 0.0, 1.0);
        search_turn_sign_ =
            declare_parameter<double>("search_turn_sign", 1.0) < 0.0 ? -1.0 : 1.0;

        // 방향 추종 설정. direction은 base_link 기준 단위 벡터를 기대한다.
        forward_fast_ = clamp(declare_parameter<double>("forward_fast", 0.70), 0.0, 1.0);
        forward_mid_ = clamp(declare_parameter<double>("forward_mid", 0.45), 0.0, 1.0);
        forward_slow_ = clamp(declare_parameter<double>("forward_slow", 0.20), 0.0, 1.0);
        yaw_gain_ = clamp(declare_parameter<double>("yaw_gain", 1.15), 0.0, 4.0);
        yaw_limit_ = clamp(declare_parameter<double>("yaw_limit", 0.72), 0.0, 1.0);
        heave_gain_ = clamp(declare_parameter<double>("heave_gain", 0.42), 0.0, 2.0);
        heave_limit_ = clamp(declare_parameter<double>("heave_limit", 0.38), 0.0, 1.0);
        center_deadband_rad_ =
            clamp(declare_parameter<double>("center_deadband_rad", 0.055), 0.0, 0.50);
        confidence_speed_floor_ =
            clamp(declare_parameter<double>("confidence_speed_floor", 0.40), 0.0, 1.0);

        // RC 채널 변환 설정.
        rc_pwm_span_ = clamp(declare_parameter<double>("rc_pwm_span", 400.0), 50.0, 700.0);
        invert_rc_heave_ = declare_parameter<bool>("invert_rc_heave", true);
        invert_rc_yaw_ = declare_parameter<bool>("invert_rc_yaw", true);
        invert_rc_lateral_ = declare_parameter<bool>("invert_rc_lateral", false);

        direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            direction_topic_,
            10,
            std::bind(
                &SnrGradientHomingControllerNode::direction_callback,
                this,
                std::placeholders::_1));
        confidence_sub_ = create_subscription<std_msgs::msg::Float64>(
            confidence_topic_,
            10,
            std::bind(
                &SnrGradientHomingControllerNode::confidence_callback,
                this,
                std::placeholders::_1));
        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            enable_topic_,
            10,
            std::bind(
                &SnrGradientHomingControllerNode::enable_callback,
                this,
                std::placeholders::_1));
        estimator_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
            estimator_ready_topic_, 10,
            std::bind(&SnrGradientHomingControllerNode::estimator_ready_callback, this, std::placeholders::_1));
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_, 20,
            std::bind(&SnrGradientHomingControllerNode::odometry_callback, this, std::placeholders::_1));
        vertical_search_request_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/homing/vertical_search_request",
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(
                &SnrGradientHomingControllerNode::vertical_search_request_callback,
                this,
                std::placeholders::_1));
        vertical_best_z_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/homing/vertical_best_z",
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(
                &SnrGradientHomingControllerNode::vertical_best_z_callback,
                this,
                std::placeholders::_1));

        rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);
        rc_preview_pub_ =
            create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_preview_topic_, 10);
        state_pub_ = create_publisher<std_msgs::msg::String>(
            state_topic_, rclcpp::QoS(1).reliable().transient_local());
        reset_pub_ = create_publisher<std_msgs::msg::Empty>(reset_topic_, 10);
        vertical_search_active_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/homing/vertical_search_active",
            rclcpp::QoS(1).reliable().transient_local());
        publish_vertical_search_active(false);

        state_ = State::DISABLED;
        state_enter_time_ = now();
        if (control_enabled_) {
            transition_to(State::INITIAL_DIAGONAL);
        } else {
            publish_state();
        }

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SnrGradientHomingControllerNode::control_loop, this));

        RCLCPP_INFO(
            get_logger(),
            "SNR gradient controller ready. enabled=%s dry_run=%s direction=%s frame=%s rc=%s preview=%s",
            control_enabled_ ? "true" : "false",
            dry_run_ ? "true" : "false",
            direction_topic_.c_str(),
            required_direction_frame_.c_str(),
            rc_override_topic_.c_str(),
            rc_preview_topic_.c_str());
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr std::uint16_t RC_NEUTRAL = 1500;
    static constexpr std::size_t PITCH_CHANNEL_INDEX = 0;
    static constexpr std::size_t ROLL_CHANNEL_INDEX = 1;
    static constexpr std::size_t VERTICAL_CHANNEL_INDEX = 2;
    static constexpr std::size_t YAW_CHANNEL_INDEX = 3;
    static constexpr std::size_t FORWARD_CHANNEL_INDEX = 4;
    static constexpr std::size_t LATERAL_CHANNEL_INDEX = 5;
    static constexpr std::size_t PRIMARY_CHANNEL_COUNT = 8;
    static constexpr double VERTICAL_TARGET_TOLERANCE_M = 0.05;
    static constexpr double VERTICAL_SEARCH_TIMEOUT_S = 40.0;

    enum class State
    {
        DISABLED,
        INITIAL_DIAGONAL,
        INITIAL_SEARCH,
        VERTICAL_SEARCH,
        HOMING
    };

    enum class VerticalSearchPhase
    {
        MOVE_UP,
        MOVE_DOWN,
        MOVE_BEST
    };

    struct Command
    {
        double forward = 0.0;
        double sway = 0.0;
        double heave = 0.0;
        double yaw = 0.0;
    };

    // [Odometry 수신] 경기장 기준 모서리, 현재 위치·yaw와 초기 대각선 이동거리를 갱신한다.
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        current_odometry_xy_ = Eigen::Vector2d(
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        current_odometry_z_m_ = msg->pose.pose.position.z;
        current_yaw_rad_ = yaw_from_quaternion(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        last_odometry_receive_time_ = now();
        have_odometry_ = true;
        if (!have_arena_corner_) {
            arena_corner_xy_ = current_odometry_xy_;
            have_arena_corner_ = true;
        }
        if (state_ == State::INITIAL_DIAGONAL && !have_initial_diagonal_start_) {
            capture_initial_diagonal_start();
        }
    }

    // [Homing 방향 수신] frame과 수치를 검증해 단위 벡터로 저장하고 방향각 안정성 이력을 갱신한다.
    void direction_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
    {
        // 좌표계가 다르면 body-frame 제어 입력으로 사용할 수 없다.
        if (!required_direction_frame_.empty() &&
            msg->header.frame_id != required_direction_frame_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Direction frame '%s' does not match required frame '%s'.",
                msg->header.frame_id.c_str(),
                required_direction_frame_.c_str());
            return;
        }

        const double x = msg->vector.x;
        const double y = msg->vector.y;
        const double z = msg->vector.z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            return;
        }

        const double norm = std::sqrt(x * x + y * y + z * z);
        if (norm < 1.0e-6) {
            return;
        }

        direction_x_ = x / norm;
        direction_y_ = y / norm;
        direction_z_ = z / norm;
        last_direction_time_ = now();
        have_direction_ = true;
        bearing_history_.push_back({last_direction_time_, std::atan2(direction_y_, direction_x_)});
        while (!bearing_history_.empty() &&
            (last_direction_time_ - bearing_history_.front().first).seconds() > direction_stability_window_s_)
        {
            bearing_history_.pop_front();
        }
    }

    // [방향 신뢰도 수신] 유효한 값을 0~1로 제한하고 freshness 판정용 수신 시각을 저장한다.
    void confidence_callback(const std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        if (!std::isfinite(msg->data)) {
            return;
        }
        direction_confidence_ = clamp(msg->data, 0.0, 1.0);
        last_confidence_time_ = now();
        have_confidence_ = true;
    }

    // [제어 enable 수신] 활성화하면 모서리 이탈 대각선 이동, 비활성화하면 RC 해제로 전환한다.
    void enable_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (msg->data == control_enabled_) {
            return;
        }

        control_enabled_ = msg->data;
        have_acquire_start_ = false;
        have_last_valid_signal_time_ = false;
        if (control_enabled_) {
            // enable 상승 시 항상 모서리 이탈 대각선 이동부터 새 순서를 시작한다.
            transition_to(State::INITIAL_DIAGONAL);
        } else {
            transition_to(State::DISABLED);
        }
    }

    // [추정기 준비 상태 수신] V2 robust gradient 방향 사용 가능 여부와 최신 수신 시각을 기록한다.
    void estimator_ready_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        estimator_ready_ = msg->data;
        last_estimator_ready_time_ = now();
    }

    // [수직 탐색 요청 수신] V2의 one-shot SNR trigger 상태를 저장한다.
    void vertical_search_request_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        vertical_search_requested_ = msg->data;
    }

    // [최고 SNR z 수신] sweep 중 V2가 추정한 최고 SNR odometry z를 저장한다.
    void vertical_best_z_callback(const std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        if (!std::isfinite(msg->data)) {
            return;
        }
        vertical_best_z_m_ = clamp(
            msg->data, vertical_search_min_z_m_, vertical_search_max_z_m_);
        have_vertical_best_z_ = true;
    }

    // [폐루프 상태기계 실행] 비활성·초기 탐색·수직 sweep·homing 상태의 RC 명령을 결정한다.
    void control_loop()
    {
        const rclcpp::Time current_time = now();

        // 비활성 상태에서는 RC override를 해제하여 수동 조종을 방해하지 않는다.
        if (!control_enabled_ || state_ == State::DISABLED) {
            publish_rc(make_release_override());
            return;
        }

        // 실행 분기 A: 시작 모서리에서 경기장 중심 방향으로 목표 거리만큼 대각선 이동한다.
        if (state_ == State::INITIAL_DIAGONAL) {
            double traveled_m = 0.0;
            if (have_initial_diagonal_start_ && odometry_is_fresh(current_time)) {
                traveled_m = (current_odometry_xy_ - initial_diagonal_start_xy_).norm();
            }
            if (have_initial_diagonal_start_ && odometry_is_fresh(current_time) &&
                traveled_m >= initial_diagonal_distance_m_)
            {
                // 대각선 구간에서 누적한 SNR map을 유지한 채 원형 탐색을 시작한다.
                publish_rc(make_rc_override(Command{}));
                transition_to(State::INITIAL_SEARCH);
                return;
            }
            const rclcpp::Time timeout_start =
                have_initial_diagonal_start_ ? initial_diagonal_start_time_ : state_enter_time_;
            if ((current_time - timeout_start).seconds() >= initial_diagonal_timeout_s_)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Initial diagonal timed out at %.2f/%.2f m; control disabled.",
                    traveled_m,
                    initial_diagonal_distance_m_);
                control_enabled_ = false;
                transition_to(State::DISABLED);
                publish_rc(make_release_override());
                return;
            }
            if (!have_initial_diagonal_start_ || !odometry_is_fresh(current_time)) {
                publish_rc(make_rc_override(Command{}));
                return;
            }
            publish_rc(make_rc_override(initial_diagonal_command()));
            return;
        }

        // 경기장 경계 clamp에 필요한 odometry가 오래되면 모든 이동축을 중립으로 둔다.
        if (!odometry_is_fresh(current_time)) {
            publish_rc(make_rc_override(Command{}));
            return;
        }

        if (state_ == State::VERTICAL_SEARCH) {
            update_vertical_search(current_time);
            return;
        }

        const bool signal_valid = have_valid_signal(current_time);

        // 실행 분기 B: 미션 시작 후 최초 방향을 얻기 위한 원형 탐색.
        if (state_ == State::INITIAL_SEARCH) {
            if (vertical_search_requested_ && !vertical_search_completed_) {
                transition_to(State::VERTICAL_SEARCH);
                publish_rc(make_rc_override(Command{}));
                return;
            }
            update_search_acquisition(current_time, signal_valid);
            if (state_ == State::HOMING) {
                publish_rc(make_rc_override(homing_command()));
            } else {
                publish_rc(make_rc_override(search_command()));
            }
            return;
        }

        // 실행 분기 C: 유효한 방향을 따라 음원 쪽으로 전진한다.
        if (state_ == State::HOMING && signal_valid) {
            last_valid_signal_time_ = current_time;
            have_last_valid_signal_time_ = true;
            publish_rc(make_rc_override(homing_command()));
            return;
        }

        // 실행 분기 D: 방향 손실 동안 정지하고 hold 이후 자동 재탐색 없이 제어를 해제한다.
        publish_rc(make_rc_override(Command{}));
        if (!have_last_valid_signal_time_) {
            last_valid_signal_time_ = current_time;
            have_last_valid_signal_time_ = true;
        }
        if ((current_time - last_valid_signal_time_).seconds() >= direction_loss_hold_s_) {
            RCLCPP_ERROR(
                get_logger(),
                "Homing direction lost for %.2f s; control disabled.",
                direction_loss_hold_s_);
            control_enabled_ = false;
            transition_to(State::DISABLED);
            publish_rc(make_release_override());
        }
    }

    // [수직 Sweep 실행] 위→아래→최고 SNR z 세 목표를 순서대로 추종한다.
    void update_vertical_search(const rclcpp::Time & current_time)
    {
        if ((current_time - state_enter_time_).seconds() >= VERTICAL_SEARCH_TIMEOUT_S) {
            RCLCPP_ERROR(
                get_logger(), "Vertical search timed out after %.1f s; control disabled.",
                VERTICAL_SEARCH_TIMEOUT_S);
            control_enabled_ = false;
            transition_to(State::DISABLED);
            publish_rc(make_release_override());
            return;
        }

        double target_z_m = vertical_up_target_z_m_;
        if (vertical_search_phase_ == VerticalSearchPhase::MOVE_DOWN) {
            target_z_m = vertical_down_target_z_m_;
        } else if (vertical_search_phase_ == VerticalSearchPhase::MOVE_BEST) {
            target_z_m = have_vertical_best_z_ ?
                vertical_best_z_m_ : vertical_search_start_z_m_;
        }
        target_z_m = clamp(
            target_z_m, vertical_search_min_z_m_, vertical_search_max_z_m_);

        if (std::abs(target_z_m - current_odometry_z_m_) <=
            VERTICAL_TARGET_TOLERANCE_M)
        {
            publish_rc(make_rc_override(Command{}));
            if (vertical_search_phase_ == VerticalSearchPhase::MOVE_UP) {
                vertical_search_phase_ = VerticalSearchPhase::MOVE_DOWN;
            } else if (vertical_search_phase_ == VerticalSearchPhase::MOVE_DOWN) {
                vertical_search_phase_ = VerticalSearchPhase::MOVE_BEST;
            } else {
                vertical_search_completed_ = true;
                transition_to(State::INITIAL_SEARCH);
            }
            return;
        }

        Command command;
        // odometry z는 위로 갈수록 증가한다. 기존 heave 축 부호에 맞춰 오차에 음수를 곱한다.
        command.heave = clamp(
            -heave_gain_ * (target_z_m - current_odometry_z_m_),
            -heave_limit_,
            heave_limit_);
        publish_rc(make_rc_override(command));
    }

    // [탐색 중 신호 획득] 준비되고 안정적인 방향이 일정 시간 유지되면 HOMING으로 전환한다.
    void update_search_acquisition(const rclcpp::Time & current_time, const bool signal_valid)
    {
        // 방향이 불안정하면 연속 획득 시간을 처음부터 다시 센다.
        if (!signal_valid || !estimator_ready_is_fresh(current_time) || !direction_is_stable()) {
            have_acquire_start_ = false;
            return;
        }
        if (!have_acquire_start_) {
            acquire_start_time_ = current_time;
            have_acquire_start_ = true;
        }

        const double search_elapsed_s = (current_time - state_enter_time_).seconds();
        const double acquire_elapsed_s = (current_time - acquire_start_time_).seconds();
        if (search_elapsed_s < search_min_duration_s_ || acquire_elapsed_s < acquire_hold_s_) {
            return;
        }

        last_valid_signal_time_ = current_time;
        have_last_valid_signal_time_ = true;
        transition_to(State::HOMING);
    }

    // [신호 유효성 판정] 방향·신뢰도의 timeout과 최소 신뢰도 조건을 함께 검사한다.
    bool have_valid_signal(const rclcpp::Time & current_time) const
    {
        if (!have_direction_ || !have_confidence_) {
            return false;
        }
        if ((current_time - last_direction_time_).seconds() > direction_timeout_s_ ||
            (current_time - last_confidence_time_).seconds() > confidence_timeout_s_)
        {
            return false;
        }
        return direction_confidence_ >= min_direction_confidence_;
    }

    // [방향 안정성 판정] 최근 body-frame bearing의 원형 분산이 허용치 이하인지 확인한다.
    bool direction_is_stable() const
    {
        if (bearing_history_.size() < 3) {
            return false;
        }
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (const auto & sample : bearing_history_) {
            mean += Eigen::Vector2d(std::cos(sample.second), std::sin(sample.second));
        }
        const double mean_angle = std::atan2(mean.y(), mean.x());
        double squared_error = 0.0;
        for (const auto & sample : bearing_history_) {
            const double error = wrap_pi(sample.second - mean_angle);
            squared_error += error * error;
        }
        return std::sqrt(squared_error / static_cast<double>(bearing_history_.size())) <=
            max_direction_std_rad_;
    }

    // [준비 상태 freshness] estimator-ready가 true이며 지정 timeout 안에 갱신됐는지 확인한다.
    bool estimator_ready_is_fresh(const rclcpp::Time & current_time) const
    {
        return estimator_ready_ &&
            (current_time - last_estimator_ready_time_).seconds() <= estimator_ready_timeout_s_;
    }

    // [Odometry freshness] 경계 제어가 오래된 위치값으로 계속 진행되지 않도록 수신 시각을 검사한다.
    bool odometry_is_fresh(const rclcpp::Time & current_time) const
    {
        return have_odometry_ &&
            (current_time - last_odometry_receive_time_).seconds() <= odometry_timeout_s_;
    }

    // [초기 대각선 기준점 저장] 첫 유효 odometry 위치와 시각을 거리·timeout 계산 원점으로 잡는다.
    void capture_initial_diagonal_start()
    {
        if (!have_odometry_) {
            return;
        }
        initial_diagonal_start_xy_ = current_odometry_xy_;
        initial_diagonal_start_time_ = now();
        have_initial_diagonal_start_ = true;
    }

    // [초기 대각선 명령 생성] 시작 모서리에서 경기장 중심 쪽 world 방향을 body forward+sway로 변환한다.
    Command initial_diagonal_command() const
    {
        const double horizontal_sign =
            arena_start_corner_ == "bottom_right" ? -1.0 : 1.0;
        Eigen::Vector2d arena_direction(
            horizontal_sign * arena_width_m_, arena_height_m_);
        arena_direction.normalize();

        const double arena_c = std::cos(arena_yaw_rad_);
        const double arena_s = std::sin(arena_yaw_rad_);
        const Eigen::Vector2d world_direction(
            arena_c * arena_direction.x() - arena_s * arena_direction.y(),
            arena_s * arena_direction.x() + arena_c * arena_direction.y());

        const double yaw_c = std::cos(current_yaw_rad_);
        const double yaw_s = std::sin(current_yaw_rad_);
        Command command;
        command.forward = initial_diagonal_command_ *
            (yaw_c * world_direction.x() + yaw_s * world_direction.y());
        command.sway = initial_diagonal_command_ *
            (-yaw_s * world_direction.x() + yaw_c * world_direction.y());
        return command;
    }

    // [탐색 명령 생성] 전진과 일정 yaw를 조합해 방향 관측을 위한 원호 주행 명령을 만든다.
    Command search_command() const
    {
        // 전진과 일정 yaw를 동시에 주어 원형 또는 충분한 곡률의 호를 만든다.
        Command command;
        command.forward = search_forward_;
        command.yaw = search_turn_sign_ * search_yaw_;
        return command;
    }

    // [Homing 명령 생성] 방향 오차·z 방향·신뢰도에 따라 전진, yaw, heave 명령을 계산한다.
    Command homing_command() const
    {
        Command command;

        const double bearing = wrap_pi(std::atan2(direction_y_, direction_x_));
        const double absolute_bearing = std::abs(bearing);
        command.yaw = absolute_bearing <= center_deadband_rad_ ?
            0.0 : clamp(yaw_gain_ * bearing, -yaw_limit_, yaw_limit_);
        command.heave = clamp(-heave_gain_ * direction_z_, -heave_limit_, heave_limit_);

        // 방향 오차가 크면 회전을 우선하고 전진 속도를 줄인다.
        double forward = forward_fast_;
        if (absolute_bearing > 1.10) {
            forward = std::min(forward, forward_slow_);
        } else if (absolute_bearing > 0.72) {
            forward = std::min(forward, forward_mid_);
        } else if (absolute_bearing > 0.42) {
            forward = std::min(forward, std::max(forward_mid_, 0.55));
        }
        if (direction_x_ < 0.05) {
            forward = std::min(forward, 0.15);
        }

        // 신뢰도가 낮을수록 전진량을 줄이되 획득 임계값 이상에서는 정지하지 않는다.
        const double confidence_scale = confidence_speed_floor_ +
            (1.0 - confidence_speed_floor_) * direction_confidence_;
        command.forward = clamp(forward * confidence_scale, 0.0, 1.0);
        return command;
    }

    // [경기장 경계 clamp] body 이동 명령을 경기장 축으로 바꾸고 벽 쪽 성분만 제거한다.
    Command clamp_command_to_arena(const Command & command) const
    {
        Command bounded = command;
        if (!have_odometry_ || !have_arena_corner_) {
            bounded.forward = 0.0;
            bounded.sway = 0.0;
            return bounded;
        }

        const double arena_c = std::cos(arena_yaw_rad_);
        const double arena_s = std::sin(arena_yaw_rad_);
        const Eigen::Vector2d offset = current_odometry_xy_ - arena_corner_xy_;
        const double raw_x = arena_c * offset.x() + arena_s * offset.y();
        const double arena_x =
            raw_x + (arena_start_corner_ == "bottom_right" ?
            arena_width_m_ - arena_start_inset_m_ : arena_start_inset_m_);
        const double arena_y =
            -arena_s * offset.x() + arena_c * offset.y() + arena_start_inset_m_;

        const double yaw_c = std::cos(current_yaw_rad_);
        const double yaw_s = std::sin(current_yaw_rad_);
        const Eigen::Vector2d world_velocity(
            yaw_c * command.forward - yaw_s * command.sway,
            yaw_s * command.forward + yaw_c * command.sway);
        Eigen::Vector2d arena_velocity(
            arena_c * world_velocity.x() + arena_s * world_velocity.y(),
            -arena_s * world_velocity.x() + arena_c * world_velocity.y());

        if (arena_x <= geofence_margin_m_ && arena_velocity.x() < 0.0) {
            arena_velocity.x() = 0.0;
        }
        if (arena_x >= arena_width_m_ - geofence_margin_m_ && arena_velocity.x() > 0.0) {
            arena_velocity.x() = 0.0;
        }
        if (arena_y <= geofence_margin_m_ && arena_velocity.y() < 0.0) {
            arena_velocity.y() = 0.0;
        }
        if (arena_y >= arena_height_m_ - geofence_margin_m_ && arena_velocity.y() > 0.0) {
            arena_velocity.y() = 0.0;
        }

        const Eigen::Vector2d bounded_world_velocity(
            arena_c * arena_velocity.x() - arena_s * arena_velocity.y(),
            arena_s * arena_velocity.x() + arena_c * arena_velocity.y());
        bounded.forward =
            yaw_c * bounded_world_velocity.x() + yaw_s * bounded_world_velocity.y();
        bounded.sway =
            -yaw_s * bounded_world_velocity.x() + yaw_c * bounded_world_velocity.y();
        return bounded;
    }

    // [RC override 변환] 정규화된 4축 명령을 MAVROS 채널별 PWM으로 매핑한다.
    mavros_msgs::msg::OverrideRCIn make_rc_override(const Command & command) const
    {
        const Command bounded = clamp_command_to_arena(command);
        mavros_msgs::msg::OverrideRCIn msg;
        msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        for (std::size_t index = 0;
            index < PRIMARY_CHANNEL_COUNT && index < msg.channels.size();
            ++index)
        {
            msg.channels[index] = RC_NEUTRAL;
        }

        msg.channels[PITCH_CHANNEL_INDEX] = RC_NEUTRAL;
        msg.channels[ROLL_CHANNEL_INDEX] = RC_NEUTRAL;
        msg.channels[VERTICAL_CHANNEL_INDEX] = axis_pwm(bounded.heave, invert_rc_heave_);
        msg.channels[YAW_CHANNEL_INDEX] = axis_pwm(bounded.yaw, invert_rc_yaw_);
        msg.channels[FORWARD_CHANNEL_INDEX] = axis_pwm(bounded.forward, false);
        msg.channels[LATERAL_CHANNEL_INDEX] =
            axis_pwm(bounded.sway, invert_rc_lateral_);
        return msg;
    }

    // [RC override 해제 생성] 모든 채널을 CHAN_RELEASE로 채워 autopilot/수동 제어권을 돌려준다.
    mavros_msgs::msg::OverrideRCIn make_release_override() const
    {
        mavros_msgs::msg::OverrideRCIn msg;
        msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
        return msg;
    }

    // [RC 명령 발행] preview는 항상 발행하고 dry-run이 아닐 때만 실제 MAVROS로 전송한다.
    void publish_rc(const mavros_msgs::msg::OverrideRCIn & msg)
    {
        // preview는 항상 발행해 dry-run과 실기에서 동일한 명령을 비교할 수 있다.
        rc_preview_pub_->publish(msg);
        if (!dry_run_) {
            rc_pub_->publish(msg);
        }
    }

    // [제어 상태 전환] 미션 시작에서만 추정기를 완전 초기화하고 탐색 전환에는 map을 유지한다.
    void transition_to(const State next_state)
    {
        if (state_ == next_state) {
            return;
        }
        if (state_ == State::VERTICAL_SEARCH &&
            next_state != State::VERTICAL_SEARCH)
        {
            publish_vertical_search_active(false);
        }
        state_ = next_state;
        state_enter_time_ = now();
        have_acquire_start_ = false;
        if (next_state == State::INITIAL_DIAGONAL) {
            vertical_search_requested_ = false;
            vertical_search_completed_ = false;
            have_vertical_best_z_ = false;
            have_initial_diagonal_start_ = false;
            if (have_odometry_) {
                capture_initial_diagonal_start();
            }
        }
        if (next_state == State::VERTICAL_SEARCH) {
            vertical_search_start_z_m_ = current_odometry_z_m_;
            vertical_up_target_z_m_ = clamp(
                vertical_search_start_z_m_ + vertical_search_distance_m_,
                vertical_search_min_z_m_,
                vertical_search_max_z_m_);
            vertical_down_target_z_m_ = clamp(
                vertical_search_start_z_m_ - vertical_search_distance_m_,
                vertical_search_min_z_m_,
                vertical_search_max_z_m_);
            vertical_search_phase_ = VerticalSearchPhase::MOVE_UP;
            have_vertical_best_z_ = false;
            publish_vertical_search_active(true);
        }
        if (next_state == State::INITIAL_DIAGONAL ||
            next_state == State::INITIAL_SEARCH)
        {
            estimator_ready_ = false;
            have_direction_ = false;
            have_confidence_ = false;
            bearing_history_.clear();
        }
        // enable 상승으로 새 미션을 시작할 때만 장기 SNR map까지 초기화한다.
        if (next_state == State::INITIAL_DIAGONAL) {
            reset_pub_->publish(std_msgs::msg::Empty{});
        }
        publish_state();
        RCLCPP_INFO(get_logger(), "Homing control state -> %s", state_name(state_));
    }

    // [수직 탐색 활성 발행] V2가 수평 map 대신 z-SNR 표본을 수집하도록 상태를 알린다.
    void publish_vertical_search_active(const bool active)
    {
        std_msgs::msg::Bool msg;
        msg.data = active;
        vertical_search_active_pub_->publish(msg);
    }

    // [제어 상태 발행] 현재 enum 상태를 사람이 읽을 수 있는 문자열 토픽으로 내보낸다.
    void publish_state()
    {
        std_msgs::msg::String msg;
        msg.data = state_name(state_);
        state_pub_->publish(msg);
    }

    // [상태 이름 변환] 내부 상태 enum을 로그와 토픽에서 사용할 고정 문자열로 바꾼다.
    static const char * state_name(const State state)
    {
        switch (state) {
            case State::DISABLED:
                return "DISABLED";
            case State::INITIAL_DIAGONAL:
                return "INITIAL_DIAGONAL";
            case State::INITIAL_SEARCH:
                return "INITIAL_SEARCH";
            case State::VERTICAL_SEARCH:
                return "VERTICAL_SEARCH";
            case State::HOMING:
                return "HOMING";
        }
        return "UNKNOWN";
    }

    // [축 명령 PWM 변환] -1~1 입력을 중립 기준 PWM으로 바꾸고 반전·안전 범위를 적용한다.
    std::uint16_t axis_pwm(const double value, const bool invert) const
    {
        const double axis = invert ? -value : value;
        const int pwm = static_cast<int>(std::llround(
            static_cast<double>(RC_NEUTRAL) + clamp(axis, -1.0, 1.0) * rc_pwm_span_));
        return static_cast<std::uint16_t>(std::clamp(pwm, 1100, 1900));
    }

    // [범위 제한] 값이 지정한 하한과 상한을 벗어나지 않도록 제한한다.
    static double clamp(const double value, const double low, const double high)
    {
        return std::max(low, std::min(value, high));
    }

    // [각도 정규화] 임의의 radian 각도를 -pi~pi 구간으로 접는다.
    static double wrap_pi(double angle)
    {
        while (angle > PI) {
            angle -= 2.0 * PI;
        }
        while (angle < -PI) {
            angle += 2.0 * PI;
        }
        return angle;
    }

    // [Quaternion→Yaw 변환] odometry 자세에서 body/world 이동축 변환에 필요한 yaw를 계산한다.
    static double yaw_from_quaternion(
        const double w, const double x, const double y, const double z)
    {
        const double sin_yaw = 2.0 * (w * z + x * y);
        const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(sin_yaw, cos_yaw);
    }

    std::string direction_topic_;
    std::string confidence_topic_;
    std::string enable_topic_;
    std::string state_topic_;
    std::string rc_override_topic_;
    std::string rc_preview_topic_;
    std::string estimator_ready_topic_;
    std::string reset_topic_;
    std::string odometry_topic_;
    std::string required_direction_frame_;

    bool control_enabled_ = false;
    bool dry_run_ = true;
    double rate_hz_ = 30.0;
    double direction_timeout_s_ = 0.8;
    double confidence_timeout_s_ = 0.8;
    double min_direction_confidence_ = 0.15;
    double acquire_hold_s_ = 1.0;
    double search_min_duration_s_ = 6.0;
    double direction_loss_hold_s_ = 1.0;
    double direction_stability_window_s_ = 1.0;
    double max_direction_std_rad_ = 0.30;
    double estimator_ready_timeout_s_ = 1.0;
    double arena_width_m_ = 15.0;
    double arena_height_m_ = 16.0;
    std::string arena_start_corner_ = "bottom_left";
    double arena_yaw_rad_ = 0.0;
    double arena_start_inset_m_ = 0.12;
    double geofence_margin_m_ = 0.20;
    double initial_diagonal_distance_m_ = 0.70;
    double initial_diagonal_command_ = 0.30;
    double initial_diagonal_timeout_s_ = 10.0;
    double odometry_timeout_s_ = 0.50;
    double vertical_search_distance_m_ = 0.50;
    double vertical_search_min_z_m_ = -1.30;
    double vertical_search_max_z_m_ = -0.20;

    double search_forward_ = 0.30;
    double search_yaw_ = 0.30;
    double search_turn_sign_ = 1.0;

    double forward_fast_ = 0.70;
    double forward_mid_ = 0.45;
    double forward_slow_ = 0.20;
    double yaw_gain_ = 1.15;
    double yaw_limit_ = 0.72;
    double heave_gain_ = 0.42;
    double heave_limit_ = 0.38;
    double center_deadband_rad_ = 0.055;
    double confidence_speed_floor_ = 0.40;
    double rc_pwm_span_ = 400.0;
    bool invert_rc_heave_ = true;
    bool invert_rc_yaw_ = true;
    bool invert_rc_lateral_ = false;

    State state_ = State::DISABLED;
    rclcpp::Time state_enter_time_;
    rclcpp::Time acquire_start_time_;
    bool have_acquire_start_ = false;
    rclcpp::Time last_valid_signal_time_;
    bool have_last_valid_signal_time_ = false;
    Eigen::Vector2d current_odometry_xy_{0.0, 0.0};
    Eigen::Vector2d arena_corner_xy_{0.0, 0.0};
    Eigen::Vector2d initial_diagonal_start_xy_{0.0, 0.0};
    double current_yaw_rad_ = 0.0;
    double current_odometry_z_m_ = 0.0;
    rclcpp::Time initial_diagonal_start_time_;
    rclcpp::Time last_odometry_receive_time_;
    bool have_odometry_ = false;
    bool have_arena_corner_ = false;
    bool have_initial_diagonal_start_ = false;

    double direction_x_ = 1.0;
    double direction_y_ = 0.0;
    double direction_z_ = 0.0;
    double direction_confidence_ = 0.0;
    rclcpp::Time last_direction_time_;
    rclcpp::Time last_confidence_time_;
    bool have_direction_ = false;
    bool have_confidence_ = false;
    bool estimator_ready_ = false;
    rclcpp::Time last_estimator_ready_time_;
    std::deque<std::pair<rclcpp::Time, double>> bearing_history_;
    VerticalSearchPhase vertical_search_phase_ = VerticalSearchPhase::MOVE_UP;
    bool vertical_search_requested_ = false;
    bool vertical_search_completed_ = false;
    double vertical_search_start_z_m_ = 0.0;
    double vertical_up_target_z_m_ = 0.0;
    double vertical_down_target_z_m_ = 0.0;
    double vertical_best_z_m_ = 0.0;
    bool have_vertical_best_z_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr confidence_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estimator_ready_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vertical_search_request_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vertical_best_z_sub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_preview_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vertical_search_active_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::SnrGradientHomingControllerNode)
