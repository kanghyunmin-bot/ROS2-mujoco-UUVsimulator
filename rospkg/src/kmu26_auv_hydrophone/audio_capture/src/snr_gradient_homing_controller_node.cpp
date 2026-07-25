#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
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

// arena_yaw_rad: 수조 가로축이 위치 지도(odom) 기준으로 얼마나 돌아가 있는지(rad).
// 초기 대각선·벽 회복 접선 방향을 맞출 때 쓴다.
// bottom_right에서 안쪽(왼쪽 위)으로 가야 하는데 반대로 가면 0과 π를 바꿔 본다.


namespace audio_capture
{
// SNR gradient 전용 제어 흐름:
//   INITIAL_DIAGONAL -> INITIAL_SEARCH -> HOMING
//   HOMING 중 vertical_search_request -> VERTICAL_SEARCH
//       -> SHORT_REACQUIRE (짧은 수평 재획득) -> HOMING
//   INITIAL_SEARCH/HOMING/curved recovery 중 명령 방향 진행속도 저하가
//   충돌 유지 시간 동안 이어지면 WALL_RECOVERY
//   WALL_RECOVERY 후에는 진입 전 탐색 상태로 복귀한다.
//   HOMING 중 짧은 direction 손실 -> 저속 curved recovery probe (map 유지)
//   HOMING 중 장시간 direction 손실 -> INITIAL_SEARCH (map 유지, reset 없음)
//   /homing/reset_estimator 는 INITIAL_DIAGONAL(미션 시작)에서만 발행한다.
class SnrGradientHomingControllerNode : public rclcpp::Node
{
public:
    // [제어 노드 초기화] homing 입력, 안전 파라미터, 상태기계, RC 출력과 주기 timer를 구성한다.
    explicit SnrGradientHomingControllerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("snr_gradient_homing_controller", options)
    {
        // ROS 입출력 설정.
        const std::string direction_topic =
            declare_parameter<std::string>("direction_topic", "/homing/direction");
        const std::string confidence_topic =
            declare_parameter<std::string>("confidence_topic", "/homing/snr_confidence");
        const std::string state_topic =
            declare_parameter<std::string>("state_topic", "/homing/control_state");
        const std::string reset_topic =
            declare_parameter<std::string>("reset_topic", "/homing/reset_estimator");
        const std::string odometry_topic =
            declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
        const std::string rc_override_topic =
            declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
        const std::string rc_preview_topic =
            declare_parameter<std::string>("rc_preview_topic", "/homing/rc_override_preview");
        required_direction_frame_ =
            declare_parameter<std::string>("required_direction_frame", "base_link");

        // 상태 전환과 방향 유효성 설정.
        rate_hz_ = clamp(declare_parameter<double>("rate_hz", 30.0), 1.0, 120.0);
        min_direction_confidence_ =
            clamp(declare_parameter<double>("min_direction_confidence", 0.05), 0.0, 1.0);
        acquire_hold_s_ =
            clamp(declare_parameter<double>("acquire_hold_s", 1.0), 0.0, 30.0);
        search_min_duration_s_ =
            clamp(declare_parameter<double>("search_min_duration_s", 6.0), 0.0, 120.0);
        short_reacquire_min_duration_s_ = clamp(
            declare_parameter<double>("short_reacquire_min_duration_s", 1.0), 0.0, 30.0);
        short_reacquire_timeout_s_ = clamp(
            declare_parameter<double>("short_reacquire_timeout_s", 3.0), 0.1, 60.0);
        recovery_to_search_s_ = clamp(
            declare_parameter<double>("recovery_to_search_s", 3.0), 0.1, 60.0);

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
        stuck_progress_threshold_mps_ = clamp(
            declare_parameter<double>("stuck_progress_threshold_mps", 0.03), 0.0, 2.0);
        stuck_yaw_rate_threshold_rps_ = clamp(
            declare_parameter<double>("stuck_yaw_rate_threshold_rps", 0.05), 0.0, 4.0);
        collision_hold_s_ = clamp(
            declare_parameter<double>("collision_hold_s", 3.0), 0.1, 10.0);
        wall_reverse_command_ = clamp(
            declare_parameter<double>("wall_reverse_command", 0.25), 0.0, 1.0);
        wall_reverse_duration_s_ = clamp(
            declare_parameter<double>("wall_reverse_duration_s", 2.0), 0.1, 10.0);
        wall_escape_forward_command_ = clamp(
            declare_parameter<double>("wall_escape_forward_command", 0.25), 0.0, 1.0);
        wall_escape_duration_s_ = clamp(
            declare_parameter<double>("wall_escape_duration_s", 1.5), 0.1, 10.0);
        collision_rearm_timeout_s_ = clamp(
            declare_parameter<double>("collision_rearm_timeout_s", 1.5), 0.1, 10.0);
        wall_homing_resume_grace_s_ = clamp(
            declare_parameter<double>("wall_homing_resume_grace_s", 2.0), 0.1, 10.0);
        initial_diagonal_command_ = clamp(
            declare_parameter<double>("initial_diagonal_command", 0.30), 0.0, 1.0);
        initial_diagonal_duration_s_ = clamp(
            declare_parameter<double>("initial_diagonal_duration_s", 6.0), 0.1, 120.0);
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
            direction_topic,
            10,
            std::bind(
                &SnrGradientHomingControllerNode::direction_callback,
                this,
                std::placeholders::_1));
        confidence_sub_ = create_subscription<std_msgs::msg::Float64>(
            confidence_topic,
            10,
            std::bind(
                &SnrGradientHomingControllerNode::confidence_callback,
                this,
                std::placeholders::_1));
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 20,
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

        rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic, 10);
        rc_preview_pub_ =
            create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_preview_topic, 10);
        state_pub_ = create_publisher<std_msgs::msg::String>(
            state_topic, rclcpp::QoS(1).reliable().transient_local());
        reset_pub_ = create_publisher<std_msgs::msg::Empty>(reset_topic, 10);
        vertical_search_active_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/homing/vertical_search_active",
            rclcpp::QoS(1).reliable().transient_local());
        publish_vertical_search_active(false);

        transition_to(State::INITIAL_DIAGONAL, true);

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SnrGradientHomingControllerNode::control_loop, this));

        RCLCPP_INFO(
            get_logger(),
            "SNR gradient controller ready. direction=%s frame=%s rc=%s preview=%s",
            direction_topic.c_str(),
            required_direction_frame_.c_str(),
            rc_override_topic.c_str(),
            rc_preview_topic.c_str());
    }

private:
    static constexpr std::uint16_t RC_NEUTRAL = 1500;
    static constexpr std::size_t VERTICAL_CHANNEL_INDEX = 2;
    static constexpr std::size_t YAW_CHANNEL_INDEX = 3;
    static constexpr std::size_t FORWARD_CHANNEL_INDEX = 4;
    static constexpr std::size_t LATERAL_CHANNEL_INDEX = 5;
    static constexpr std::size_t PRIMARY_CHANNEL_COUNT = 8;
    static constexpr double VERTICAL_TARGET_TOLERANCE_M = 0.05;
    static constexpr double VERTICAL_SEARCH_TIMEOUT_S = 40.0;
    static constexpr double WALL_ESCAPE_AWAY_WEIGHT = 0.40;
    // MuJoCo arena에서 확인한 벽면 접선 진행 부호를 사용한다.
    static constexpr double WALL_ESCAPE_TANGENT_WEIGHT = -0.60;

    enum class State
    {
        INITIAL_DIAGONAL,
        INITIAL_SEARCH,
        HOMING,
        VERTICAL_SEARCH,
        SHORT_REACQUIRE,
        WALL_RECOVERY
    };

    enum class VerticalSearchPhase
    {
        MOVE_UP,
        MOVE_DOWN,
        MOVE_BEST
    };

    enum class WallRecoveryPhase
    {
        REVERSE,
        ESCAPE_TURN,
        ESCAPE_FORWARD
    };

    struct Command
    {
        double forward = 0.0;
        double sway = 0.0;
        double heave = 0.0;
        double yaw = 0.0;
    };

    // [Odometry 수신] 경기장 기준 모서리, 현재 위치·yaw와 초기 대각선 시작 시각을 갱신한다.
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        current_odometry_xy_ = Eigen::Vector2d(
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        current_odometry_z_m_ = msg->pose.pose.position.z;
        const double velocity_x = msg->twist.twist.linear.x;
        const double velocity_y = msg->twist.twist.linear.y;
        const double yaw_rate = msg->twist.twist.angular.z;
        have_planar_velocity_ = std::isfinite(velocity_x) && std::isfinite(velocity_y);
        if (have_planar_velocity_) {
            current_velocity_body_xy_ = Eigen::Vector2d(velocity_x, velocity_y);
        }
        have_yaw_rate_ = std::isfinite(yaw_rate);
        if (have_yaw_rate_) {
            current_yaw_rate_rps_ = yaw_rate;
        }
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
        if (state_ == State::INITIAL_DIAGONAL && !initial_diagonal_timer_started_) {
            start_initial_diagonal_timer();
        }
    }

    // [Homing 방향 수신] frame과 수치를 검증해 최신 단위 벡터로 저장한다.
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
        have_direction_ = true;
    }

    // [방향 신뢰도 수신] 유효한 값을 0~1로 제한해 저장한다.
    void confidence_callback(const std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        if (!std::isfinite(msg->data)) {
            return;
        }
        direction_confidence_ = clamp(msg->data, 0.0, 1.0);
        have_confidence_ = true;
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

    // [폐루프 상태기계 실행] 대각선·탐색·수직 sweep·짧은 재획득·homing/recovery 명령을 결정한다.
    void control_loop()
    {
        const rclcpp::Time current_time = now();

        // 충돌 판정과 수직 제어에 필요한 odometry가 오래되면 모든 이동축을 중립으로 둔다.
        if (state_ != State::INITIAL_DIAGONAL && !odometry_is_fresh(current_time)) {
            publish_rc(make_rc_override(Command{}));
            return;
        }

        // Estimator가 준 direction과 confidence가 사용 가능한지 한 번만 판정한다.
        const bool signal_valid = homing_signal_valid();
        bool homing_direction_valid = signal_valid;
        if (signal_valid) {
            post_wall_homing_grace_active_ = false;
        } else if (state_ == State::HOMING) {
            homing_direction_valid = use_preserved_homing_direction(current_time);
        }
        switch (state_) {
            // 미션 시작: 모서리에서 경기장 중심 쪽으로 대각선 이동해 벽에서 떨어진다.
            case State::INITIAL_DIAGONAL: {
                const bool odometry_fresh = odometry_is_fresh(current_time);
                if (initial_diagonal_timer_started_ &&
                    (current_time - initial_diagonal_start_time_).seconds() >=
                    initial_diagonal_duration_s_)
                {
                    publish_rc(make_rc_override(Command{}));
                    transition_to(State::INITIAL_SEARCH);
                } else {
                    const Command command = initial_diagonal_timer_started_ && odometry_fresh ?
                        initial_diagonal_command() : Command{};
                    publish_rc(make_rc_override(command));
                }
                break;
            }

            case State::INITIAL_SEARCH: {
                if (!signal_valid) {
                    have_acquire_start_ = false;
                } else if (!have_acquire_start_) {
                    acquire_start_time_ = current_time;
                    have_acquire_start_ = true;
                }

                const bool search_complete = have_acquire_start_ &&
                    (current_time - state_enter_time_).seconds() >= search_min_duration_s_ &&
                    (current_time - acquire_start_time_).seconds() >= acquire_hold_s_;
                if (search_complete) {
                    transition_to(State::HOMING);
                    publish_rc(make_rc_override(homing_command()));
                } else {
                    const Command command = search_command();
                    if (command_progress_indicates_collision(current_time, command)) {
                        start_wall_recovery(State::INITIAL_SEARCH, false, false);
                        publish_rc(make_rc_override(wall_reverse_command()));
                    } else {
                        publish_rc(make_rc_override(command));
                    }
                }
                break;
            }

            case State::HOMING:
                if (vertical_search_requested_ && !vertical_search_completed_) {
                    publish_rc(make_rc_override(Command{}));
                    transition_to(State::VERTICAL_SEARCH);
                } else if (homing_direction_valid) {
                    recovery_active_ = false;
                    const Command command = homing_command();
                    if (command_progress_indicates_collision(current_time, command)) {
                        start_wall_recovery(State::HOMING, true, false);
                        publish_rc(make_rc_override(wall_reverse_command()));
                    } else {
                        publish_rc(make_rc_override(command));
                    }
                } else {
                    if (!recovery_active_) {
                        recovery_active_ = true;
                        recovery_start_time_ = current_time;
                        RCLCPP_WARN(
                            get_logger(),
                            "HOMING direction unavailable: direction_received=%s, "
                            "confidence_received=%s, confidence=%.2f; "
                            "starting curved recovery probe.",
                            have_direction_ ? "true" : "false",
                            have_confidence_ ? "true" : "false",
                            direction_confidence_);
                    }
                    if ((current_time - recovery_start_time_).seconds() >=
                        recovery_to_search_s_)
                    {
                        RCLCPP_WARN(
                            get_logger(),
                            "HOMING recovery timed out after %.1f s; returning to INITIAL_SEARCH.",
                            recovery_to_search_s_);
                        publish_rc(make_rc_override(Command{}));
                        transition_to(State::INITIAL_SEARCH);
                    } else {
                        const Command command = recovery_command();
                        if (command_progress_indicates_collision(current_time, command)) {
                            start_wall_recovery(State::HOMING, true, true);
                            publish_rc(make_rc_override(wall_reverse_command()));
                        } else {
                            publish_rc(make_rc_override(command));
                        }
                    }
                }
                break;

            case State::WALL_RECOVERY: {
                const double phase_elapsed_s =
                    (current_time - wall_recovery_phase_start_time_).seconds();
                if (wall_recovery_phase_ == WallRecoveryPhase::REVERSE) {
                    if (phase_elapsed_s < wall_reverse_duration_s_) {
                        publish_rc(make_rc_override(wall_reverse_command()));
                    } else {
                        wall_recovery_phase_ = WallRecoveryPhase::ESCAPE_TURN;
                        wall_recovery_phase_start_time_ = current_time;
                        RCLCPP_INFO(get_logger(), "WALL_RECOVERY phase -> ESCAPE_TURN");
                        publish_rc(make_rc_override(Command{}));
                    }
                    break;
                }

                if (wall_recovery_phase_ == WallRecoveryPhase::ESCAPE_TURN) {
                    const double yaw_error = wrap_pi(
                        wall_recovery_escape_yaw_rad_ - current_yaw_rad_);
                    if (std::abs(yaw_error) <= center_deadband_rad_) {
                        wall_recovery_phase_ = WallRecoveryPhase::ESCAPE_FORWARD;
                        wall_recovery_phase_start_time_ = current_time;
                        RCLCPP_INFO(get_logger(), "WALL_RECOVERY phase -> ESCAPE_FORWARD");
                        publish_rc(make_rc_override(wall_escape_forward_command()));
                    } else {
                        Command command;
                        command.yaw = clamp(yaw_gain_ * yaw_error, -yaw_limit_, yaw_limit_);
                        publish_rc(make_rc_override(command));
                    }
                    break;
                }

                if (phase_elapsed_s >= wall_escape_duration_s_) {
                    const bool preserve_homing_after_recovery =
                        wall_recovery_return_state_ == State::HOMING &&
                        have_preserved_homing_direction_ &&
                        !wall_recovery_resume_curved_recovery_;
                    if (preserve_homing_after_recovery) {
                        post_wall_homing_grace_active_ = true;
                        post_wall_homing_grace_start_time_ = current_time;
                        use_preserved_homing_direction(current_time);
                    }
                    transition_to(wall_recovery_return_state_);
                    recovery_active_ = wall_recovery_resume_curved_recovery_;
                    collision_detection_armed_ = false;
                    collision_rearm_start_time_ = current_time;
                    if (signal_valid || preserve_homing_after_recovery) {
                        publish_rc(make_rc_override(homing_command()));
                    } else if (wall_recovery_resume_curved_recovery_) {
                        publish_rc(make_rc_override(recovery_command()));
                    } else {
                        publish_rc(make_rc_override(Command{}));
                    }
                } else {
                    publish_rc(make_rc_override(wall_escape_forward_command()));
                }
                break;
            }

            case State::VERTICAL_SEARCH:
                update_vertical_search(current_time);
                break;

            case State::SHORT_REACQUIRE: {
                const double elapsed_s = (current_time - state_enter_time_).seconds();
                if (signal_valid && elapsed_s >= short_reacquire_min_duration_s_) {
                    RCLCPP_INFO(get_logger(), "SHORT_REACQUIRE succeeded; returning to HOMING.");
                    transition_to(State::HOMING);
                    publish_rc(make_rc_override(homing_command()));
                } else if (elapsed_s >= short_reacquire_timeout_s_) {
                    RCLCPP_WARN(
                        get_logger(),
                        "SHORT_REACQUIRE timed out after %.1f s; returning to INITIAL_SEARCH.",
                        short_reacquire_timeout_s_);
                    publish_rc(make_rc_override(Command{}));
                    transition_to(State::INITIAL_SEARCH);
                } else {
                    publish_rc(make_rc_override(recovery_command()));
                }
                break;
            }
        }
    }

    // [수직 Sweep 실행] heave만 사용해 위→아래→최고 SNR z를 추종한다 (forward/sway/yaw=0).
    void update_vertical_search(const rclcpp::Time & current_time)
    {
        if ((current_time - state_enter_time_).seconds() >= VERTICAL_SEARCH_TIMEOUT_S) {
            RCLCPP_WARN(
                get_logger(),
                "VERTICAL_SEARCH timed out after %.1f s; returning to INITIAL_SEARCH.",
                VERTICAL_SEARCH_TIMEOUT_S);
            // 동일한 latched 요청으로 즉시 수직 탐색에 재진입하지 않도록 이번 시도를 종료 처리한다.
            vertical_search_completed_ = true;
            publish_rc(make_rc_override(Command{}));
            transition_to(State::INITIAL_SEARCH);
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
                RCLCPP_INFO(
                    get_logger(),
                    "VERTICAL_SEARCH completed; entering SHORT_REACQUIRE.");
                transition_to(State::SHORT_REACQUIRE);
            }
            return;
        }

        Command command;
        // odometry z(depth 융합값)는 위로 갈수록 증가한다. heave 부호에 맞춰 오차에 음수를 곱한다.
        command.heave = clamp(
            -heave_gain_ * (target_z_m - current_odometry_z_m_),
            -heave_limit_,
            heave_limit_);
        publish_rc(make_rc_override(command));
    }

    // [Homing 방향 유효성] 수신된 direction과 confidence 값만 검사한다.
    bool homing_signal_valid() const
    {
        return have_direction_ &&
            have_confidence_ &&
            direction_confidence_ >= min_direction_confidence_;
    }

    // [충돌 판정] 명령 방향의 수평 진행속도와 yaw 진행속도가 모두 부족한지 본다.
    bool command_progress_indicates_collision(
        const rclcpp::Time & current_time,
        const Command & command)
    {
        const Eigen::Vector2d horizontal_command(command.forward, command.sway);
        const double command_magnitude = horizontal_command.norm();
        const bool linear_commanded = command_magnitude > 0.0;
        const bool yaw_commanded = std::abs(command.yaw) > 0.0;
        if (!linear_commanded && !yaw_commanded) {
            collision_candidate_active_ = false;
            return false;
        }

        const bool have_linear_measurement = linear_commanded && have_planar_velocity_;
        const bool have_yaw_measurement = yaw_commanded && have_yaw_rate_;
        if (!have_linear_measurement && !have_yaw_measurement) {
            collision_candidate_active_ = false;
            return false;
        }

        const double progress_velocity_mps = have_linear_measurement ?
            projected_velocity(command) : 0.0;
        const double progress_yaw_rate_rps = have_yaw_measurement ?
            current_yaw_rate_rps_ * (command.yaw > 0.0 ? 1.0 : -1.0) : 0.0;
        const bool linear_progressed = have_linear_measurement &&
            progress_velocity_mps >= stuck_progress_threshold_mps_;
        const bool yaw_progressed = have_yaw_measurement &&
            progress_yaw_rate_rps >= stuck_yaw_rate_threshold_rps_;
        // 선형 또는 회전 중 명령한 성분 하나라도 진행하면 정상 제어로 본다.
        const bool motion_progressed = linear_progressed || yaw_progressed;
        if (!collision_detection_armed_) {
            const bool rearm_timed_out =
                (current_time - collision_rearm_start_time_).seconds() >=
                collision_rearm_timeout_s_;
            if (motion_progressed || rearm_timed_out) {
                collision_detection_armed_ = true;
                RCLCPP_INFO(
                    get_logger(),
                    "Collision detection rearmed: linear=%.3f m/s, yaw=%.3f rad/s.",
                    progress_velocity_mps,
                    progress_yaw_rate_rps);
            }
            collision_candidate_active_ = false;
            return false;
        }

        if (motion_progressed) {
            collision_candidate_active_ = false;
            return false;
        }

        if (!collision_candidate_active_) {
            collision_candidate_start_time_ = current_time;
            collision_candidate_active_ = true;
            return false;
        }
        if ((current_time - collision_candidate_start_time_).seconds() < collision_hold_s_) {
            return false;
        }

        RCLCPP_WARN(
            get_logger(),
            "%s collision detected: linear_command=%.2f yaw_command=%.2f, "
            "linear_progress=%.3f m/s yaw_progress=%.3f rad/s.",
            state_name(state_),
            command_magnitude,
            command.yaw,
            progress_velocity_mps,
            progress_yaw_rate_rps);
        collision_candidate_active_ = false;
        return true;
    }

    // [명령 방향 투영속도] 현재 body-frame 선속도의 명령 방향 성분을 반환한다.
    double projected_velocity(const Command & command) const
    {
        const double command_heading = std::atan2(command.sway, command.forward);
        const Eigen::Vector2d command_direction(
            std::cos(command_heading), std::sin(command_heading));
        return current_velocity_body_xy_.dot(command_direction);
    }

    // [벽 접선 계산] 현재 arena 위치에서 가장 가까운 벽과 나란한 world-frame 단위벡터를 만든다.
    Eigen::Vector2d nearest_wall_tangent_world(const Eigen::Vector2d & heading) const
    {
        if (!have_odometry_ || !have_arena_corner_) {
            return Eigen::Vector2d(-heading.y(), heading.x());
        }

        const double arena_c = std::cos(arena_yaw_rad_);
        const double arena_s = std::sin(arena_yaw_rad_);
        const Eigen::Vector2d offset = current_odometry_xy_ - arena_corner_xy_;
        const double arena_x = arena_c * offset.x() + arena_s * offset.y() +
            (arena_start_corner_ == "bottom_right" ?
            arena_width_m_ - arena_start_inset_m_ : arena_start_inset_m_);
        const double arena_y =
            -arena_s * offset.x() + arena_c * offset.y() + arena_start_inset_m_;
        const double x_wall_distance = std::min(
            std::abs(arena_x), std::abs(arena_width_m_ - arena_x));
        const double y_wall_distance = std::min(
            std::abs(arena_y), std::abs(arena_height_m_ - arena_y));
        const Eigen::Vector2d tangent_arena = x_wall_distance <= y_wall_distance ?
            Eigen::Vector2d(0.0, 1.0) : Eigen::Vector2d(1.0, 0.0);
        return Eigen::Vector2d(
            arena_c * tangent_arena.x() - arena_s * tangent_arena.y(),
            arena_s * tangent_arena.x() + arena_c * tangent_arena.y());
    }

    // [벽 회복 시작] 충돌 헤딩 반대 성분과 homing 쪽 벽 접선을 합성해 회피 yaw를 고정한다.
    void start_wall_recovery(
        const State return_state,
        const bool preserve_homing_direction,
        const bool resume_curved_recovery)
    {
        wall_recovery_return_state_ = return_state;
        wall_recovery_resume_curved_recovery_ = resume_curved_recovery;
        const Eigen::Vector2d collision_heading(
            std::cos(current_yaw_rad_), std::sin(current_yaw_rad_));
        Eigen::Vector2d tangent = nearest_wall_tangent_world(collision_heading);
        have_preserved_homing_direction_ = false;
        if (preserve_homing_direction && have_direction_) {
            preserved_homing_yaw_world_rad_ = wrap_pi(current_yaw_rad_ +
                std::atan2(direction_y_, direction_x_));
            have_preserved_homing_direction_ = true;
            const Eigen::Vector2d homing_direction(
                std::cos(preserved_homing_yaw_world_rad_),
                std::sin(preserved_homing_yaw_world_rad_));
            if (tangent.dot(homing_direction) < 0.0) {
                tangent = -tangent;
            }
        } else if (search_turn_sign_ < 0.0) {
            tangent = -tangent;
        }
        const Eigen::Vector2d escape_direction =
            WALL_ESCAPE_AWAY_WEIGHT * -collision_heading +
            WALL_ESCAPE_TANGENT_WEIGHT * tangent;
        wall_recovery_escape_yaw_rad_ = std::atan2(
            escape_direction.y(), escape_direction.x());
        wall_recovery_phase_ = WallRecoveryPhase::REVERSE;
        wall_recovery_phase_start_time_ = now();
        collision_detection_armed_ = false;
        transition_to(State::WALL_RECOVERY);
    }

    // [벽 회복 후 방향 유지] 충돌 당시 world 방향을 현재 자세 기준 body 방향으로 갱신한다.
    bool use_preserved_homing_direction(const rclcpp::Time & current_time)
    {
        if (!post_wall_homing_grace_active_ ||
            !have_preserved_homing_direction_)
        {
            return false;
        }
        if ((current_time - post_wall_homing_grace_start_time_).seconds() >
            wall_homing_resume_grace_s_)
        {
            post_wall_homing_grace_active_ = false;
            return false;
        }
        const double body_bearing = wrap_pi(
            preserved_homing_yaw_world_rad_ - current_yaw_rad_);
        direction_x_ = std::cos(body_bearing);
        direction_y_ = std::sin(body_bearing);
        return true;
    }

    // [벽 회복 후진 명령] 진입 직후와 후진 유지 구간에서 동일한 명령을 사용한다.
    Command wall_reverse_command() const
    {
        Command command;
        command.forward = -wall_reverse_command_;
        return command;
    }

    // [벽 회피 전진 명령] 측면 회피 방향으로 회전한 뒤 벽에서 떨어지는 명령을 만든다.
    Command wall_escape_forward_command() const
    {
        Command command;
        command.forward = wall_escape_forward_command_;
        return command;
    }

    // [Odometry freshness] 경계 제어가 오래된 위치값으로 계속 진행되지 않도록 수신 시각을 검사한다.
    bool odometry_is_fresh(const rclcpp::Time & current_time) const
    {
        return have_odometry_ &&
            (current_time - last_odometry_receive_time_).seconds() <= odometry_timeout_s_;
    }

    // [초기 대각선 타이머 시작] 첫 odometry 수신 시 open-loop 이동 시간을 재기 시작한다.
    void start_initial_diagonal_timer()
    {
        if (!have_odometry_) {
            return;
        }
        initial_diagonal_start_time_ = now();
        initial_diagonal_timer_started_ = true;
    }

    // [초기 대각선 명령 생성] 시작 모서리에서 경기장 중심 쪽 world 방향을 body forward+sway로 변환한다.
    Command initial_diagonal_command() const
    {
        const double horizontal_sign =
            arena_start_corner_ == "bottom_right" ? -1.0 : 1.0; // 왼쪽 모서리: -1, 오른쪽 모서리: 1
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

    // [Recovery / SHORT_REACQUIRE 명령] 탐색보다 느린 forward + 작은 yaw로 곡선 이동을 만든다.
    Command recovery_command() const
    {
        Command command = search_command();
        command.forward *= 0.5;
        command.yaw *= 2.0 / 3.0;
        return command;
    }

    // [Homing 명령 생성] 수평 방향 오차와 신뢰도에 따라 전진과 yaw 명령을 계산한다.
    Command homing_command() const
    {
        Command command;

        const double bearing = std::atan2(direction_y_, direction_x_);
        const double absolute_bearing = std::abs(bearing);
        command.yaw = absolute_bearing <= center_deadband_rad_ ?
            0.0 : clamp(yaw_gain_ * bearing, -yaw_limit_, yaw_limit_);
        // 방향 오차가 크면 회전을 우선하고 전진 속도를 줄인다.
        double forward = forward_fast_;
        if (absolute_bearing > 1.10) {
            forward = std::min(forward, forward_slow_);
        } else if (absolute_bearing > 0.72) {
            forward = std::min(forward, forward_mid_);
        } else if (absolute_bearing > 0.42) {
            forward = std::min(forward, std::max(forward_mid_, 0.55));
        }
        // 신뢰도가 낮을수록 전진량을 줄이되 획득 임계값 이상에서는 정지하지 않는다.
        const double confidence_scale = confidence_speed_floor_ +
            (1.0 - confidence_speed_floor_) * direction_confidence_;
        command.forward = clamp(forward * confidence_scale, 0.0, 1.0);
        return command;
    }

    // [RC override 변환] 정규화된 4축 명령을 MAVROS 채널별 PWM으로 매핑한다.
    mavros_msgs::msg::OverrideRCIn make_rc_override(const Command & command) const
    {
        mavros_msgs::msg::OverrideRCIn msg;
        msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        for (std::size_t index = 0;
            index < PRIMARY_CHANNEL_COUNT && index < msg.channels.size();
            ++index)
        {
            msg.channels[index] = RC_NEUTRAL;
        }

        msg.channels[VERTICAL_CHANNEL_INDEX] = axis_pwm(command.heave, invert_rc_heave_);
        msg.channels[YAW_CHANNEL_INDEX] = axis_pwm(command.yaw, invert_rc_yaw_);
        msg.channels[FORWARD_CHANNEL_INDEX] = axis_pwm(command.forward, false);
        msg.channels[LATERAL_CHANNEL_INDEX] =
            axis_pwm(command.sway, invert_rc_lateral_);
        return msg;
    }

    // [RC 명령 발행] 진단 preview와 실제 MAVROS override를 항상 함께 전송한다.
    void publish_rc(const mavros_msgs::msg::OverrideRCIn & msg)
    {
        rc_preview_pub_->publish(msg);
        rc_pub_->publish(msg);
    }

    // [제어 상태 전환] estimator reset은 INITIAL_DIAGONAL(미션 시작)에서만 발행한다.
    void transition_to(const State next_state, const bool force = false)
    {
        if (!force && state_ == next_state) {
            return;
        }
        if (state_ == State::VERTICAL_SEARCH &&
            next_state != State::VERTICAL_SEARCH)
        {
            publish_vertical_search_active(false);
        }
        if (next_state != State::HOMING) {
            recovery_active_ = false;
        }
        if (next_state != State::HOMING && next_state != State::WALL_RECOVERY) {
            post_wall_homing_grace_active_ = false;
            have_preserved_homing_direction_ = false;
        }
        state_ = next_state;
        state_enter_time_ = now();
        have_acquire_start_ = false;
        collision_candidate_active_ = false;
        if (next_state == State::INITIAL_DIAGONAL) {
            vertical_search_requested_ = false;
            vertical_search_completed_ = false;
            have_vertical_best_z_ = false;
            initial_diagonal_timer_started_ = false;
            if (have_odometry_) {
                start_initial_diagonal_timer();
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
            RCLCPP_INFO(get_logger(), "VERTICAL_SEARCH started.");
        }
        if (next_state == State::SHORT_REACQUIRE) {
            RCLCPP_INFO(
                get_logger(),
                "SHORT_REACQUIRE started (min=%.1f s, timeout=%.1f s).",
                short_reacquire_min_duration_s_,
                short_reacquire_timeout_s_);
        }
        if (next_state == State::WALL_RECOVERY) {
            RCLCPP_INFO(
                get_logger(),
                "WALL_RECOVERY started: reverse %.2f for %.2f s, escape yaw %.2f rad, return to %s.",
                wall_reverse_command_,
                wall_reverse_duration_s_,
                wall_recovery_escape_yaw_rad_,
                state_name(wall_recovery_return_state_));
        }
        // 로컬 수신 플래그만 비운다. Rolling Grid Map reset은 하지 않는다.
        if (next_state == State::INITIAL_DIAGONAL ||
            next_state == State::INITIAL_SEARCH ||
            next_state == State::SHORT_REACQUIRE)
        {
            have_direction_ = false;
            have_confidence_ = false;
        }
        // 새 미션 시작에서만 장기 SNR map까지 초기화한다.
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
            case State::INITIAL_DIAGONAL:
                return "INITIAL_DIAGONAL";
            case State::INITIAL_SEARCH:
                return "INITIAL_SEARCH";
            case State::VERTICAL_SEARCH:
                return "VERTICAL_SEARCH";
            case State::SHORT_REACQUIRE:
                return "SHORT_REACQUIRE";
            case State::HOMING:
                return "HOMING";
            case State::WALL_RECOVERY:
                return "WALL_RECOVERY";
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
        constexpr double PI = 3.14159265358979323846;
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

    std::string required_direction_frame_;

    double rate_hz_ = 30.0;
    double min_direction_confidence_ = 0.05;
    double acquire_hold_s_ = 1.0;
    double search_min_duration_s_ = 6.0;
    double short_reacquire_min_duration_s_ = 1.0;
    double short_reacquire_timeout_s_ = 3.0;
    double recovery_to_search_s_ = 3.0;
    double arena_width_m_ = 15.0;
    double arena_height_m_ = 16.0;
    std::string arena_start_corner_ = "bottom_left";
    double arena_yaw_rad_ = 0.0;
    double arena_start_inset_m_ = 0.12;
    double stuck_progress_threshold_mps_ = 0.03;
    double stuck_yaw_rate_threshold_rps_ = 0.05;
    double collision_hold_s_ = 3.0;
    double wall_reverse_command_ = 0.25;
    double wall_reverse_duration_s_ = 2.0;
    double wall_escape_forward_command_ = 0.25;
    double wall_escape_duration_s_ = 1.5;
    double collision_rearm_timeout_s_ = 1.5;
    double wall_homing_resume_grace_s_ = 2.0;
    double initial_diagonal_command_ = 0.50;
    double initial_diagonal_duration_s_ = 6.0;
    double odometry_timeout_s_ = 0.50;
    double vertical_search_distance_m_ = 0.50;
    double vertical_search_min_z_m_ = -1.30;
    double vertical_search_max_z_m_ = -0.20;

    double search_forward_ = 0.50;
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

    State state_ = State::INITIAL_DIAGONAL;
    rclcpp::Time state_enter_time_;
    rclcpp::Time acquire_start_time_;
    bool have_acquire_start_ = false;
    rclcpp::Time recovery_start_time_;
    bool recovery_active_ = false;
    Eigen::Vector2d current_odometry_xy_{0.0, 0.0};
    Eigen::Vector2d current_velocity_body_xy_{0.0, 0.0};
    Eigen::Vector2d arena_corner_xy_{0.0, 0.0};
    double current_yaw_rad_ = 0.0;
    double current_yaw_rate_rps_ = 0.0;
    double current_odometry_z_m_ = 0.0;
    rclcpp::Time initial_diagonal_start_time_;
    rclcpp::Time last_odometry_receive_time_;
    bool have_odometry_ = false;
    bool have_planar_velocity_ = false;
    bool have_yaw_rate_ = false;
    bool have_arena_corner_ = false;
    bool initial_diagonal_timer_started_ = false;
    rclcpp::Time collision_candidate_start_time_;
    bool collision_candidate_active_ = false;
    bool collision_detection_armed_ = true;
    rclcpp::Time collision_rearm_start_time_;
    State wall_recovery_return_state_ = State::HOMING;
    WallRecoveryPhase wall_recovery_phase_ = WallRecoveryPhase::REVERSE;
    rclcpp::Time wall_recovery_phase_start_time_;
    bool wall_recovery_resume_curved_recovery_ = false;
    double wall_recovery_escape_yaw_rad_ = 0.0;
    double preserved_homing_yaw_world_rad_ = 0.0;
    bool have_preserved_homing_direction_ = false;
    rclcpp::Time post_wall_homing_grace_start_time_;
    bool post_wall_homing_grace_active_ = false;

    double direction_x_ = 1.0;
    double direction_y_ = 0.0;
    double direction_confidence_ = 0.0;
    bool have_direction_ = false;
    bool have_confidence_ = false;
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
