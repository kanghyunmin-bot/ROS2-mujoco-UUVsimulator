// AUV 부표(buoy) 미션 상태머신 노드.
//
// 비전 bbox + 수심을 받아 MAVROS RC override로 throttle/yaw/forward/lateral 제안을 발행한다.
// 기본 흐름:
//   IDLE -> TARGET_CONFIRM -> WAIT_CONTROL_GRANT -> SEARCH/APPROACH_BUOY -> ALIGN_STICK
//        -> INSERT_FORK -> DETACH -> BACKOFF -> VERIFY_RELEASE
//        -> (미검출 시 SEARCH 반복 / 재시도 / 탐색 소진 시 AREA_VERIFY -> ASCEND -> COMPLETE)
// 수심 타임아웃·최대수심 초과 시 FAILSAFE로 전환하고 제어 채널을 해제한다.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "auv_buoy_vision_control/depth_conversion.hpp"
#include "auv_buoy_vision_control/collection_servo.hpp"

class MissionStateMachineNode : public rclcpp::Node
{
public:
  MissionStateMachineNode()
  : Node("mission_state_machine_node")
  {
    // --- 토픽 ---
    bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/vision/buoy_bbox");
    depth_pose_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    // pose.z 를 양의 하방(positive-down) 수심[m]으로 변환: depth = scale * z + offset
    depth_pose_scale_ = declare_parameter<double>("depth_pose_scale", -1.0);
    depth_pose_offset_m_ = declare_parameter<double>("depth_pose_offset_m", 0.0);
    // [ACOUSTIC-VISION HANDSHAKE] Acoustic 요청 후 타깃을 확인하고, 승인 후에만 RC를 출력한다.
    vision_search_request_topic_ = declare_parameter<std::string>(
      "vision_search_request_topic", "/homing/vision_search_active");
    target_confirmed_topic_ = declare_parameter<std::string>(
      "target_confirmed_topic", "/vision/target_confirmed");
    vision_control_granted_topic_ = declare_parameter<std::string>(
      "vision_control_granted_topic", "/homing/vision_control_granted");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mission/state");
    rc_override_topic_ =
      declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
    rc_monitor_topic_ =
      declare_parameter<std::string>("rc_monitor_topic", "/mission/rc_command");

    // --- 제어 주기 / 타임아웃 / 수심 ---
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    detection_timeout_sec_ = declare_parameter<double>("detection_timeout_sec", 1.2);
    depth_timeout_sec_ = declare_parameter<double>("depth_timeout_sec", 1.0);
    surface_depth_m_ = declare_parameter<double>("surface_depth_m", 0.4);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 10.5);
    depth_tolerance_m_ = declare_parameter<double>("depth_tolerance_m", 0.2);
    // Hydrophone 제어기와 같은 PID 구조. 수심은 positive-down이므로 양의 오차는 하강 명령이다.
    depth_kp_ = declare_parameter<double>("depth_kp", 0.3);
    depth_ki_ = declare_parameter<double>("depth_ki", 0.05);
    depth_kd_ = declare_parameter<double>("depth_kd", 0.15);
    depth_bias_ = declare_parameter<double>("depth_bias", 0.0);
    depth_integral_limit_ = declare_parameter<double>("depth_integral_limit", 2.0);
    heave_limit_ = declare_parameter<double>("heave_limit", 0.4);
    max_depth_delta_pwm_ = declare_parameter<int>("max_depth_delta_pwm", 160);
    // 수심·bbox 중심/크기 1차 LPF 시상수[s]. 0이면 필터 비활성
    lpf_tau_sec_ = declare_parameter<double>("lpf_tau_sec", 0.3);

    // --- 비전 탐지 / 탐색 ---
    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);
    stick_class_id_ = declare_parameter<int>("stick_class_id", 1);
    min_detection_hits_ = declare_parameter<int>("min_detection_hits", 3);
    target_confirm_hits_ = declare_parameter<int>("target_confirm_hits", 3);
    target_confirm_sec_ = declare_parameter<double>("target_confirm_sec", 0.2);
    // bbox 면적 / 이미지 면적 비율이 이 값 이상이면 "충분히 가까움"으로 판단
    approach_area_ratio_ = declare_parameter<double>("approach_area_ratio", 0.30);
    search_timeout_sec_ = declare_parameter<double>("search_timeout_sec", 40.0);
    area_verify_sec_ = declare_parameter<double>("area_verify_sec", 12.0);
    // SEARCH 다중 부표 선택: 면적 비슷 판정 / confidence 비슷 판정 / 동일 타깃 판정
    buoy_area_similar_ratio_ = declare_parameter<double>("buoy_area_similar_ratio", 0.15);
    buoy_confidence_similar_delta_ =
      declare_parameter<double>("buoy_confidence_similar_delta", 0.05);
    buoy_same_target_center_ratio_ =
      declare_parameter<double>("buoy_same_target_center_ratio", 0.12);

    // --- 막대(stick) 정렬 ---
    // 이미지 3사분면(왼쪽 아래) 쪽. 끝단이 아닌 대략 (0.30, 0.70)
    fork_target_x_ = declare_parameter<double>("fork_target_x", 0.30);
    fork_target_y_ = declare_parameter<double>("fork_target_y", 0.70);
    stick_deadband_x_ = declare_parameter<double>("stick_deadband_x", 0.06);
    stick_deadband_y_ = declare_parameter<double>("stick_deadband_y", 0.08);
    align_stable_sec_ = declare_parameter<double>("align_stable_sec", 0.7);

    // --- 포크 삽입 / 분리 / 후퇴 / 검증 ---
    insert_pwm_ = declare_parameter<int>("insert_pwm", 1560);
    insert_duration_sec_ = declare_parameter<double>("insert_duration_sec", 0.8);
    detach_pwm_ = declare_parameter<int>("detach_pwm", 1620);
    detach_duration_sec_ = declare_parameter<double>("detach_duration_sec", 0.3);
    backoff_pwm_ = declare_parameter<int>("backoff_pwm", 1420);
    backoff_duration_sec_ = declare_parameter<double>("backoff_duration_sec", 0.5);
    verify_clear_sec_ = declare_parameter<double>("verify_clear_sec", 1.0);
    verify_timeout_sec_ = declare_parameter<double>("verify_timeout_sec", 3.0);
    max_target_retries_ = declare_parameter<int>("max_target_retries", 2);

    // --- RC 채널 / PWM ---
    throttle_channel_ = declare_parameter<int>("throttle_channel", 3);
    yaw_channel_ = declare_parameter<int>("yaw_channel", 4);
    forward_channel_ = declare_parameter<int>("forward_channel", 5);
    neutral_pwm_ = declare_parameter<int>("neutral_pwm", 1500);
    min_pwm_ = declare_parameter<int>("min_pwm", 1300);
    max_pwm_ = declare_parameter<int>("max_pwm", 1700);
    max_yaw_delta_ = declare_parameter<int>("max_yaw_delta", 60);
    max_tracking_depth_delta_ = declare_parameter<int>("max_tracking_depth_delta", 100);
    // APPROACH 전진 최대/최소 PWM. 멀리서 max, 가까워질수록 min까지 선형(P) 감속
    approach_forward_pwm_ = declare_parameter<int>("approach_forward_pwm", 1540);
    approach_forward_min_pwm_ = declare_parameter<int>("approach_forward_min_pwm", 1510);
    // APPROACH/ALIGN throttle: 1=비전만, 0=수심 P만. 기본 0.4는 수심 쪽에 조금 더 무게
    approach_vision_throttle_weight_ =
      declare_parameter<double>("approach_vision_throttle_weight", 0.4);
    search_yaw_pwm_ = declare_parameter<int>("search_yaw_pwm", 1530);
    search_forward_pwm_ = declare_parameter<int>("search_forward_pwm", 1520);
    search_detection_brake_yaw_pwm_ =
      declare_parameter<int>("search_detection_brake_yaw_pwm", 1450);
    search_detection_hold_sec_ =
      declare_parameter<double>("search_detection_hold_sec", 2.0);
    yaw_invert_ = declare_parameter<bool>("yaw_invert", false);
    // true면 throttle PWM 증가 = 상승 (일반적인 설정)
    vertical_positive_is_up_ = declare_parameter<bool>("vertical_positive_is_up", true);

    search_depth_min_ = declare_parameter<double>("search_depth_min_m", 0.3);
    search_depth_max_ = declare_parameter<double>("search_depth_max_m", 2.0);
    search_depth_rate_ = declare_parameter<double>("search_depth_rate_mps", 0.05);
    visual_depth_rate_ = declare_parameter<double>("visual_depth_rate_mps", 0.06);
    visual_depth_gain_ = declare_parameter<double>("visual_depth_gain_m", 0.7);
    hand_target_x_ = declare_parameter<double>("hand_target_x", 0.45);
    hand_target_y_ = declare_parameter<double>("hand_target_y", 0.60);
    hand_handoff_height_ = declare_parameter<double>("hand_handoff_height_ratio", 0.12);
    approach_target_x_ = declare_parameter<double>("approach_target_x", 0.45);
    hand_acquire_height_ = declare_parameter<double>("hand_acquire_buoy_height_ratio", 0.22);
    hand_acquire_front_x_ = declare_parameter<double>("hand_acquire_front_x", 0.85);
    hand_insert_height_ = declare_parameter<double>("hand_insert_height_ratio", 0.25);
    hand_right_x_ = declare_parameter<double>("hand_right_error_x", 1.0);
    hand_right_y_ = declare_parameter<double>("hand_right_error_y", 0.0);
    hand_depth_x_ = declare_parameter<double>("hand_depth_error_x", 0.0);
    hand_depth_y_ = declare_parameter<double>("hand_depth_error_y", 1.0);
    for (double value : {hand_right_x_, hand_right_y_, hand_depth_x_, hand_depth_y_}) {
      if (!std::isfinite(value)) throw std::invalid_argument("Non-finite hand calibration");
    }
    lateral_channel_ = declare_parameter<int>("lateral_channel", 6);
    yaw_deadzone_ = declare_parameter<int>("rc_yaw_deadzone_pwm", 40);
    translation_deadzone_ = declare_parameter<int>("rc_translation_deadzone_pwm", 30);
    heave_deadzone_ = declare_parameter<int>("rc_heave_deadzone_pwm", 30);
    for (int deadzone : {yaw_deadzone_, translation_deadzone_, heave_deadzone_}) {
      if (deadzone < 0 || deadzone >= 200) throw std::invalid_argument("RC dead zone out of range");
    }
    if (!(search_depth_min_ >= 0.0 && search_depth_max_ > search_depth_min_ &&
          search_depth_max_ < max_depth_m_ && search_depth_rate_ > 0.0 &&
          visual_depth_gain_ > 0.0 && visual_depth_rate_ > 0.0 && hand_target_x_ > 0.0 && hand_target_x_ < 1.0 &&
          hand_target_y_ > 0.0 && hand_target_y_ < 1.0 && hand_insert_height_ > 0.0 &&
          hand_insert_height_ < 1.0 && hand_handoff_height_ > 0.0 &&
          hand_handoff_height_ < hand_insert_height_ && approach_target_x_ > 0.0 &&
          approach_target_x_ < 0.5 && hand_acquire_height_ > 0.0 && hand_acquire_height_ < 1.0 &&
          hand_acquire_front_x_ > 0.5 && hand_acquire_front_x_ < 1.0 && lateral_channel_ == 6)) {
      throw std::invalid_argument("invalid collection search/hand parameters");
    }
    validate_parameters();

    // --- 구독 / 발행 ---
    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, 10, std::bind(&MissionStateMachineNode::on_bbox, this, std::placeholders::_1));
    hand_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/auto_collection/hand_bbox", 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) { on_hand_bbox(msg); });
    depth_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      depth_pose_topic_, 10,
      std::bind(&MissionStateMachineNode::on_depth_pose, this, std::placeholders::_1));
    vision_search_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      vision_search_request_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&MissionStateMachineNode::on_vision_search_request, this,
        std::placeholders::_1));
    vision_control_granted_sub_ = create_subscription<std_msgs::msg::Bool>(
      vision_control_granted_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&MissionStateMachineNode::on_vision_control_granted, this,
        std::placeholders::_1));
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);
    rc_monitor_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_monitor_topic_, 10);
    // latched: 늦게 구독해도 마지막 상태를 받을 수 있음
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local());
    target_confirmed_pub_ = create_publisher<std_msgs::msg::Bool>(
      target_confirmed_topic_, rclcpp::QoS(1).reliable().transient_local());

    const double period_sec = 1.0 / std::max(1.0, control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&MissionStateMachineNode::on_timer, this));

    state_entered_at_ = now();
    publish_state();
    publish_target_confirmed(false);
    RCLCPP_INFO(
      get_logger(),
      "Mission state machine ready; vision_request=%s control_grant=%s depth_pose=%s "
      "bbox=%s state=%s rc_output=%s rc_monitor=%s",
      vision_search_request_topic_.c_str(), vision_control_granted_topic_.c_str(),
      depth_pose_topic_.c_str(), bbox_topic_.c_str(),
      state_topic_.c_str(), rc_override_topic_.c_str(), rc_monitor_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Vision RC remains silent until acoustic control is granted");
  }

  // 노드 종료 시 제어 채널을 한 번 RELEASE 해서 수동/다른 제어기에 넘긴다.
  void publish_release_once()
  {
    if (!vision_has_control_) {
      return;
    }
    auto channels = nochange_channels();
    release_controlled_channels(channels);
    publish_channels(channels);
  }

private:
  // 미션 단계. 타이머 콜백에서 switch로 분기한다.
  enum class State
  {
    IDLE,            // [ACOUSTIC-VISION HANDSHAKE] Acoustic 요청 대기, RC 미발행
    TARGET_CONFIRM,  // [ACOUSTIC-VISION HANDSHAKE] buoy 확정만, RC 미발행
    WAIT_CONTROL_GRANT, // [ACOUSTIC-VISION HANDSHAKE] Acoustic RC 종료 승인 대기
    SEARCH,          // yaw 회전하며 buoy 탐색
    APPROACH_BUOY,   // buoy 중심 추적 + 전진
    ALIGN_STICK,     // stick을 포크 목표점으로 정밀 정렬
    INSERT_FORK,     // 전진 펄스로 포크 삽입
    DETACH,          // 더 강한 전진으로 분리
    BACKOFF,         // 후진으로 간격 확보
    VERIFY_RELEASE,  // 대상이 사라졌는지 확인 (재시도 가능)
    AREA_VERIFY,     // 최종 영역 재탐색
    ASCEND,          // 수면으로 부상
    COMPLETE,        // 미션 완료
    FAILSAFE         // 수심 이상 등 안전 정지
  };

  // bbox 토픽 Float32MultiArray 레이아웃 (index):
  //   [1]=유효성(>=0.5), [2]=class_id, [3]=confidence,
  //   [4]=cx, [5]=cy, [6]=w, [7]=h, [8]=img_w, [9]=img_h
  struct Detection
  {
    float confidence{0.0F};
    float center_x{0.0F};
    float center_y{0.0F};
    float width{0.0F};
    float height{0.0F};
    float image_width{0.0F};
    float image_height{0.0F};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    int consecutive_hits{0};  // 타임아웃 내 연속 수신 횟수
  };

  void validate_parameters()
  {
    if (
      min_pwm_ < 1300 || max_pwm_ > 1700 || min_pwm_ >= max_pwm_ ||
      neutral_pwm_ < min_pwm_ || neutral_pwm_ > max_pwm_)
    {
      throw std::invalid_argument(
              "Thruster PWM range must stay within 1300..1700 and include neutral_pwm");
    }
    for (const int channel : {throttle_channel_, yaw_channel_, forward_channel_}) {
      if (channel < 1 || channel > 18) {
        throw std::invalid_argument("RC channel numbers must be in [1, 18]");
      }
    }
    if (
      throttle_channel_ == yaw_channel_ || throttle_channel_ == forward_channel_ ||
      yaw_channel_ == forward_channel_)
    {
      throw std::invalid_argument("Controlled RC channels must be unique");
    }
    if (max_depth_m_ <= surface_depth_m_) {
      throw std::invalid_argument("max_depth_m must be greater than surface_depth_m");
    }
    if (depth_pose_topic_.empty()) {
      throw std::invalid_argument("depth_pose_topic must not be empty");
    }
    if (buoy_class_id_ == stick_class_id_) {
      throw std::invalid_argument("buoy_class_id and stick_class_id must differ");
    }
    if (approach_vision_throttle_weight_ < 0.0 || approach_vision_throttle_weight_ > 1.0) {
      throw std::invalid_argument("approach_vision_throttle_weight must be in [0, 1]");
    }
    if (lpf_tau_sec_ < 0.0) {
      throw std::invalid_argument("lpf_tau_sec must be >= 0");
    }
    if (
      !std::isfinite(depth_kp_) || !std::isfinite(depth_ki_) ||
      !std::isfinite(depth_kd_) || !std::isfinite(depth_bias_) ||
      !std::isfinite(depth_integral_limit_) || !std::isfinite(heave_limit_) ||
      depth_kp_ < 0.0 || depth_ki_ < 0.0 || depth_kd_ < 0.0 ||
      depth_integral_limit_ < 0.0 || heave_limit_ <= 0.0 ||
      max_depth_delta_pwm_ < 0)
    {
      throw std::invalid_argument("invalid depth PID parameters");
    }
    if (search_detection_hold_sec_ < 0.0) {
      throw std::invalid_argument("search_detection_hold_sec must be >= 0");
    }
    if (target_confirm_hits_ < 1 || target_confirm_sec_ < 0.0)
    {
      throw std::invalid_argument("invalid acoustic-vision handshake confirmation parameters");
    }
  }

  // [ACOUSTIC-VISION HANDSHAKE] near zone 요청 후 buoy를 먼저 확정한다. RC는 내지 않는다.
  void on_vision_search_request(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || state_ != State::IDLE) {
      return;
    }
    buoy_.reset();
    hand_acquire_ = false;
    stick_.reset();
    mission_hold_depth_m_.reset();
    target_confirm_started_at_.reset();
    publish_target_confirmed(false);
    transition_to(State::TARGET_CONFIRM, "acoustic vision-search request");
  }

  // [ACOUSTIC-VISION HANDSHAKE] Acoustic confirm / 경계 / timeout grant.
  // IDLE에서도 받는다(탐색 요청 없이 강제 인계된 경우).
  // buoy가 이미 확정돼 있으면 APPROACH, 아니면 SEARCH.
  void on_vision_control_granted(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    if (state_ != State::IDLE &&
      state_ != State::TARGET_CONFIRM &&
      state_ != State::WAIT_CONTROL_GRANT)
    {
      return;
    }
    vision_has_control_ = true;
    if (!has_recent_depth()) {
      transition_to(State::FAILSAFE, "acoustic control granted without valid depth");
      return;
    }
    // Hydrophone은 목표 수심값을 별도 전달하지 않는다. 제어권 인계 순간의 실제 수심을
    // 미션 목표로 고정해, 이후 Vision 단계에서도 같은 깊이를 유지한다.
    mission_hold_depth_m_ = *depth_m_;
    reset_depth_pid();
    RCLCPP_INFO(
      get_logger(), "Hydrophone handoff depth captured: %.2f m", *mission_hold_depth_m_);
    if (recent(buoy_) && buoy_->consecutive_hits >= target_confirm_hits_) {
      transition_to(State::APPROACH_BUOY, "acoustic control released; buoy already confirmed");
      return;
    }
    if (state_ == State::IDLE) {
      buoy_.reset();
      hand_acquire_ = false;
      stick_.reset();
      target_confirm_started_at_.reset();
    }
    transition_to(State::SEARCH, "acoustic control released; visual search started");
  }

  void on_depth_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const double converted_depth_m =
      depth_pose_scale_ * msg->pose.pose.position.z + depth_pose_offset_m_;
    if (std::isfinite(converted_depth_m) && converted_depth_m < 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Depth pose is %.3f m above the configured reference; clamping depth to 0 m.",
        -converted_depth_m);
    }
    accept_depth(auv_buoy_vision_control::pose_z_to_positive_down_depth(
      msg->pose.pose.position.z, depth_pose_scale_, depth_pose_offset_m_));
  }

  // 수심은 양의 하방[m]. 비정상 값은 무시한다. 연속 샘플은 LPF로 완화.
  void accept_depth(double depth_m)
  {
    if (!std::isfinite(depth_m) || depth_m < 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid depth value");
      return;
    }
    const auto received_at = now();
    if (
      depth_m_ &&
      (received_at - depth_received_at_).seconds() <= depth_timeout_sec_)
    {
      const double dt = (received_at - depth_received_at_).seconds();
      depth_m_ = low_pass(*depth_m_, depth_m, dt);
    } else {
      depth_m_ = depth_m;
    }
    depth_received_at_ = received_at;
  }

  void on_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // 메시지에 10개 단위 블록이 여러 개 올 수 있음 (한 프레임 다중 검출)
    if (msg->data.size() < 10) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring bbox with fewer than 10 values");
      return;
    }

    std::optional<Detection> best_buoy;
    std::optional<Detection> best_stick;
    const size_t block_count = msg->data.size() / 10;
    for (size_t block = 0; block < block_count; ++block) {
      const size_t base = block * 10;
      bool finite = true;
      for (size_t index = 0; index < 10; ++index) {
        if (!std::isfinite(msg->data[base + index])) {
          finite = false;
          break;
        }
      }
      if (!finite) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring bbox containing NaN/Inf");
        continue;
      }
      // [1]<0.5 또는 이미지 크기 비정상이면 미탐지로 무시
      if (
        msg->data[base + 1] < 0.5F || msg->data[base + 8] <= 0.0F ||
        msg->data[base + 9] <= 0.0F)
      {
        continue;
      }

      Detection det{
        msg->data[base + 3], msg->data[base + 4], msg->data[base + 5],
        msg->data[base + 6], msg->data[base + 7], msg->data[base + 8],
        msg->data[base + 9], now(), 1};
      const int class_id = static_cast<int>(std::lround(msg->data[base + 2]));
      if (class_id == buoy_class_id_) {
        // Preserve the tracked target before considering larger unrelated boxes.
        if (recent(buoy_) && state_ != State::SEARCH && !same_buoy_target(det, *buoy_)) continue;
        if (!best_buoy || det.confidence > best_buoy->confidence) {
          best_buoy = det;
        }
      } else if (class_id == stick_class_id_) {
        if (!best_stick || det.confidence > best_stick->confidence) {
          best_stick = det;
        }
      }
    }

    if (best_buoy) {
      accept_buoy_detection(*best_buoy);
    }
    if (best_stick) {
      update_detection_slot(stick_, *best_stick);
    }
  }

  void on_hand_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::optional<Detection> best;
    for (size_t base = 0; base + 9 < msg->data.size(); base += 10) {
      bool finite = true;
      for (size_t i = 0; i < 10; ++i) finite = finite && std::isfinite(msg->data[base+i]);
      if (!finite || msg->data[base+1] < 0.5 ||
          static_cast<int>(msg->data[base+2]) != stick_class_id_ ||
          msg->data[base+8] <= 0 || msg->data[base+9] <= 0 ||
          msg->data[base+6] <= 0 || msg->data[base+7] <= 0) continue;
      Detection d{msg->data[base+3], msg->data[base+4], msg->data[base+5],
        msg->data[base+6], msg->data[base+7], msg->data[base+8], msg->data[base+9], now(), 1};
      if (!best || d.confidence > best->confidence) best = d;
    }
    if (best) update_detection_slot(hand_stick_, *best);
  }

  // Prefer the existing target; a batch uses detection confidence/fit first.
  // APPROACH/ALIGN: 같은 타깃만 갱신(탐색 중 고른 부표를 유지).
  void accept_buoy_detection(Detection incoming)
  {
    // SEARCH 중 첫 buoy 검출은 선회 관성을 한 주기 반대로 제동하고,
    // 이후 일정 시간 동안 추가 검출을 기다린다.
    if (state_ == State::SEARCH && !search_detection_hold_started_at_) {
      search_detection_hold_started_at_ = now();
      search_detection_brake_pending_ = true;
    }
    const bool selecting =
      state_ == State::SEARCH || state_ == State::AREA_VERIFY ||
      state_ == State::IDLE || state_ == State::TARGET_CONFIRM ||
      state_ == State::WAIT_CONTROL_GRANT;
    if (!recent(buoy_)) {
      buoy_ = incoming;
      target_confirm_started_at_.reset();
      return;
    }
    if (same_buoy_target(incoming, *buoy_)) {
      update_detection_slot(buoy_, incoming);
      return;
    }
    if (selecting && is_better_buoy(incoming, *buoy_)) {
      buoy_ = incoming;
      target_confirm_started_at_.reset();
      return;
    }
    // APPROACH 등에서는 다른 부표로 타깃을 바꾸지 않음
  }

  bool same_buoy_target(const Detection & a, const Detection & b) const
  {
    const double ref_w = std::max(a.image_width, b.image_width);
    const double ref_h = std::max(a.image_height, b.image_height);
    if (ref_w <= 0.0 || ref_h <= 0.0) {
      return false;
    }
    const double dx = std::abs(a.center_x - b.center_x) / ref_w;
    const double dy = std::abs(a.center_y - b.center_y) / ref_h;
    return dx <= buoy_same_target_center_ratio_ && dy <= buoy_same_target_center_ratio_;
  }

  // 면적 큰 것 우선. 비슷하면 박스 확률(confidence) 높은 것. 그것도 비슷하면 오른쪽.
  bool is_better_buoy(const Detection & candidate, const Detection & current) const
  {
    const double cand_area = std::max(0.0, static_cast<double>(candidate.width * candidate.height));
    const double cur_area = std::max(0.0, static_cast<double>(current.width * current.height));
    const double larger = std::max({cand_area, cur_area, 1.0});
    if (std::abs(cand_area - cur_area) > buoy_area_similar_ratio_ * larger) {
      return cand_area > cur_area;
    }
    if (
      std::abs(candidate.confidence - current.confidence) > buoy_confidence_similar_delta_)
    {
      return candidate.confidence > current.confidence;
    }
    return candidate.center_x > current.center_x;
  }

  // 최근 탐지와 이어지면 consecutive_hits를 증가시키고 중심/크기에 LPF를 적용한다.
  // 타깃이 바뀌거나 타임아웃 후에는 raw 값으로 리셋.
  void update_detection_slot(std::optional<Detection> & slot, Detection incoming)
  {
    const auto received_at = now();
    int hits = 1;
    if (slot && (received_at - slot->received_at).seconds() <= detection_timeout_sec_) {
      hits = slot->consecutive_hits + 1;
      const double dt = (received_at - slot->received_at).seconds();
      incoming.center_x = static_cast<float>(low_pass(slot->center_x, incoming.center_x, dt));
      incoming.center_y = static_cast<float>(low_pass(slot->center_y, incoming.center_y, dt));
      incoming.width = static_cast<float>(low_pass(slot->width, incoming.width, dt));
      incoming.height = static_cast<float>(low_pass(slot->height, incoming.height, dt));
    }
    incoming.received_at = received_at;
    incoming.consecutive_hits = hits;
    slot = incoming;
  }

  // 1차 LPF: y = αx + (1-α)y_prev, α = dt/(τ+dt). τ=0이면 필터 없음.
  double low_pass(double previous, double sample, double dt_sec) const
  {
    if (lpf_tau_sec_ <= 1e-9 || dt_sec <= 0.0) {
      return sample;
    }
    const double alpha = dt_sec / (lpf_tau_sec_ + dt_sec);
    return alpha * sample + (1.0 - alpha) * previous;
  }

  // 주기 제어 루프: 핸드셰이크 게이트 후 현재 State별 동작을 수행하고 RC를 발행한다.
  void on_timer()
  {
    auto channels = nochange_channels();
    const double control_now = now().seconds();
    control_dt_ = last_control_s_ > 0.0 ? std::clamp(control_now - last_control_s_, 0.0, 0.2) : 0.0;
    last_control_s_ = control_now;

    // [ACOUSTIC-VISION HANDSHAKE] 승인 전 상태는 RELEASE도 발행하지 않는다.
    if (state_ == State::IDLE) {
      return;
    }
    if (state_ == State::TARGET_CONFIRM) {
      run_target_confirm();
      return;
    }
    if (state_ == State::WAIT_CONTROL_GRANT || !vision_has_control_) {
      return;
    }

    // 안전: 수심 유실 또는 최대수심 초과
    if (mission_state_requires_depth() && (!has_recent_depth() || !mission_hold_depth_m_)) {
      transition_to(State::FAILSAFE, "depth input or handoff depth stale");
    }
    if (depth_m_ && *depth_m_ > max_depth_m_) {
      transition_to(State::FAILSAFE, "maximum depth exceeded");
    }

    switch (state_) {
      case State::IDLE:
      case State::TARGET_CONFIRM:
      case State::WAIT_CONTROL_GRANT:
        break;
      case State::SEARCH:
        run_search(channels);
        break;
      case State::APPROACH_BUOY:
        run_approach(channels);
        break;
      case State::ALIGN_STICK:
        run_align_stick(channels);
        break;
      case State::INSERT_FORK:
        // 일정 시간 전진 삽입 펄스 + 작업수심 유지(양성 부력 대응)
        set_neutral_control(channels);
        hold_work_depth(channels);
        if (state_age_sec() >= insert_duration_sec_) {
          transition_to(State::DETACH, "fork insertion pulse complete");
          set_channel(channels, forward_channel_, detach_pwm_);
        } else {
          set_channel(channels, forward_channel_, insert_pwm_);
        }
        break;
      case State::DETACH:
        // 짧은 강한 전진으로 분리 + 작업수심 유지
        set_neutral_control(channels);
        hold_work_depth(channels);
        if (state_age_sec() >= detach_duration_sec_) {
          transition_to(State::BACKOFF, "detach pulse complete");
          set_channel(channels, forward_channel_, backoff_pwm_);
        } else {
          set_channel(channels, forward_channel_, detach_pwm_);
        }
        break;
      case State::BACKOFF:
        // 후진으로 대상과 거리 확보 + 작업수심 유지
        set_neutral_control(channels);
        hold_work_depth(channels);
        if (state_age_sec() >= backoff_duration_sec_) {
          transition_to(State::VERIFY_RELEASE, "backoff complete");
        } else {
          set_channel(channels, forward_channel_, backoff_pwm_);
        }
        break;
      case State::VERIFY_RELEASE:
        run_verify_release(channels);
        break;
      case State::AREA_VERIFY:
        run_area_verify(channels);
        break;
      case State::ASCEND:
        run_ascend(channels);
        break;
      case State::COMPLETE:
      case State::FAILSAFE:
        release_controlled_channels(channels);
        break;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "servo state=%s depth=%.3f target=%.3f hand=%d heave=%u sway=%u forward=%u",
      state_name(state_), depth_m_.value_or(-1.0), mission_hold_depth_m_.value_or(-1.0),
      recent(hand_stick_), channels[2], channels[5], channels[4]);
    publish_channels(channels);
  }

  // [ACOUSTIC-VISION HANDSHAKE] grant 전에는 Acoustic에 confirm만 보내고,
  // grant 이후 SEARCH에서는 같은 기준으로 APPROACH로 넘긴다.
  void run_target_confirm()
  {
    const bool stable_candidate = recent(buoy_) &&
      buoy_->consecutive_hits >= target_confirm_hits_;
    if (!stable_candidate) {
      target_confirm_started_at_.reset();
      return;
    }
    if (!target_confirm_started_at_) {
      target_confirm_started_at_ = now();
      return;
    }
    if ((now() - *target_confirm_started_at_).seconds() < target_confirm_sec_) {
      return;
    }
    publish_target_confirmed(true);
    if (!vision_has_control_) {
      transition_to(State::WAIT_CONTROL_GRANT, "stable nearest buoy confirmed");
      return;
    }
    transition_to(State::APPROACH_BUOY, "stable nearest buoy confirmed");
  }

  // Sweep depth and yaw without forward motion. Pause yaw while confirming a candidate.
  void run_search(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    if (!recent(buoy_)) {
      *mission_hold_depth_m_ = auv_buoy_vision_control::search_depth_target(
        *mission_hold_depth_m_, search_depth_rate_, control_dt_,
        search_depth_min_, search_depth_max_, search_depth_direction_);
    }
    set_channel(channels, throttle_channel_, depth_control_pwm(*mission_hold_depth_m_));
    const int yaw_pwm = search_detection_brake_pending_ ?
      search_detection_brake_yaw_pwm_ : search_yaw_pwm_;
    set_channel(channels, yaw_channel_, recent(buoy_) ? neutral_pwm_ : yaw_pwm);
    set_channel(channels, forward_channel_, neutral_pwm_);
    search_detection_brake_pending_ = false;
    run_target_confirm();
    if (
      state_ == State::SEARCH && search_detection_hold_started_at_ &&
      (now() - *search_detection_hold_started_at_).seconds() >= search_detection_hold_sec_)
    {
      buoy_.reset();
      hand_acquire_ = false;
      stick_.reset();
      target_confirm_started_at_.reset();
      search_detection_hold_started_at_.reset();
      RCLCPP_INFO(get_logger(), "Buoy was not confirmed within search detection hold; resuming search");
    }
    if (state_ == State::SEARCH && state_age_sec() >= search_timeout_sec_) {
      transition_to(State::AREA_VERIFY, "initial search exhausted");
    }
  }

  // Prefer front-left during approach, then stop and acquire the hand view.
  void run_approach(std::array<uint16_t, 18> & channels)
  {
    // The hand view takes priority even after the buoy exits the front view.
    if (recent(hand_stick_)) {
      if (hand_stick_->consecutive_hits >= min_detection_hits_ &&
          hand_stick_->height / hand_stick_->image_height >= hand_handoff_height_) {
        set_neutral_control(channels);
        transition_to(State::ALIGN_STICK, "hand camera confirms stick; transfer alignment");
        return;
      }
      if (!recent(buoy_)) {
        set_neutral_control(channels);
        hold_work_depth(channels);
        return;
      }
    }
    if (!recent(buoy_)) {
      set_neutral_control(channels);
      transition_to(State::SEARCH, "both cameras lost during approach");
      return;
    }
    // The mounted hand camera looks right. Stop advance and expose the target
    // in its overlapping field of view before committing to fork alignment.
    if (buoy_->height / buoy_->image_height >= hand_acquire_height_) hand_acquire_ = true;
    apply_visual_tracking(
      channels, *buoy_, hand_acquire_ ? hand_acquire_front_x_ : approach_target_x_, 0.5,
      hand_acquire_ ? neutral_pwm_ : approach_forward_pwm_from_area(*buoy_),
      *mission_hold_depth_m_, approach_vision_throttle_weight_);
  }

  // 박스 면적 비율로 전진 P제어 (선형 감속).
  // Interpolate between the configured maximum and minimum approach commands.
  int approach_forward_pwm_from_area(const Detection & buoy) const
  {
    const double ratio = detection_area_ratio(buoy);
    const double error = std::max(0.0, approach_area_ratio_ - ratio);
    const int floor_pwm = std::min(approach_forward_min_pwm_, approach_forward_pwm_);
    const int max_delta = std::max(0, approach_forward_pwm_ - floor_pwm);
    const double kp =
      (approach_area_ratio_ > 1e-6) ? (static_cast<double>(max_delta) / approach_area_ratio_) : 0.0;
    const int delta = static_cast<int>(std::lround(kp * error));
    return std::clamp(floor_pwm + delta, floor_pwm, approach_forward_pwm_);
  }

  // stick을 fork_target으로 정렬. deadband 안에서 align_stable_sec_ 유지 시 INSERT.
  void run_align_stick(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    if (!recent(hand_stick_)) {
      condition_started_at_.reset();
      if (!recent(buoy_)) {
        hold_work_depth(channels);
        transition_to(State::SEARCH, "both cameras lost target; stop advance and reacquire");
        return;
      }
      // After handoff, do not alternate incompatible front/hand image targets.
      // Hold the last acquired depth and stop advance during short hand occlusions.
      hold_work_depth(channels);
      if (hand_stick_ && (now() - hand_stick_->received_at).seconds() < 3.0) {
        const auto [x, y] = normalized_error(*hand_stick_, hand_target_x_, hand_target_y_);
        set_channel(channels, lateral_channel_, neutral_pwm_ + static_cast<int>((hand_right_x_ * x + hand_right_y_ * y) * 25));
      } else {
        transition_to(State::APPROACH_BUOY, "hand view lost; reacquire through front camera");
      }
      return;
    }
    const auto [x, y] = normalized_error(*hand_stick_, hand_target_x_, hand_target_y_);
    // Camera roll/pitch couples horizontal image error into physical depth.
    const double right_error = hand_right_x_ * x + hand_right_y_ * y;
    const double depth_error = hand_depth_x_ * x + hand_depth_y_ * y;
    update_visual_depth(depth_error);
    hold_work_depth(channels);
    set_channel(channels, lateral_channel_, neutral_pwm_ + static_cast<int>(right_error * 65));
    const bool aligned = std::abs(x) <= stick_deadband_x_ && std::abs(y) <= stick_deadband_y_;
    // Lateral/depth alignment must finish before further approach; continuous
    // advance here previously carried the shaft past the gap into the hull.
    set_channel(channels, forward_channel_, aligned ? std::min(insert_pwm_, 1530) : neutral_pwm_);
    const bool at_fork = hand_stick_->height / hand_stick_->image_height >= hand_insert_height_;
    if (aligned && at_fork) {
      if (!condition_started_at_) condition_started_at_ = now();
      else if ((now() - *condition_started_at_).seconds() >= align_stable_sec_) {
        *mission_hold_depth_m_ = *depth_m_;
        reset_depth_pid();
        transition_to(State::INSERT_FORK, "hand alignment stable; hold achieved depth");
      }
    } else condition_started_at_.reset();
  }

  // Disappearance only resumes search; the runner independently labels contact/release.
  void run_verify_release(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_work_depth(channels);
    if (!recent(buoy_) && state_age_sec() >= verify_clear_sec_) {
      target_retries_ = 0;
      buoy_.reset();
      hand_acquire_ = false;
      stick_.reset();
      transition_to(State::SEARCH, "target absent after backoff; release unverified");
      return;
    }
    if (state_age_sec() >= verify_timeout_sec_) {
      if (target_retries_ < max_target_retries_) {
        ++target_retries_;
        transition_to(
          recent(stick_) ? State::ALIGN_STICK : State::APPROACH_BUOY,
          "target remains; retrying detach");
      } else {
        RCLCPP_ERROR(get_logger(), "Target retry limit reached; abandoning this target");
        target_retries_ = 0;
        buoy_.reset();
      hand_acquire_ = false;
        stick_.reset();
        transition_to(State::SEARCH, "target retry limit reached");
      }
    }
  }

  // 마지막 재탐색. buoy 발견 시 APPROACH, 없으면 ASCEND.
  void run_area_verify(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    set_channel(channels, throttle_channel_, depth_control_pwm(*mission_hold_depth_m_));
    set_channel(channels, yaw_channel_, search_yaw_pwm_);
    set_channel(channels, forward_channel_, search_forward_pwm_);
    if (confirmed_buoy()) {
      transition_to(State::APPROACH_BUOY, "buoy found during area verification");
    } else if (state_age_sec() >= area_verify_sec_) {
      transition_to(State::ASCEND, "area verification complete with no targets");
    }
  }

  // 수면 근처까지 부상 후 COMPLETE.
  void run_ascend(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    // 부상 중에는 양성 부력 바이어스를 끄고 수면 목표만 추종
    set_channel(channels, throttle_channel_, depth_control_pwm(surface_depth_m_, false));
    if (*depth_m_ <= surface_depth_m_ + depth_tolerance_m_) {
      transition_to(State::COMPLETE, "surface depth reached");
    }
  }

  // 화면 정규화 오차로 yaw/throttle 보정 + forward 설정.
  // Legacy blend arguments are retained; vision now moves a bounded depth setpoint.
  // error_x/y 는 [-1, 1], 목표점 기준 화면 중심 대비 편차.
  void apply_visual_tracking(
    std::array<uint16_t, 18> & channels, const Detection & detection,
    double target_x, double target_y, int forward_pwm,
    std::optional<double> depth_blend_target_m = std::nullopt,
    double vision_throttle_weight = 1.0)
  {
    set_neutral_control(channels);
    const auto [error_x, error_y] = normalized_error(detection, target_x, target_y);
    const double yaw_sign = yaw_invert_ ? -1.0 : 1.0;
    set_channel(
      channels, yaw_channel_,
      neutral_pwm_ + static_cast<int>(yaw_sign * error_x * max_yaw_delta_));

    (void)depth_blend_target_m;
    (void)vision_throttle_weight;
    update_visual_depth(error_y);
    set_channel(channels, throttle_channel_, depth_control_pwm(*mission_hold_depth_m_));
    set_channel(channels, forward_channel_, auv_buoy_vision_control::aligned_forward_pwm(
      error_x, error_y, neutral_pwm_, forward_pwm, 0.4));
  }

  // 탐지 중심을 [0,1]로 정규화한 뒤 목표점 대비 오차를 [-1,1]로 클램프.
  std::pair<double, double> normalized_error(
    const Detection & detection, double target_x, double target_y) const
  {
    const double x = detection.center_x / detection.image_width;
    const double y = detection.center_y / detection.image_height;
    return {
      std::clamp((x - target_x) * 2.0, -1.0, 1.0),
      std::clamp((y - target_y) * 2.0, -1.0, 1.0)};
  }

  double detection_area_ratio(const Detection & detection) const
  {
    return static_cast<double>(detection.width * detection.height) /
           static_cast<double>(detection.image_width * detection.image_height);
  }

  void update_visual_depth(double image_error_y)
  {
    const double desired = auv_buoy_vision_control::visual_depth_target(
      *depth_m_, image_error_y, visual_depth_gain_, search_depth_min_, search_depth_max_);
    const double step = visual_depth_rate_ * control_dt_;
    *mission_hold_depth_m_ += std::clamp(desired - *mission_hold_depth_m_, -step, step);
  }

  // Hydrophone 인계 시 저장한 작업수심 유지용 throttle 설정.
  void hold_work_depth(std::array<uint16_t, 18> & channels)
  {
    if (!depth_m_ || !mission_hold_depth_m_) {
      return;
    }
    set_channel(channels, throttle_channel_, depth_control_pwm(*mission_hold_depth_m_));
  }

  // Hydrophone 제어기와 같은 PID + anti-windup. 목표 수심은 인계 시 캡처한 값으로 유지한다.
  // 수심은 positive-down이라 target-current가 양수면 하강 명령을 낸다.
  int depth_control_pwm(double target_depth_m, bool apply_depth_bias = true)
  {
    const double error_m = target_depth_m - *depth_m_;
    const auto current_time = now();
    double dt = 0.0;
    double error_derivative = 0.0;
    if (depth_pid_initialized_) {
      dt = std::clamp((current_time - last_depth_control_time_).seconds(), 0.0, 0.2);
      if (dt > 1e-6) {
        // Derivative on measured depth avoids kicks when the visual setpoint changes.
        error_derivative = -(*depth_m_ - previous_depth_measurement_m_) / dt;
      }
    }
    last_depth_control_time_ = current_time;
    previous_depth_error_m_ = error_m;
    previous_depth_measurement_m_ = *depth_m_;
    depth_pid_initialized_ = true;

    const double candidate_integral = std::clamp(
      depth_error_integral_ + error_m * dt,
      -depth_integral_limit_, depth_integral_limit_);
    const double bias = apply_depth_bias ? depth_bias_ : 0.0;
    const double candidate_heave = bias + depth_kp_ * error_m +
      depth_ki_ * candidate_integral + depth_kd_ * error_derivative;
    // 포화 상태에서 더 포화시키는 방향으로는 적분하지 않는다.
    if (
      std::abs(candidate_heave) <= heave_limit_ ||
      (candidate_heave > heave_limit_ && error_m < 0.0) ||
      (candidate_heave < -heave_limit_ && error_m > 0.0))
    {
      depth_error_integral_ = candidate_integral;
    }
    const double heave = std::clamp(
      bias + depth_kp_ * error_m + depth_ki_ * depth_error_integral_ +
      depth_kd_ * error_derivative,
      -heave_limit_, heave_limit_);
    const double pwm_scale = static_cast<double>(max_depth_delta_pwm_) / heave_limit_;
    const int delta = static_cast<int>(std::lround(heave * pwm_scale));
    const int sign = vertical_positive_is_up_ ? -1 : 1;
    return neutral_pwm_ + sign * delta;
  }

  void reset_depth_pid()
  {
    depth_pid_initialized_ = false;
    previous_depth_error_m_ = 0.0;
    depth_error_integral_ = 0.0;
    last_depth_control_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  bool recent(const std::optional<Detection> & detection) const
  {
    return detection && (now() - detection->received_at).seconds() <= detection_timeout_sec_;
  }

  // 최근 탐지 + 연속 hit 수가 충분할 때만 buoy를 "확정"한다 (오탐 완화).
  bool confirmed_buoy() const
  {
    return recent(buoy_) && buoy_->consecutive_hits >= min_detection_hits_;
  }

  bool has_recent_depth() const
  {
    return depth_m_ && (now() - depth_received_at_).seconds() <= depth_timeout_sec_;
  }

  bool mission_state_requires_depth() const
  {
    return state_ != State::IDLE && state_ != State::TARGET_CONFIRM &&
           state_ != State::WAIT_CONTROL_GRANT &&
           state_ != State::COMPLETE &&
           state_ != State::FAILSAFE;
  }

  double state_age_sec() const
  {
    return (now() - state_entered_at_).seconds();
  }

  // 상태 전이. 동일 상태면 no-op. 진입 시각·조건 타이머를 리셋하고 state 토픽을 발행한다.
  void transition_to(State next, const std::string & reason)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "State %s -> %s: %s", state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    state_entered_at_ = now();
    condition_started_at_.reset();
    if (next != State::SEARCH) {
      search_detection_hold_started_at_.reset();
      search_detection_brake_pending_ = false;
    }
    if (next == State::ASCEND || next == State::COMPLETE || next == State::FAILSAFE) {
      reset_depth_pid();
    }
    if (next == State::COMPLETE || next == State::FAILSAFE) {
      mission_hold_depth_m_.reset();
    }
    publish_state();
  }

  void publish_state()
  {
    if (!state_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    state_pub_->publish(msg);
  }

  // [ACOUSTIC-VISION HANDSHAKE] 타깃 확정 상태를 Acoustic에 전달한다.
  void publish_target_confirmed(bool confirmed)
  {
    std_msgs::msg::Bool msg;
    msg.data = confirmed;
    target_confirmed_pub_->publish(msg);
  }

  static const char * state_name(State state)
  {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::TARGET_CONFIRM: return "TARGET_CONFIRM";
      case State::WAIT_CONTROL_GRANT: return "WAIT_CONTROL_GRANT";
      case State::SEARCH: return "SEARCH";
      case State::APPROACH_BUOY: return "APPROACH_BUOY";
      case State::ALIGN_STICK: return "ALIGN_STICK";
      case State::INSERT_FORK: return "INSERT_FORK";
      case State::DETACH: return "DETACH";
      case State::BACKOFF: return "BACKOFF";
      case State::VERIFY_RELEASE: return "VERIFY_RELEASE";
      case State::AREA_VERIFY: return "AREA_VERIFY";
      case State::ASCEND: return "ASCEND";
      case State::COMPLETE: return "COMPLETE";
      case State::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  // MAVROS OverrideRCIn: CHAN_NOCHANGE로 채운 뒤 제어할 채널만 덮어쓴다.
  std::array<uint16_t, 18> nochange_channels() const
  {
    std::array<uint16_t, 18> channels{};
    channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    return channels;
  }

  void set_neutral_control(std::array<uint16_t, 18> & channels)
  {
    set_channel(channels, throttle_channel_, neutral_pwm_);
    set_channel(channels, yaw_channel_, neutral_pwm_);
    set_channel(channels, forward_channel_, neutral_pwm_);
    set_channel(channels, lateral_channel_, neutral_pwm_);
  }

  // RC override를 끊고 해당 채널을 수동/다른 소스에 반환.
  void release_controlled_channels(std::array<uint16_t, 18> & channels)
  {
    set_channel(channels, throttle_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, yaw_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, forward_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, lateral_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }

  void set_channel(std::array<uint16_t, 18> & channels, int channel, int pwm) const
  {
    if (
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE &&
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE)
    {
      const int deadzone = channel == yaw_channel_ ? yaw_deadzone_ :
        channel == throttle_channel_ ? heave_deadzone_ : translation_deadzone_;
      pwm = neutral_pwm_ + auv_buoy_vision_control::compensate_rc_deadzone(
        pwm-neutral_pwm_, deadzone);
      pwm = std::clamp(pwm, min_pwm_, max_pwm_);
    }
    channels[static_cast<size_t>(channel - 1)] = static_cast<uint16_t>(pwm);
  }

  void publish_channels(const std::array<uint16_t, 18> & channels)
  {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels = channels;
    rc_pub_->publish(msg);
    rc_monitor_pub_->publish(msg);
  }

  // --- 파라미터 캐시 ---
  std::string bbox_topic_;
  std::string depth_pose_topic_;
  double depth_pose_scale_{-1.0};
  double depth_pose_offset_m_{0.0};
  // [ACOUSTIC-VISION HANDSHAKE] 제어권 요청/타깃 확인/최종 승인 토픽
  std::string vision_search_request_topic_;
  std::string target_confirmed_topic_;
  std::string vision_control_granted_topic_;
  std::string state_topic_;
  std::string rc_override_topic_;
  std::string rc_monitor_topic_;
  double control_rate_hz_{20.0};
  double detection_timeout_sec_{1.2};
  double depth_timeout_sec_{1.0};
  double surface_depth_m_{0.4};
  double max_depth_m_{10.5};
  double depth_tolerance_m_{0.2};
  double depth_kp_{0.3};
  double depth_ki_{0.05};
  double depth_kd_{0.04};
  double depth_bias_{0.1};
  double depth_integral_limit_{2.0};
  double heave_limit_{0.4};
  int max_depth_delta_pwm_{160};
  double lpf_tau_sec_{0.3};
  int buoy_class_id_{0};
  int stick_class_id_{1};
  int min_detection_hits_{3};
  int target_confirm_hits_{3};
  double target_confirm_sec_{0.2};
  double approach_area_ratio_{0.30};
  double search_timeout_sec_{40.0};
  double area_verify_sec_{12.0};
  double buoy_area_similar_ratio_{0.15};
  double buoy_confidence_similar_delta_{0.05};
  double buoy_same_target_center_ratio_{0.12};
  double fork_target_x_{0.30};
  double fork_target_y_{0.70};
  double stick_deadband_x_{0.06};
  double stick_deadband_y_{0.08};
  double align_stable_sec_{0.7};
  int insert_pwm_{1560};
  double insert_duration_sec_{0.8};
  int detach_pwm_{1620};
  double detach_duration_sec_{0.3};
  int backoff_pwm_{1420};
  double backoff_duration_sec_{0.5};
  double verify_clear_sec_{1.0};
  double verify_timeout_sec_{3.0};
  int max_target_retries_{2};
  int throttle_channel_{3};
  int yaw_channel_{4};
  int forward_channel_{5};
  int neutral_pwm_{1500};
  int min_pwm_{1300};
  int max_pwm_{1700};
  int max_yaw_delta_{60};
  int max_tracking_depth_delta_{100};
  int approach_forward_pwm_{1540};
  int approach_forward_min_pwm_{1510};
  double approach_vision_throttle_weight_{0.4};
  int search_yaw_pwm_{1530};
  int search_forward_pwm_{1520};
  int search_detection_brake_yaw_pwm_{1450};
  double search_detection_hold_sec_{2.0};
  bool yaw_invert_{false};
  bool vertical_positive_is_up_{true};

  double search_depth_min_{0.3}, search_depth_max_{2.0}, search_depth_rate_{0.05};
  double visual_depth_rate_{0.06}, previous_depth_measurement_m_{0.0};
  double visual_depth_gain_{0.7}, hand_target_x_{0.45}, hand_target_y_{0.60};
  double hand_handoff_height_{0.12}, approach_target_x_{0.45};
  double hand_right_x_{1.0}, hand_right_y_{0.0}, hand_depth_x_{0.0}, hand_depth_y_{1.0};
  bool hand_acquire_{false};
  double hand_acquire_height_{0.22}, hand_acquire_front_x_{0.85};
  double hand_insert_height_{0.25}, control_dt_{0.0}, last_control_s_{0.0};
  int search_depth_direction_{1}, lateral_channel_{6};
  int yaw_deadzone_{40}, translation_deadzone_{30}, heave_deadzone_{30};
  std::optional<Detection> hand_stick_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_bbox_sub_;
  // --- 런타임 상태 ---
  bool vision_has_control_{false};
  State state_{State::IDLE};
  rclcpp::Time state_entered_at_{0, 0, RCL_ROS_TIME};
  // ALIGN deadband 유지 등 "조건 지속 시간" 측정용
  std::optional<rclcpp::Time> condition_started_at_;
  std::optional<rclcpp::Time> target_confirm_started_at_;
  std::optional<rclcpp::Time> search_detection_hold_started_at_;
  bool search_detection_brake_pending_{false};
  std::optional<double> depth_m_;
  rclcpp::Time depth_received_at_{0, 0, RCL_ROS_TIME};
  std::optional<double> mission_hold_depth_m_;
  bool depth_pid_initialized_{false};
  rclcpp::Time last_depth_control_time_{0, 0, RCL_ROS_TIME};
  double previous_depth_error_m_{0.0};
  double depth_error_integral_{0.0};
  std::optional<Detection> buoy_;
  std::optional<Detection> stick_;
  int target_retries_{0};

  // --- ROS 인터페이스 ---
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_search_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_control_granted_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_monitor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_confirmed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionStateMachineNode>();
  rclcpp::spin(node);
  node->publish_release_once();  // 종료 시 RC override 해제
  rclcpp::shutdown();
  return 0;
}
