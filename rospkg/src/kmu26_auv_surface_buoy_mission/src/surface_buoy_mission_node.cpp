#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <auv_msg/msg/collector_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "kmu26_auv_surface_buoy_mission/arena_frame_transform.hpp"
#include "kmu26_auv_surface_buoy_mission/depth_p_controller.hpp"
#include "kmu26_auv_surface_buoy_mission/dump_cycle.hpp"
#include "kmu26_auv_surface_buoy_mission/dump_motion.hpp"
#include "kmu26_auv_surface_buoy_mission/lane_planner.hpp"
#include "kmu26_auv_surface_buoy_mission/vision_gate.hpp"
#include "kmu26_auv_surface_buoy_mission/yaw_control.hpp"

namespace kmu26_auv_surface_buoy_mission
{
namespace
{
// /mission/surface_start는 가벼운 문자열 계약을 사용한다.
// 예: "cycle=2;final=false;work_depth=0.85"
// 아래 두 함수는 이 문자열에서 필요한 필드만 꺼낸다.
uint32_t parse_uint_field(const std::string & text, const std::string & key)
{
  const auto begin = text.find(key + "=");
  if (begin == std::string::npos) {
    return 0;
  }
  return static_cast<uint32_t>(std::stoul(text.substr(begin + key.size() + 1)));
}

double parse_double_field(const std::string & text, const std::string & key, double fallback)
{
  const auto begin = text.find(key + "=");
  if (begin == std::string::npos) {
    return fallback;
  }
  return std::stod(text.substr(begin + key.size() + 1));
}
}  // namespace

/**
 * @brief 수면 부표 수집과 가점존 배출을 담당하는 ROS 2 상태 머신 노드.
 *
 * 전체 임무 흐름
 *   1. 레인 노드에서 /mission/surface_start를 받고 RC 제어권을 넘겨받는다.
 *   2. 수집 수심으로 상승한 뒤 수면 레인을 순회하며 정면 카메라로 부표를 찾는다.
 *   3. 정면 bbox를 화면 중심에 맞추고 전진한다. 부표가 정면 영상의 아래쪽에서
 *      상단 카메라 영역으로 넘어가거나 CollectorState가 netted=true이면 포획으로 센다.
 *   4. 포획 후에도 같은 레인의 같은 끝점을 향해 주행하고, 끝점에서만 레인을 완료한다.
 *   5. 완료한 레인에서 부표를 잡았을 때만 가점존으로 이동해 최대 3회 배출한다.
 *   6. 미완료 레인이 있으면 수집 수심으로 돌아가 다음 레인을 처리한다.
 *   7. 모든 레인을 완료한 뒤 작업 수심으로 복귀해 /mission/surface_complete를 발행한다.
 *
 * 이 노드는 물리적인 부표 삭제를 직접 수행하지 않는다. /mission/score_release에
 * RELEASE 계약을 발행하고, 시뮬레이터가 위치 조건과 계약을 함께 검사해 배출을 확정한다.
 */
class SurfaceBuoyMissionNode : public rclcpp::Node
{
public:
  SurfaceBuoyMissionNode()
  : Node("surface_buoy_mission_node")
  {
    declare_parameters();
    create_interfaces();
    state_entered_at_ = now();
    publish_state();
    publish_counts();
    RCLCPP_INFO(
      get_logger(),
      "Surface mission ready: external vision bbox topics front=%s top=%s",
      front_bbox_topic_.c_str(), top_bbox_topic_.c_str());
  }

  void publish_release_once()
  {
    // 노드 종료나 미션 완료 시 RC override를 한 번만 해제한다. 반복 해제 메시지로
    // 다음 제어 노드의 출력을 덮지 않기 위한 보호 장치다.
    if (!active_ || release_sent_) {
      return;
    }
    auto channels = nochange_channels();
    release_channels(channels);
    publish_channels(channels);
    release_sent_ = true;
  }

private:
  // 상태 전이는 on_timer()에서 20 Hz로 실행된다. 각 상태는 RC 채널 3(상하),
  // 4(yaw), 5(전후)에 필요한 값만 기록한다.
  using State = SurfaceMissionState;

  // YOLO 노드가 보내는 한 개 bbox를 내부에서 사용하기 편한 형태로 보관한다.
  // 좌표와 크기는 픽셀 단위이며 received로 검출 데이터의 신선도를 판단한다.
  struct Detection
  {
    double cx{0.0};
    double cy{0.0};
    double width{0.0};
    double height{0.0};
    double image_width{0.0};
    double image_height{0.0};
    double confidence{0.0};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
  };

  void declare_parameters()
  {
    // 입력·출력 토픽 계약. launch에서 바꿀 수 있지만 기본값은 시뮬레이터와 일치한다.
    front_bbox_topic_ = declare_parameter<std::string>(
      "surface_front_bbox_topic", "/vision/surface/front/buoy_bbox");
    top_bbox_topic_ = declare_parameter<std::string>(
      "surface_top_bbox_topic", "/vision/surface/top/buoy_bbox");
    front_enable_topic_ = declare_parameter<std::string>(
      "surface_front_enable_topic", "/vision/surface/front/enabled");
    top_enable_topic_ = declare_parameter<std::string>(
      "surface_top_enable_topic", "/vision/surface/top/enabled");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/homing/sim_odom");
    depth_pose_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    start_frame_topic_ = declare_parameter<std::string>("start_frame_topic", "/start_frame");
    surface_start_topic_ = declare_parameter<std::string>("surface_start_topic", "/mission/surface_start");
    surface_complete_topic_ = declare_parameter<std::string>("surface_complete_topic", "/mission/surface_complete");
    arena_config_topic_ = declare_parameter<std::string>("arena_config_topic", "/mission/arena_config");
    score_release_topic_ = declare_parameter<std::string>("score_release_topic", "/mission/score_release");
    rc_topic_ = declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");

    // 가점존은 arena safe bounds 중앙에서의 offset으로 계산한다.
    bonus_center_offset_x_m_ = declare_parameter<double>("bonus_zone_center_offset_x_m", 1.95);
    bonus_center_offset_y_m_ = declare_parameter<double>("bonus_zone_center_offset_y_m", 0.0);
    bonus_radius_m_ = declare_parameter<double>("bonus_zone_radius_m", 0.65);
    bonus_lane_clearance_m_ = declare_parameter<double>("bonus_zone_lane_clearance_m", 0.35);
    bonus_approach_distance_m_ = declare_parameter<double>("bonus_approach_distance_m", 1.0);
    bonus_dump_heading_rad_ = declare_parameter<double>("bonus_dump_heading_rad", 0.0);
    score_zone_world_z_m_ = declare_parameter<double>("score_zone_world_z_m", -0.30);
    collection_depth_m_ = declare_parameter<double>("collection_depth_m", 0.30);
    dump_depth_m_ = declare_parameter<double>("dump_depth_m", 0.85);
    depth_tolerance_m_ = declare_parameter<double>("depth_tolerance_m", 0.08);

    min_surface_lane_segment_length_m_ = declare_parameter<double>(
      "min_surface_lane_segment_length_m", 1.0);
    // 레인별 가점존 배출은 항상 전진 overshoot 후 즉시 급후진한다.
    surface_total_buoy_count_ = declare_parameter<int>("surface_total_buoy_count", 5);
    max_dump_attempts_ = declare_parameter<int>("max_dump_attempts", 3);
    dump_forward_overshoot_m_ = declare_parameter<double>("dump_forward_overshoot_m", 0.70);
    dump_reverse_distance_m_ = declare_parameter<double>("dump_reverse_distance_m", 1.10);
    dump_forward_timeout_s_ = declare_parameter<double>("dump_forward_timeout_sec", 4.0);
    dump_reverse_timeout_s_ = declare_parameter<double>("dump_reverse_timeout_sec", 4.0);
    dump_settle_s_ = declare_parameter<double>("dump_settle_sec", 0.7);
    capture_forward_s_ = declare_parameter<double>("capture_forward_sec", 2.0);
    capture_ignore_s_ = declare_parameter<double>("capture_ignore_sec", 0.8);

    // 상단 카메라 ROI와 시간 필터. 단일 프레임 오검출로 망 상태가 뒤집히지 않게 한다.
    top_roi_x_min_ = declare_parameter<double>("top_net_roi_x_min", 0.10);
    top_roi_x_max_ = declare_parameter<double>("top_net_roi_x_max", 0.90);
    top_roi_y_min_ = declare_parameter<double>("top_net_roi_y_min", 0.05);
    top_roi_y_max_ = declare_parameter<double>("top_net_roi_y_max", 0.95);
    top_confirm_s_ = declare_parameter<double>("top_occupied_confirm_sec", 0.5);
    top_empty_confirm_s_ = declare_parameter<double>("top_empty_confirm_sec", 0.5);
    detection_timeout_s_ = declare_parameter<double>("top_detection_timeout_sec", 1.5);
    front_min_bbox_height_ratio_ = declare_parameter<double>(
      "surface_front_min_bbox_height_ratio", 0.03);
    capture_min_bbox_height_ratio_ = declare_parameter<double>(
      "surface_capture_min_bbox_height_ratio", 0.10);
    align_deadband_x_ = declare_parameter<double>("surface_align_deadband_x", 0.07);
    align_stable_s_ = declare_parameter<double>("surface_align_stable_sec", 0.4);

    // MAVROS RC override 제어 이득과 PWM 제한값.
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    depth_kp_pwm_per_m_ = declare_parameter<double>("surface_depth_kp_pwm_per_m", 300.0);
    max_depth_delta_pwm_ = declare_parameter<int>("surface_max_depth_delta_pwm", 180);
    neutral_pwm_ = declare_parameter<int>("neutral_pwm", 1500);
    min_pwm_ = declare_parameter<int>("min_pwm", 1300);
    max_pwm_ = declare_parameter<int>("max_pwm", 1700);
    search_forward_pwm_ = declare_parameter<int>("surface_search_forward_pwm", 1600);
    capture_forward_pwm_ = declare_parameter<int>("surface_capture_forward_pwm", 1650);
    dump_forward_pwm_ = declare_parameter<int>("dump_forward_pwm", 1660);
    dump_reverse_pwm_ = declare_parameter<int>("dump_reverse_pwm", 1340);
    waypoint_yaw_kp_pwm_per_rad_ = declare_parameter<double>(
      "surface_waypoint_yaw_kp_pwm_per_rad", 450.0);
    align_yaw_kp_pwm_per_normalized_x_ = declare_parameter<double>(
      "surface_align_yaw_kp_pwm_per_normalized_x", 500.0);
    yaw_command_deadband_rad_ = declare_parameter<double>(
      "surface_yaw_command_deadband_rad", 0.03);
    min_yaw_delta_pwm_ = declare_parameter<int>("surface_min_yaw_delta_pwm", 55);
    max_yaw_delta_pwm_ = declare_parameter<int>("surface_max_yaw_delta_pwm", 200);
    waypoint_tolerance_m_ = declare_parameter<double>("surface_waypoint_tolerance_m", 0.25);
    heading_tolerance_rad_ = declare_parameter<double>("surface_heading_tolerance_rad", 0.20);
    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);

    if (
      bonus_radius_m_ <= 0.0 || bonus_lane_clearance_m_ < 0.0 ||
      min_surface_lane_segment_length_m_ <= 0.0 ||
      dump_forward_overshoot_m_ <= 0.0 || dump_reverse_distance_m_ <= 0.0 ||
      dump_forward_timeout_s_ <= 0.0 || dump_reverse_timeout_s_ <= 0.0 ||
      front_min_bbox_height_ratio_ < 0.0 || front_min_bbox_height_ratio_ >= 1.0 ||
      capture_min_bbox_height_ratio_ <= front_min_bbox_height_ratio_ ||
      capture_min_bbox_height_ratio_ >= 1.0 ||
      align_deadband_x_ <= 0.0 || align_deadband_x_ >= 0.5 ||
      waypoint_yaw_kp_pwm_per_rad_ <= 0.0 || align_yaw_kp_pwm_per_normalized_x_ <= 0.0 ||
      yaw_command_deadband_rad_ < 0.0 || min_yaw_delta_pwm_ < 0 ||
      min_yaw_delta_pwm_ > max_yaw_delta_pwm_ ||
      max_yaw_delta_pwm_ > std::min(max_pwm_ - neutral_pwm_, neutral_pwm_ - min_pwm_) ||
      max_dump_attempts_ < 1)
    {
      throw std::invalid_argument("surface lane or dump parameters are invalid");
    }
  }

  void create_interfaces()
  {
    // 임무 시작 전에 발행된 시작 좌표·대회장 설정도 새 구독자가 받을 수 있도록
    // transient_local QoS를 사용한다.
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    front_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      front_bbox_topic_, 10,
      std::bind(&SurfaceBuoyMissionNode::on_front_bbox, this, std::placeholders::_1));
    top_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      top_bbox_topic_, 10, std::bind(&SurfaceBuoyMissionNode::on_top_bbox, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 30, std::bind(&SurfaceBuoyMissionNode::on_odometry, this, std::placeholders::_1));
    depth_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      depth_pose_topic_, 10, std::bind(&SurfaceBuoyMissionNode::on_depth, this, std::placeholders::_1));
    start_frame_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      start_frame_topic_, latched, std::bind(&SurfaceBuoyMissionNode::on_start_frame, this, std::placeholders::_1));
    arena_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      arena_config_topic_, latched, std::bind(&SurfaceBuoyMissionNode::on_arena_config, this, std::placeholders::_1));
    start_sub_ = create_subscription<std_msgs::msg::String>(
      surface_start_topic_, latched, std::bind(&SurfaceBuoyMissionNode::on_surface_start, this, std::placeholders::_1));
    collector_sub_ = create_subscription<auv_msg::msg::CollectorState>(
      "/collector/state", 30, std::bind(&SurfaceBuoyMissionNode::on_collector_state, this, std::placeholders::_1));

    // 상태와 누적 개수는 GUI/감시 노드가 늦게 연결돼도 현재값을 알 수 있게 latched로 발행한다.
    state_pub_ = create_publisher<std_msgs::msg::String>("/mission/surface_state", latched);
    complete_pub_ = create_publisher<std_msgs::msg::String>(surface_complete_topic_, 10);
    score_release_pub_ = create_publisher<std_msgs::msg::String>(score_release_topic_, 10);
    remaining_pub_ = create_publisher<std_msgs::msg::UInt32>("/mission/surface_remaining_count", latched);
    deposit_pub_ = create_publisher<std_msgs::msg::UInt32>("/mission/bonus_deposit_count", latched);
    front_enable_pub_ = create_publisher<std_msgs::msg::Bool>(front_enable_topic_, latched);
    top_enable_pub_ = create_publisher<std_msgs::msg::Bool>(top_enable_topic_, latched);

    // /clock을 쓰는 2배속 검증에서도 차량 기준 20 Hz를 유지한다. 실물처럼
    // use_sim_time=false이면 같은 타이머가 시스템 시간을 사용한다.
    timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(1.0 / std::max(1.0, control_rate_hz_)),
      std::bind(&SurfaceBuoyMissionNode::on_timer, this));
  }

  void on_arena_config(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 7 || lane_planner_) {
      return;
    }
    ArenaConfig config;
    config.length_m = msg->data[0];
    config.width_m = msg->data[1];
    config.offset_x_m = msg->data[2];
    config.offset_y_m = msg->data[3];
    config.safety_margin_m = msg->data[4];
    config.lane_search_offset_m = msg->data[5];
    config.start_corner = msg->data[6] > 0.5 ? "bottom_right" : "bottom_left";
    config.lane_orientation = LaneOrientation::VERTICAL;

    // 먼저 제외영역 없는 planner로 safe bounds를 얻고 중앙+offset 가점존을 계산한다.
    const LanePlanner bounds_planner(config);
    const auto & bounds = bounds_planner.safe_bounds();
    bonus_center_.x = 0.5 * (bounds.x_min + bounds.x_max) + bonus_center_offset_x_m_;
    bonus_center_.y = 0.5 * (bounds.y_min + bounds.y_max) + bonus_center_offset_y_m_;
    const double exclusion_radius = bonus_radius_m_ + bonus_lane_clearance_m_;

    // 가점존과 lane clearance 전체가 공용 안전 경계 안에 들어오는지 검증한다.
    // 좌표 계약이 틀린 상태에서 차량을 움직이는 것보다 즉시 실패시키는 편이 안전하다.
    if (
      bonus_center_.x - exclusion_radius < bounds.x_min ||
      bonus_center_.x + exclusion_radius > bounds.x_max ||
      bonus_center_.y - exclusion_radius < bounds.y_min ||
      bonus_center_.y + exclusion_radius > bounds.y_max)
    {
      throw std::invalid_argument(
              "bonus-zone circle and lane clearance are outside the shared arena safe bounds");
    }

    config.circular_exclusion_enabled = true;
    config.circular_exclusion_center = bonus_center_;
    config.circular_exclusion_radius_m = exclusion_radius;
    config.min_lane_segment_length_m = min_surface_lane_segment_length_m_;
    lane_planner_ = std::make_unique<LanePlanner>(config);
    surface_lane_completed_.assign(lane_planner_->lanes().size(), false);
    RCLCPP_INFO(
      get_logger(), "Vertical surface lanes=%zu, score=(%.3f, %.3f), exclusion_radius=%.2f",
      lane_planner_->lanes().size(), bonus_center_.x, bonus_center_.y, exclusion_radius);
  }

  void on_start_frame(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (arena_transform_.initialized()) {
      return;
    }
    const double yaw = ArenaFrameTransform::yaw_from_quaternion(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);
    // 이후 waypoint와 가점존 계산은 모두 대회장 좌표계로 통일한다.
    arena_transform_.initialize({msg->pose.position.x, msg->pose.position.y}, yaw);
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!arena_transform_.initialized()) {
      return;
    }
    current_position_ = arena_transform_.position_from_odom(
      {msg->pose.pose.position.x, msg->pose.pose.position.y});
    const double odom_yaw = ArenaFrameTransform::yaw_from_quaternion(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    current_yaw_rad_ = arena_transform_.yaw_from_odom(odom_yaw);
    have_odometry_ = true;
    odometry_at_ = now();
  }

  void on_depth(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const double depth = -msg->pose.pose.position.z;
    if (std::isfinite(depth) && depth >= 0.0) {
      depth_m_ = depth;
      depth_at_ = now();
    }
  }

  std::vector<Detection> detections(const std_msgs::msg::Float32MultiArray & msg) const
  {
    // YOLO bbox 배열은 검출 하나당 10개 값이다. confidence/class/image size가
    // 유효한 부표만 골라 Detection 목록으로 변환한다.
    std::vector<Detection> rows;
    for (std::size_t base = 0; base + 9 < msg.data.size(); base += 10) {
      if (
        msg.data[base + 1] < 0.5F ||
        static_cast<int>(std::lround(msg.data[base + 2])) != buoy_class_id_ ||
        msg.data[base + 8] <= 0.0F || msg.data[base + 9] <= 0.0F)
      {
        continue;
      }
      rows.push_back(
        {msg.data[base + 4], msg.data[base + 5], msg.data[base + 6], msg.data[base + 7],
          msg.data[base + 8], msg.data[base + 9], msg.data[base + 3], now()});
    }
    return rows;
  }

  void on_front_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    auto rows = detections(*msg);
    rows.erase(
      std::remove_if(
        rows.begin(), rows.end(), [this](const Detection & row) {
          return row.height / row.image_height < front_min_bbox_height_ratio_;
        }),
      rows.end());
    if (rows.empty()) {
      front_.reset();
      return;
    }
    // 3D active-lane 필터는 후속 예외처리 설계 전까지 적용하지 않는다.
    // 기존 동작대로 화면에서 가장 큰 부표 bbox를 현재 접근 대상으로 선택한다.
    front_ = *std::max_element(
      rows.begin(), rows.end(), [](const Detection & a, const Detection & b) {
        return a.width * a.height < b.width * b.height;
      });
  }

  void on_top_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto rows = detections(*msg);
    bool occupied = false;
    for (const auto & row : rows) {
      const double x0 = (row.cx - 0.5 * row.width) / row.image_width;
      const double x1 = (row.cx + 0.5 * row.width) / row.image_width;
      const double y0 = (row.cy - 0.5 * row.height) / row.image_height;
      const double y1 = (row.cy + 0.5 * row.height) / row.image_height;
      // 초기 점검에서는 화면 전체를 보고, 실제 수집/배출 중에는 수집망 ROI와
      // 겹치는 bbox만 망 내부 부표로 인정한다.
      if (state_ == State::INITIAL_TOP_CHECK ||
        (x1 >= top_roi_x_min_ && x0 <= top_roi_x_max_ &&
        y1 >= top_roi_y_min_ && y0 <= top_roi_y_max_))
      {
        occupied = true;
        break;
      }
    }
    top_occupied_ = occupied;
    top_received_at_ = now();
    if (occupied) {
      if (!top_occupied_since_) {
        top_occupied_since_ = now();
      }
      top_empty_since_.reset();
    } else {
      if (!top_empty_since_) {
        top_empty_since_ = now();
      }
      top_occupied_since_.reset();
    }
  }

  void on_collector_state(const auv_msg::msg::CollectorState::SharedPtr msg)
  {
    collector_state_received_ = true;
    collector_physically_occupied_ =
      msg->collector_eq_active || msg->capture_state == "NETTING" ||
      msg->capture_state == "NETTED";
    // target_id 집합으로 같은 부표 이벤트가 여러 번 들어와도 한 번만 센다.
    // 시뮬레이터의 물리 접촉 판정은 카메라 추정보다 우선하는 확정 신호다.
    if (msg->netted && !msg->target_id.empty() && captured_ids_.insert(msg->target_id).second) {
      if (active_) {
        ++lane_capture_count_;
        capture_confirmed_ = true;
      }
      publish_counts();
    }
    if (msg->released && !msg->target_id.empty() && deposited_ids_.insert(msg->target_id).second) {
      publish_counts();
    }
  }

  void on_surface_start(const std_msgs::msg::String::SharedPtr msg)
  {
    const uint32_t cycle = parse_uint_field(msg->data, "cycle");
    if (cycle == 0 || cycle <= last_completed_cycle_ || cycle == cycle_id_) {
      return;
    }
    // arena/start_frame이 늦게 도착하면 시작 계약을 버리지 않고 보류한다.
    if (!lane_planner_ || !arena_transform_.initialized()) {
      RCLCPP_WARN(get_logger(), "Surface start deferred: shared arena/start frame unavailable");
      pending_start_ = msg->data;
      return;
    }
    begin_cycle(msg->data);
  }

  void begin_cycle(const std::string & contract)
  {
    // 새 cycle마다 순간 상태만 초기화한다. captured_ids_/deposited_ids_는
    // 전체 미션 누적 점수이므로 지우지 않는다.
    cycle_id_ = parse_uint_field(contract, "cycle");
    final_cycle_ = contract.find("final=true") != std::string::npos;
    work_depth_m_ = parse_double_field(contract, "work_depth", dump_depth_m_);
    lane_capture_count_ = 0;
    dump_attempt_count_ = 0;
    surface_lane_completed_.assign(lane_planner_->lanes().size(), false);
    active_surface_lane_.reset();
    front_.reset();
    capture_confirmed_ = false;
    release_sent_ = false;
    active_ = true;
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_topic_, 10);
    close_score_gate();
    transition(State::ASCEND_TO_COLLECTION_DEPTH, "lane RC handoff received");
  }

  void on_timer()
  {
    if (!pending_start_.empty() && lane_planner_ && arena_transform_.initialized()) {
      const std::string pending = pending_start_;
      pending_start_.clear();
      begin_cycle(pending);
    }
    if (!active_) {
      return;
    }
    if (!have_odometry_ || !depth_m_) {
      return;
    }
    // 매 주기 중립값에서 시작하여 현재 상태가 필요한 채널만 덮어쓴다.
    // 이렇게 하면 이전 상태의 PWM이 다음 상태에 남지 않는다.
    auto channels = nochange_channels();
    set_neutral(channels);
    switch (state_) {
      case State::IDLE: return;
      case State::ASCEND_TO_COLLECTION_DEPTH: run_ascend(channels); break;
      case State::INITIAL_TOP_CHECK: run_initial_top_check(channels); break;
      case State::SURFACE_SEARCH: run_search(channels); break;
      case State::SURFACE_ALIGN: run_align(channels); break;
      case State::SURFACE_CAPTURE: run_capture(channels); break;
      case State::MOVE_TO_BONUS: run_move_bonus(channels); break;
      case State::DESCEND_TO_DUMP_DEPTH: run_descend_dump(channels); break;
      case State::MOVE_TO_BONUS_CENTER: run_bonus_center(channels); break;
      case State::DUMP_EJECT: run_dump_eject(channels); break;
      case State::DUMP_CHECK: run_dump_check(channels); break;
      case State::RETURN_TO_BONUS_CENTER: run_return_center(channels); break;
      case State::RETURN_TO_WORK_DEPTH: run_return_depth(channels); break;
      case State::FAILSAFE: fail_cycle("surface failsafe"); return;
    }
    publish_channels(channels);
  }

  void run_ascend(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, collection_depth_m_);
    if (std::abs(*depth_m_ - collection_depth_m_) <= depth_tolerance_m_) {
      if (!initial_top_check_done_) {
        transition(State::INITIAL_TOP_CHECK, "collection depth reached");
      } else {
        begin_search("collection depth reached");
      }
    }
  }

  void run_initial_top_check(std::array<uint16_t, 18> & channels)
  {
    // 요구사항상 시작할 때 망이 비었으면 대기하지 않고 바로 레인 탐색으로 간다.
    hold_depth(channels, collection_depth_m_);
    if (recent_top() || state_age() >= detection_timeout_s_) {
      initial_top_check_done_ = true;
      begin_search(top_occupied_ ? "initial top buoy visible" : "initial top check complete");
    }
  }

  void begin_search(const std::string & reason)
  {
    select_surface_lane();
    if (!active_surface_lane_) {
      transition(State::RETURN_TO_WORK_DEPTH, "all surface lanes complete");
      return;
    }
    transition(State::SURFACE_SEARCH, reason);
  }

  void resume_current_lane(const std::string & reason)
  {
    // 정렬/포획 상태로 빠질 때 active_surface_lane_, 진행 방향, 목표 끝점은 바꾸지 않았다.
    // 따라서 새 레인을 선택하지 않고 SURFACE_SEARCH로만 돌아가면 기존 경로가 그대로 이어진다.
    if (!active_surface_lane_) {
      transition(State::RETURN_TO_WORK_DEPTH, "surface lane unavailable");
      return;
    }
    front_.reset();
    transition(State::SURFACE_SEARCH, reason);
  }

  void select_surface_lane()
  {
    // 현재 위치에서 가장 가까운 미완료 레인의 끝점을 선택해 불필요한 이동을 줄인다.
    const auto choice = lane_planner_->closest_uncompleted_endpoint(
      current_position_, surface_lane_completed_);
    if (!choice) {
      active_surface_lane_.reset();
      return;
    }
    active_surface_lane_ = choice->lane_index;
    surface_lane_start_ = choice->start;
    surface_lane_finish_ = choice->finish;
    surface_heading_to_start_ = true;
    lane_capture_count_ = 0;
  }

  void run_search(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, collection_depth_m_);
    // 레인 시작점으로 이동하는 동안 보이는 부표는 다른 레인에 있을 수 있다.
    // 3D active-lane 필터는 추후 적용하되, 최소한 시작점에 도착해 끝점 방향으로
    // 탐색을 시작한 뒤에만 정면 부표 추적으로 전환한다.
    if (!surface_heading_to_start_ && recent_front() && state_age() >= capture_ignore_s_) {
      align_started_at_.reset();
      transition(State::SURFACE_ALIGN, "front buoy acquired");
      return;
    }
    if (!active_surface_lane_) {
      transition(State::RETURN_TO_WORK_DEPTH, "all surface lanes complete");
      return;
    }
    // 한 레인의 시작점에 먼저 간 뒤 반대 끝까지 주행하면 해당 레인 검색을 완료한 것으로 본다.
    const Vec2 target = surface_heading_to_start_ ? surface_lane_start_ : surface_lane_finish_;
    if (follow_waypoint(channels, target, search_forward_pwm_, collection_depth_m_)) {
      if (surface_heading_to_start_) {
        surface_heading_to_start_ = false;
        // 이제 레인 끝점 방향 탐색이 시작되므로 정면 detector만 켠다.
        publish_vision_gates();
      } else {
        surface_lane_completed_[*active_surface_lane_] = true;
        finish_current_lane();
      }
    }
  }

  void run_align(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, collection_depth_m_);
    if (!recent_front()) {
      resume_current_lane("front target lost; resume current lane");
      return;
    }
    // 정면 영상 중심 x=0.5를 기준으로 yaw를 보정한다. 수면 부표는 카메라보다 위에
    // 있어 접근할수록 bbox가 화면 위로 이동하므로, 중심선이 안정되면 포획 전진을
    // 시작하고 이후 정면→상단 영상 전이 또는 물리 CollectorState로 성공을 확정한다.
    const double nx = front_->cx / front_->image_width;
    const double error = 0.5 - nx;
    const double bbox_height_ratio = front_->height / front_->image_height;
    const int yaw_delta = yaw_delta_pwm(
      error,
      YawControlConfig{
        align_yaw_kp_pwm_per_normalized_x_, align_deadband_x_,
        min_yaw_delta_pwm_, max_yaw_delta_pwm_});
    // ArduSub RC yaw는 PWM 증가가 arena/world 음의 yaw이므로 기하학적 오차와 부호가 반대다.
    set_channel(channels, 4, neutral_pwm_ - yaw_delta);
    if (std::abs(error) <= align_deadband_x_) {
      // 중심이 맞는 동안에는 낮은 탐색 PWM으로 천천히 접근한다. 실측상 먼 부표는
      // bbox 높이가 약 3%, 포획권에서는 11~12%였으므로 10% 이전에는 포획 상태로
      // 넘어가지 않는다.
      set_channel(channels, 5, search_forward_pwm_);
    }
    if (capture_entry_ready(
        error, align_deadband_x_, bbox_height_ratio, capture_min_bbox_height_ratio_))
    {
      if (!align_started_at_) {
        align_started_at_ = now();
      }
      if ((now() - *align_started_at_).seconds() >= align_stable_s_) {
        capture_confirmed_ = false;
        capture_front_start_y_ = front_->cy / front_->image_height;
        capture_front_min_y_ = capture_front_start_y_;
        transition(State::SURFACE_CAPTURE, "front centerline stable");
      }
    } else {
      align_started_at_.reset();
    }
  }

  void run_capture(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, collection_depth_m_);
    set_channel(channels, 5, capture_forward_pwm_);
    if (recent_front()) {
      // 포획 전진 중에도 정면 bbox가 남아 있는 동안에는 yaw를 계속 보정한다.
      // 포획 상태 진입 순간의 heading을 고정하면 작은 횡오차가 수집망 폭보다
      // 커져 부표 옆을 지나칠 수 있다.
      const double error = 0.5 - front_->cx / front_->image_width;
      const int yaw_delta = yaw_delta_pwm(
        error,
        YawControlConfig{
          align_yaw_kp_pwm_per_normalized_x_, align_deadband_x_,
          min_yaw_delta_pwm_, max_yaw_delta_pwm_});
      set_channel(channels, 4, neutral_pwm_ - yaw_delta);
      capture_front_min_y_ = std::min(capture_front_min_y_, front_->cy / front_->image_height);
    }
    // 포획 성공은 두 경로로 확정한다.
    // 1) CollectorState의 실제 netted 이벤트
    // 2) 정면 bbox가 위로 이동한 뒤 상단 카메라에서 연속 확인되는 영상 전이
    const bool front_moved_up = capture_front_start_y_ - capture_front_min_y_ >= 0.08;
    const bool transitioned_to_top = front_moved_up && top_occupied_confirmed();
    // CollectorState가 제공되는 시뮬레이션/실물 구성에서는 영상 전이만으로
    // 포획을 확정하지 않는다. 기체가 부표 위를 지나치기만 해도 상단 영상에는
    // 보이므로 반드시 물리 netted 이벤트를 기다린다. CollectorState 자체가 없는
    // 구성에서만 기존 영상 전이를 보조 계약으로 사용한다.
    const bool vision_capture_fallback = !collector_state_received_ && transitioned_to_top;
    if (capture_confirmed_ || vision_capture_fallback) {
      if (!capture_confirmed_) {
        ++lane_capture_count_;
        publish_counts();
      }
      capture_confirmed_ = false;
      resume_current_lane("capture committed; resume current lane");
      return;
    }
    if (state_age() >= capture_forward_s_) {
      resume_current_lane("capture not confirmed; resume current lane");
    }
  }

  void finish_current_lane()
  {
    // 이 함수는 레인 끝점에 도착해 surface_lane_completed_를 true로 만든 뒤에만 호출된다.
    // 해당 레인에서 포획한 것이 있으면 먼저 배출하고, 없으면 바로 다음 레인을 선택한다.
    active_surface_lane_.reset();
    if (lane_capture_count_ > 0) {
      // 각 레인의 첫 배출은 반드시 attempt 1부터 시작한다.
      reset_dump_attempts_for_lane(dump_attempt_count_);
      transition(State::MOVE_TO_BONUS, "current lane complete with captures");
      return;
    }

    select_surface_lane();
    if (active_surface_lane_) {
      transition(State::SURFACE_SEARCH, "empty lane complete; start next lane");
    } else {
      transition(State::RETURN_TO_WORK_DEPTH, "all surface lanes complete");
    }
  }

  void run_move_bonus(std::array<uint16_t, 18> & channels)
  {
    if (distance(current_position_, bonus_center_) <= bonus_approach_distance_m_) {
      transition(State::DESCEND_TO_DUMP_DEPTH, "bonus approach radius reached");
      return;
    }
    follow_waypoint(channels, bonus_center_, search_forward_pwm_, collection_depth_m_);
  }

  void run_descend_dump(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, dump_depth_m_);
    if (std::abs(*depth_m_ - dump_depth_m_) <= depth_tolerance_m_) {
      transition(State::MOVE_TO_BONUS_CENTER, "dump depth reached");
    }
  }

  void run_bonus_center(std::array<uint16_t, 18> & channels)
  {
    // 가점존 중심 도착만으로 배출하지 않고 지정된 heading까지 맞춘 뒤 계약을 연다.
    if (follow_waypoint(channels, bonus_center_, search_forward_pwm_, dump_depth_m_)) {
      const double heading_error = wrap_pi(bonus_dump_heading_rad_ - current_yaw_rad_);
      set_channel(channels, 4, neutral_pwm_ - waypoint_yaw_delta(heading_error));
      if (std::abs(heading_error) <= heading_tolerance_rad_) {
        start_dump_attempt();
      }
    }
  }

  void start_dump_attempt()
  {
    next_dump_attempt(dump_attempt_count_);
    dump_motion_phase_ = DumpMotionPhase::FORWARD_OVERSHOOT;
    dump_motion_phase_started_at_ = now();
    dump_forward_peak_ = current_position_;
    open_score_gate();
    transition(State::DUMP_EJECT, "dump attempt " + std::to_string(dump_attempt_count_));
  }

  void run_dump_eject(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, dump_depth_m_);
    const double heading_error = wrap_pi(bonus_dump_heading_rad_ - current_yaw_rad_);
    set_channel(channels, 4, neutral_pwm_ - waypoint_yaw_delta(heading_error));

    if (dump_motion_phase_ == DumpMotionPhase::FORWARD_OVERSHOOT) {
      const bool forward_complete = dump_forward_target_reached(
        current_position_, bonus_center_, bonus_dump_heading_rad_, dump_forward_overshoot_m_) ||
        (now() - dump_motion_phase_started_at_).seconds() >= dump_forward_timeout_s_;
      if (!forward_complete) {
        set_channel(channels, 5, dump_forward_pwm_);
        return;
      }

      // 같은 제어 주기에서 바로 후진 PWM을 기록하므로 중립 대기 구간이 없다.
      dump_forward_peak_ = current_position_;
      dump_motion_phase_ = DumpMotionPhase::SHARP_REVERSE;
      dump_motion_phase_started_at_ = now();
    }

    set_channel(channels, 5, dump_reverse_pwm_);
    const bool reverse_complete = dump_reverse_target_reached(
      current_position_, dump_forward_peak_, bonus_center_, bonus_dump_heading_rad_,
      dump_reverse_distance_m_) ||
      (now() - dump_motion_phase_started_at_).seconds() >= dump_reverse_timeout_s_;
    if (reverse_complete) {
      close_score_gate();
      transition(State::DUMP_CHECK, "forward overshoot and sharp reverse complete");
    }
  }

  void run_dump_check(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, dump_depth_m_);
    if (state_age() < dump_settle_s_) {
      return;
    }
    // 카메라가 오래됐으면 '비었다'고 추측하지 않는다. 테스트용으로 무한 재시도는
    // 하지 않고 현재 배출 루프를 종료해 다음 미션이 진행될 수 있게 한다.
    TopNetStatus top_status = TopNetStatus::OCCUPIED;
    if (!recent_top()) {
      top_status = TopNetStatus::STALE;
      RCLCPP_WARN(get_logger(), "Top camera stale/unknown after dump; ending dump loop");
    } else if (top_empty_confirmed() || !top_occupied_) {
      top_status = TopNetStatus::EMPTY;
    }
    if (collector_state_received_) {
      top_status = reconcile_dump_status(top_status, collector_physically_occupied_);
    }
    if (!should_repeat_dump(
        top_status, dump_attempt_count_, static_cast<uint32_t>(max_dump_attempts_)))
    {
      lane_capture_count_ = 0;
      const bool lanes_remaining = std::any_of(
        surface_lane_completed_.begin(), surface_lane_completed_.end(),
        [](bool completed) {return !completed;});
      if (lanes_remaining) {
        // 다음 레인은 수집 수심에 도착한 run_ascend() -> begin_search()에서 선택한다.
        transition(State::ASCEND_TO_COLLECTION_DEPTH, "dump complete; continue remaining lanes");
      } else {
        transition(
          State::RETURN_TO_WORK_DEPTH,
          top_status == TopNetStatus::STALE ? "top stale after final lane dump" :
          top_status == TopNetStatus::EMPTY ? "collector empty after final lane dump" :
          "maximum dump attempts reached after final lane");
      }
      return;
    }
    transition(State::RETURN_TO_BONUS_CENTER, "collector still occupied");
  }

  void run_return_center(std::array<uint16_t, 18> & channels)
  {
    if (follow_waypoint(channels, bonus_center_, search_forward_pwm_, dump_depth_m_)) {
      transition(State::MOVE_TO_BONUS_CENTER, "bonus center reacquired");
    }
  }

  void run_return_depth(std::array<uint16_t, 18> & channels)
  {
    hold_depth(channels, work_depth_m_);
    if (std::abs(*depth_m_ - work_depth_m_) <= depth_tolerance_m_) {
      complete_cycle();
    }
  }

  bool follow_waypoint(
    std::array<uint16_t, 18> & channels, const Vec2 & target,
    int forward_pwm, double target_depth)
  {
    // 위치 오차로 목표 방위를 계산하고, 선수가 충분히 맞을 때만 전진한다.
    // 깊이 P 제어는 이동 상태와 독립적으로 매 주기 함께 적용한다.
    hold_depth(channels, target_depth);
    const Vec2 delta = target - current_position_;
    if (norm(delta) <= waypoint_tolerance_m_) {
      return true;
    }
    const double desired = std::atan2(delta.y, delta.x);
    const double error = wrap_pi(desired - current_yaw_rad_);
    set_channel(channels, 4, neutral_pwm_ - waypoint_yaw_delta(error));
    if (std::abs(error) <= heading_tolerance_rad_) {
      set_channel(channels, 5, forward_pwm);
    }
    return false;
  }

  int waypoint_yaw_delta(const double heading_error) const
  {
    return yaw_delta_pwm(
      heading_error,
      YawControlConfig{
        waypoint_yaw_kp_pwm_per_rad_, yaw_command_deadband_rad_,
        min_yaw_delta_pwm_, max_yaw_delta_pwm_});
  }

  void hold_depth(std::array<uint16_t, 18> & channels, double target_depth)
  {
    // 이 프로젝트의 깊이는 수면 아래가 양수다. depth_p_pwm()가 RC 채널 3의
    // 방향과 PWM 상한/하한을 함께 처리한다.
    set_channel(
      channels, 3,
      depth_p_pwm(
        target_depth, *depth_m_,
        DepthPConfig{depth_kp_pwm_per_m_, max_depth_delta_pwm_, neutral_pwm_, true}));
  }

  bool recent_front() const
  {
    return front_ && (now() - front_->received).seconds() <= detection_timeout_s_;
  }

  bool recent_top() const
  {
    return top_received_at_.nanoseconds() > 0 &&
      (now() - top_received_at_).seconds() <= detection_timeout_s_;
  }

  bool top_occupied_confirmed() const
  {
    return recent_top() && top_occupied_ && top_occupied_since_ &&
      (now() - *top_occupied_since_).seconds() >= top_confirm_s_;
  }

  bool top_empty_confirmed() const
  {
    return recent_top() && !top_occupied_ && top_empty_since_ &&
      (now() - *top_empty_since_).seconds() >= top_empty_confirm_s_;
  }

  void open_score_gate()
  {
    // 가점존 중심을 대회장 좌표에서 odom/world 좌표로 변환해 시뮬레이터에 전달한다.
    // 시뮬레이터는 이 계약이 RELEASE이고 부표가 실제 영역 안에 있을 때만 점수 처리한다.
    const Vec2 world = arena_transform_.position_to_odom(bonus_center_);
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{\"state\":\"RELEASE\",\"score_zone\":{\"xyz\":[" <<
      world.x << ',' << world.y << ',' << score_zone_world_z_m_ << "]}}";
    msg.data = out.str();
    score_release_pub_->publish(msg);
  }

  void close_score_gate()
  {
    std_msgs::msg::String msg;
    msg.data = "{\"state\":\"IDLE\",\"score_zone\":{\"xyz\":[0,0,0]}}";
    score_release_pub_->publish(msg);
  }

  void complete_cycle()
  {
    // 완료 순서: 배출 계약 닫기 -> RC 해제 -> 완료 발행 -> IDLE 전환.
    // 다음 레인 제어 노드가 RC를 이어받기 전에 남은 명령이 없도록 한다.
    close_score_gate();
    publish_release_once();
    rc_pub_.reset();
    std_msgs::msg::String msg;
    msg.data = "cycle=" + std::to_string(cycle_id_) + ";success=true";
    complete_pub_->publish(msg);
    last_completed_cycle_ = cycle_id_;
    active_ = false;
    state_ = State::IDLE;
    publish_state();
    RCLCPP_INFO(
      get_logger(), "Surface cycle %u complete (final=%s, deposited=%zu, remaining=%u)",
      cycle_id_, final_cycle_ ? "true" : "false", deposited_ids_.size(), remaining_count());
  }

  void fail_cycle(const std::string & reason)
  {
    close_score_gate();
    publish_release_once();
    rc_pub_.reset();
    std_msgs::msg::String msg;
    msg.data = "cycle=" + std::to_string(cycle_id_) + ";success=false;reason=" + reason;
    complete_pub_->publish(msg);
    active_ = false;
    state_ = State::IDLE;
    publish_state();
  }

  void transition(State next, const std::string & reason)
  {
    // 모든 상태 전이는 이 함수를 통해 시간 기준과 모니터링 토픽을 함께 갱신한다.
    RCLCPP_INFO(get_logger(), "Surface %s -> %s: %s", state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    state_entered_at_ = now();
    publish_state();
  }

  double state_age() const {return (now() - state_entered_at_).seconds();}

  static const char * state_name(State state)
  {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::ASCEND_TO_COLLECTION_DEPTH: return "ASCEND_TO_COLLECTION_DEPTH";
      case State::INITIAL_TOP_CHECK: return "INITIAL_TOP_CHECK";
      case State::SURFACE_SEARCH: return "SURFACE_SEARCH";
      case State::SURFACE_ALIGN: return "SURFACE_ALIGN";
      case State::SURFACE_CAPTURE: return "SURFACE_CAPTURE";
      case State::MOVE_TO_BONUS: return "MOVE_TO_BONUS";
      case State::DESCEND_TO_DUMP_DEPTH: return "DESCEND_TO_DUMP_DEPTH";
      case State::MOVE_TO_BONUS_CENTER: return "MOVE_TO_BONUS_CENTER";
      case State::DUMP_EJECT: return "DUMP_EJECT";
      case State::DUMP_CHECK: return "DUMP_CHECK";
      case State::RETURN_TO_BONUS_CENTER: return "RETURN_TO_BONUS_CENTER";
      case State::RETURN_TO_WORK_DEPTH: return "RETURN_TO_WORK_DEPTH";
      case State::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  void publish_state()
  {
    if (state_pub_) {
      std_msgs::msg::String msg;
      msg.data = state_name(state_);
      state_pub_->publish(msg);
    }
    publish_vision_gates();
  }

  void publish_vision_gates()
  {
    if (!front_enable_pub_ || !top_enable_pub_) {return;}
    const auto gate = vision_gate_for_state(state_, surface_heading_to_start_);

    if (gate.front_enabled != front_vision_enabled_) {
      front_vision_enabled_ = gate.front_enabled;
      front_.reset();
    }
    if (gate.top_enabled != top_vision_enabled_) {
      top_vision_enabled_ = gate.top_enabled;
      top_occupied_ = false;
      top_received_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      top_occupied_since_.reset();
      top_empty_since_.reset();
    }

    std_msgs::msg::Bool front;
    front.data = front_vision_enabled_;
    front_enable_pub_->publish(front);
    std_msgs::msg::Bool top;
    top.data = top_vision_enabled_;
    top_enable_pub_->publish(top);
    RCLCPP_INFO(
      get_logger(), "Surface vision gate: front=%s top=%s",
      front.data ? "ON" : "OFF", top.data ? "ON" : "OFF");
  }

  uint32_t remaining_count() const
  {
    return static_cast<uint32_t>(std::max<int>(0, surface_total_buoy_count_ - static_cast<int>(captured_ids_.size())));
  }

  void publish_counts()
  {
    if (!remaining_pub_ || !deposit_pub_) {return;}
    std_msgs::msg::UInt32 remaining;
    remaining.data = remaining_count();
    remaining_pub_->publish(remaining);
    std_msgs::msg::UInt32 deposited;
    deposited.data = static_cast<uint32_t>(deposited_ids_.size());
    deposit_pub_->publish(deposited);
  }

  std::array<uint16_t, 18> nochange_channels() const
  {
    std::array<uint16_t, 18> channels{};
    channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    return channels;
  }

  void set_neutral(std::array<uint16_t, 18> & channels) const
  {
    set_channel(channels, 3, neutral_pwm_);
    set_channel(channels, 4, neutral_pwm_);
    set_channel(channels, 5, neutral_pwm_);
  }

  void release_channels(std::array<uint16_t, 18> & channels) const
  {
    set_channel(channels, 3, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, 4, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, 5, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }

  void set_channel(std::array<uint16_t, 18> & channels, int channel, int pwm) const
  {
    if (pwm != mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE && pwm != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE) {
      pwm = std::clamp(pwm, min_pwm_, max_pwm_);
    }
    channels[static_cast<std::size_t>(channel - 1)] = static_cast<uint16_t>(pwm);
  }

  void publish_channels(const std::array<uint16_t, 18> & channels)
  {
    if (!rc_pub_) {
      return;
    }
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels = channels;
    rc_pub_->publish(msg);
  }

  // ROS 토픽 및 조정 가능한 임무 파라미터
  std::string front_bbox_topic_, top_bbox_topic_, front_enable_topic_, top_enable_topic_;
  std::string odometry_topic_, depth_pose_topic_;
  std::string start_frame_topic_, surface_start_topic_, surface_complete_topic_;
  std::string arena_config_topic_, score_release_topic_, rc_topic_;
  Vec2 bonus_center_{};
  double bonus_center_offset_x_m_{1.95}, bonus_center_offset_y_m_{0.0};
  double bonus_radius_m_{0.65}, bonus_approach_distance_m_{1.0}, bonus_dump_heading_rad_{0.0};
  double bonus_lane_clearance_m_{0.35}, min_surface_lane_segment_length_m_{1.0};
  double score_zone_world_z_m_{-0.3}, collection_depth_m_{0.30}, dump_depth_m_{0.85};
  double work_depth_m_{0.85}, depth_tolerance_m_{0.08};
  int surface_total_buoy_count_{5}, max_dump_attempts_{3};
  double dump_forward_overshoot_m_{0.70}, dump_reverse_distance_m_{1.10};
  double dump_forward_timeout_s_{4.0}, dump_reverse_timeout_s_{4.0}, dump_settle_s_{0.7};
  double capture_forward_s_{2.0}, capture_ignore_s_{0.8};
  double top_roi_x_min_{0.1}, top_roi_x_max_{0.9}, top_roi_y_min_{0.05}, top_roi_y_max_{0.95};
  double top_confirm_s_{0.5}, top_empty_confirm_s_{0.5}, detection_timeout_s_{1.5};
  double front_min_bbox_height_ratio_{0.03}, capture_min_bbox_height_ratio_{0.10};
  double align_deadband_x_{0.07}, align_stable_s_{0.4};
  double control_rate_hz_{20.0}, depth_kp_pwm_per_m_{300.0}, waypoint_tolerance_m_{0.25};
  double heading_tolerance_rad_{0.2}, waypoint_yaw_kp_pwm_per_rad_{450.0};
  double align_yaw_kp_pwm_per_normalized_x_{500.0}, yaw_command_deadband_rad_{0.03};
  int max_depth_delta_pwm_{180}, neutral_pwm_{1500}, min_pwm_{1300}, max_pwm_{1700};
  int search_forward_pwm_{1600}, capture_forward_pwm_{1650};
  int dump_forward_pwm_{1660}, dump_reverse_pwm_{1340};
  int min_yaw_delta_pwm_{55}, max_yaw_delta_pwm_{200};
  int buoy_class_id_{0};

  // 상태 머신의 현재 위치·센서·카운트 메모리
  State state_{State::IDLE};
  rclcpp::Time state_entered_at_{0, 0, RCL_ROS_TIME};
  ArenaFrameTransform arena_transform_;
  std::unique_ptr<LanePlanner> lane_planner_;
  std::vector<bool> surface_lane_completed_;
  std::optional<std::size_t> active_surface_lane_;
  Vec2 surface_lane_start_{}, surface_lane_finish_{}, current_position_{};
  bool surface_heading_to_start_{true};
  double current_yaw_rad_{0.0};
  bool have_odometry_{false};
  rclcpp::Time odometry_at_{0, 0, RCL_ROS_TIME};
  std::optional<double> depth_m_;
  rclcpp::Time depth_at_{0, 0, RCL_ROS_TIME};
  std::optional<Detection> front_;
  bool top_occupied_{false};
  rclcpp::Time top_received_at_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> top_occupied_since_, top_empty_since_, align_started_at_;
  bool initial_top_check_done_{false}, capture_confirmed_{false};
  bool collector_state_received_{false}, collector_physically_occupied_{false};
  bool front_vision_enabled_{false}, top_vision_enabled_{false};
  double capture_front_start_y_{0.0}, capture_front_min_y_{0.0};
  uint32_t lane_capture_count_{0}, dump_attempt_count_{0};
  uint32_t cycle_id_{0}, last_completed_cycle_{0};
  bool final_cycle_{false}, active_{false}, release_sent_{false};
  std::string pending_start_;
  std::set<std::string> captured_ids_, deposited_ids_;
  DumpMotionPhase dump_motion_phase_{DumpMotionPhase::FORWARD_OVERSHOOT};
  Vec2 dump_forward_peak_{};
  rclcpp::Time dump_motion_phase_started_at_{0, 0, RCL_ROS_TIME};

  // ROS 인터페이스 수명은 노드와 함께 유지한다.
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr front_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr top_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_frame_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arena_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
  rclcpp::Subscription<auv_msg::msg::CollectorState>::SharedPtr collector_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, complete_pub_, score_release_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr remaining_pub_, deposit_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_enable_pub_, top_enable_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace kmu26_auv_surface_buoy_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kmu26_auv_surface_buoy_mission::SurfaceBuoyMissionNode>();
  rclcpp::spin(node);
  // Ctrl+C나 executor 종료 시 마지막으로 RC override를 해제한다.
  node->publish_release_once();
  rclcpp::shutdown();
  return 0;
}
