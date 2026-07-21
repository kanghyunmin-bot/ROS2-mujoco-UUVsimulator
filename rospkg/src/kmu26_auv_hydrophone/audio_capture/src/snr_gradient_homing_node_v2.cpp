#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>


// ros2 launch audio_capture snr_gradient_homing_control.launch.py \
//   controller_dry_run:=false \
//   arena_start_corner:=bottom_right \
//   arena_yaw_rad:=3.141592653589793


namespace audio_capture
{
// V2는 연속 단일 주파수 음원이 좁은 수조에서 만드는 정재파를 직접 음원 방향으로
// 해석하지 않는다. 21.164 kHz의 수중 반파장은 약 3.5 cm이므로 원시 SNR에는
// 수 cm 간격의 peak/null이 생길 수 있다. 이 노드는 다음 두 공간 규모를 분리한다.
//
//  1. Local robust gradient:
//     최근 원시 표본에 거리/시간 가중 Huber 회귀를 적용한다.
//  2. Grid-map gradient:
//     반파장보다 큰 셀마다 SNR median을 만든 뒤 넓은 범위에서 다시 Huber 회귀한다.
//
// 두 결과가 같은 방향일 때만 적극적으로 융합한다. 서로 반대이면 더 강한 한쪽을
// 낮은 신뢰도로 사용하거나 출력을 보류한다. 따라서 기존 V1의 "SNR span이 크면
// confidence가 높다"는 가정을 제거하고, 회귀 잔차/관측성/두 규모의 일치도를
// confidence에 직접 반영한다.
class SnrGradientHomingNodeV2 : public rclcpp::Node
{
public:
    // [노드 초기화] 환경 파라미터와 기존 제어기 호환 토픽을 구성한다.
    explicit SnrGradientHomingNodeV2(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("snr_gradient_homing_v2", options)
    {
        // 실제 시험 환경이 바뀔 때 조정할 값만 ROS 파라미터로 노출한다.
        map_cell_size_m_ = std::clamp(
            declare_parameter<double>("map_cell_size_m", 0.12), 0.05, 1.0);
        local_radius_m_ = std::max(
            2.0 * map_cell_size_m_,
            declare_parameter<double>("local_radius_m", 0.75));
        map_radius_m_ = std::max(
            local_radius_m_,
            declare_parameter<double>("map_radius_m", 2.0));

        snr_sub_ = create_subscription<audio_common_msgs::msg::Float64Stamped>(
            SNR_TOPIC, 20,
            std::bind(&SnrGradientHomingNodeV2::snr_callback, this, std::placeholders::_1));
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            ODOMETRY_TOPIC, 30,
            std::bind(
                &SnrGradientHomingNodeV2::odometry_callback, this, std::placeholders::_1));
        reset_sub_ = create_subscription<std_msgs::msg::Empty>(
            RESET_TOPIC, 10,
            std::bind(&SnrGradientHomingNodeV2::reset_callback, this, std::placeholders::_1));
        vertical_search_active_sub_ = create_subscription<std_msgs::msg::Bool>(
            VERTICAL_SEARCH_ACTIVE_TOPIC,
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(
                &SnrGradientHomingNodeV2::vertical_search_active_callback,
                this,
                std::placeholders::_1));

        direction_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>(DIRECTION_TOPIC, 10);
        confidence_pub_ =
            create_publisher<std_msgs::msg::Float64>(CONFIDENCE_TOPIC, 10);
        high_snr_region_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>(HIGH_SNR_REGION_TOPIC, 10);
        local_direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            LOCAL_DIRECTION_TOPIC, 10);
        map_direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            MAP_DIRECTION_TOPIC, 10);
        estimator_ready_pub_ =
            create_publisher<std_msgs::msg::Bool>(READY_TOPIC, 10);
        vertical_search_request_pub_ = create_publisher<std_msgs::msg::Bool>(
            VERTICAL_SEARCH_REQUEST_TOPIC,
            rclcpp::QoS(1).reliable().transient_local());
        vertical_best_z_pub_ = create_publisher<std_msgs::msg::Float64>(
            VERTICAL_BEST_Z_TOPIC,
            rclcpp::QoS(1).reliable().transient_local());
        publish_vertical_search_request(false);

        RCLCPP_INFO(
            get_logger(),
            "SNR homing V2 ready: rolling grid cell=%.2f m local=%.2f m "
            "map=%.2f m keep=%.2f m.",
            map_cell_size_m_, local_radius_m_, map_radius_m_,
            map_radius_m_ + GRID_KEEP_MARGIN_M);
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr char SNR_TOPIC[] =
        "/audio_phase_estimator/iq_snr_ratio_stamped";
    static constexpr char ODOMETRY_TOPIC[] = "/odometry/filtered";
    static constexpr char DIRECTION_TOPIC[] = "/homing/direction";
    static constexpr char CONFIDENCE_TOPIC[] = "/homing/snr_confidence";
    static constexpr char HIGH_SNR_REGION_TOPIC[] = "/homing/high_snr_region";
    static constexpr char LOCAL_DIRECTION_TOPIC[] = "/homing/local_gradient_direction";
    static constexpr char MAP_DIRECTION_TOPIC[] = "/homing/map_gradient_direction";
    static constexpr char READY_TOPIC[] = "/homing/estimator_ready";
    static constexpr char RESET_TOPIC[] = "/homing/reset_estimator";
    static constexpr char VERTICAL_SEARCH_REQUEST_TOPIC[] =
        "/homing/vertical_search_request";
    static constexpr char VERTICAL_SEARCH_ACTIVE_TOPIC[] =
        "/homing/vertical_search_active";
    static constexpr char VERTICAL_BEST_Z_TOPIC[] = "/homing/vertical_best_z";

    // 알고리즘 구조를 정의하는 값은 실행 중 변경하지 않는다.
    static constexpr std::size_t MAX_RAW_SAMPLES = 600;
    static constexpr std::size_t MAX_ODOMETRY_SAMPLES = 500;
    static constexpr std::size_t MAX_PENDING_SNR = 500;
    static constexpr std::size_t MAX_CELL_VALUES = 31;
    static constexpr std::size_t MIN_LOCAL_OBSERVATIONS = 12;
    static constexpr std::size_t MIN_MAP_CELLS = 8;
    static constexpr std::size_t MIN_HIGH_SNR_REGION_CELLS = 10;
    static constexpr std::size_t MAX_DIRECTION_HISTORY = 30;
    static constexpr std::size_t MIN_DIRECTION_STABILITY_SAMPLES = 5;
    static constexpr std::size_t MAX_VERTICAL_BIN_VALUES = 31;
    static constexpr std::size_t MIN_VERTICAL_BIN_VALUES = 2;
    static constexpr std::size_t IRLS_ITERATIONS = 5;
    static constexpr double RAW_SAMPLE_AGE_S = 45.0;
    static constexpr double ODOMETRY_HISTORY_AGE_S = 12.0;
    static constexpr double MAX_ODOMETRY_EXTRAPOLATION_S = 0.12;
    static constexpr double MAX_PENDING_SNR_AGE_S = 2.0;
    static constexpr double MIN_SAMPLE_SPACING_M = 0.02;
    static constexpr double GRID_KEEP_MARGIN_M = 1.0;
    static constexpr double GRID_MAX_AGE_S = 60.0;
    static constexpr double MIN_SNR_DB = -240.0;
    static constexpr double MAX_SNR_DB = 80.0;
    static constexpr double HUBER_K = 1.5;
    static constexpr double MIN_RESIDUAL_SCALE_DB = 0.10;
    static constexpr double MIN_GRADIENT_DB_PER_M = 0.05;
    static constexpr double MIN_COVERAGE_RATIO = 0.035;
    static constexpr double MIN_OUTPUT_CONFIDENCE = 0.12;
    static constexpr double READY_CONFIDENCE = 0.28;
    static constexpr double MAX_STABLE_STD_RAD = 0.40;
    static constexpr double DIRECTION_HISTORY_MAX_AGE_S = 2.0;
    static constexpr double FUSION_MIN_DOT = 0.50;
    static constexpr double STRONGER_MODEL_RATIO = 1.60;
    static constexpr double DIRECTION_FILTER_ALPHA = 0.22;
    static constexpr double VERTICAL_TRIGGER_SNR_DB = 10.0;
    static constexpr double VERTICAL_TRIGGER_HOLD_S = 1.0;
    static constexpr double VERTICAL_BIN_SIZE_M = 0.05;
    static constexpr double MIN_VERTICAL_SAMPLE_SPACING_M = 0.02;

    struct Sample
    {
        rclcpp::Time stamp;
        Eigen::Vector3d position_m{0.0, 0.0, 0.0};
        double snr_db = 0.0;
    };

    struct PoseSample
    {
        rclcpp::Time stamp;
        Eigen::Vector3d position_m{0.0, 0.0, 0.0};
    };

    struct PendingSnr
    {
        rclcpp::Time stamp;
        double snr_db = 0.0;
    };

    struct GridCell
    {
        std::deque<double> snr_db_values;
        rclcpp::Time last_stamp;
        std::size_t total_visits = 0;
    };

    struct Observation
    {
        Eigen::Vector2d position_m{0.0, 0.0};
        double value_db = 0.0;
        double base_weight = 1.0;
    };

    struct FitResult
    {
        bool valid = false;
        Eigen::Vector2d direction{1.0, 0.0};
        Eigen::Vector2d gradient{0.0, 0.0};
        double confidence = 0.0;
        double coverage_ratio = 0.0;
        double robust_r2 = 0.0;
        double residual_scale_db = 0.0;
        double effective_count = 0.0;
    };

    struct TimedDirection
    {
        rclcpp::Time stamp;
        Eigen::Vector2d direction{1.0, 0.0};
    };

    enum class PoseLookup
    {
        FOUND,
        WAIT_FOR_FUTURE,
        TOO_OLD,
        EMPTY
    };

    // [Odometry 수신] 현재 위치·yaw와 시각별 위치 이력을 갱신하고 대기 SNR을 처리한다.
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        current_position_m_.x() = msg->pose.pose.position.x;
        current_position_m_.y() = msg->pose.pose.position.y;
        current_position_m_.z() = msg->pose.pose.position.z;
        current_yaw_rad_ = yaw_from_quaternion(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        odometry_frame_id_ =
            msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;
        have_odometry_ = true;

        const rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() <= 0) {
            return;
        }
        if (!odometry_history_.empty() &&
            stamp < odometry_history_.back().stamp)
        {
            odometry_history_.clear();
            reset_measurements();
            RCLCPP_INFO(get_logger(), "Odometry time moved backwards; V2 map reset.");
        }
        odometry_history_.push_back({stamp, current_position_m_});
        while (odometry_history_.size() > MAX_ODOMETRY_SAMPLES ||
            (!odometry_history_.empty() &&
            (stamp - odometry_history_.front().stamp).seconds() >
            ODOMETRY_HISTORY_AGE_S))
        {
            odometry_history_.pop_front();
        }
        process_pending_snr();
    }

    // [수직 탐색 상태 수신] sweep 시작 시 별도 z-SNR bin과 stale 수평 방향 상태를 초기화한다.
    void vertical_search_active_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if (msg->data == vertical_search_active_) {
            return;
        }
        vertical_search_active_ = msg->data;
        direction_history_.clear();
        have_filtered_direction_ = false;
        publish_status(false);
        if (vertical_search_active_) {
            vertical_bins_.clear();
            have_last_vertical_sample_z_ = false;
            RCLCPP_INFO(get_logger(), "Vertical SNR sweep collection started.");
        } else {
            RCLCPP_INFO(get_logger(), "Vertical SNR sweep collection stopped.");
        }
    }

    // [SNR 수신] IQ SNR ratio를 dB로 변환해 timestamp 순서의 처리 대기열에 넣는다.
    void snr_callback(
        const audio_common_msgs::msg::Float64Stamped::ConstSharedPtr msg)
    {
        if (!std::isfinite(msg->data) || msg->data <= 0.0) {
            return;
        }
        const rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() <= 0) {
            return;
        }
        // Upstream 값은 power가 아니라 target/noise IQ magnitude 비율이므로 20log10을 사용한다.
        const double snr_db =
            std::clamp(20.0 * std::log10(msg->data), MIN_SNR_DB, MAX_SNR_DB);
        update_vertical_search_trigger(stamp, snr_db);
        const PendingSnr pending{stamp, snr_db};
        const auto insert_at = std::upper_bound(
            pending_snr_.begin(), pending_snr_.end(), stamp,
            [](const rclcpp::Time & value, const PendingSnr & sample) {
                return value < sample.stamp;
            });
        pending_snr_.insert(insert_at, pending);
        while (pending_snr_.size() > MAX_PENDING_SNR) {
            pending_snr_.pop_front();
        }
        process_pending_snr();
    }

    // [수직 탐색 Trigger] 10 dB 이상 SNR이 1초 지속되면 미션당 한 번 sweep을 요청한다.
    void update_vertical_search_trigger(
        const rclcpp::Time & stamp, const double snr_db)
    {
        if (vertical_search_requested_ || vertical_search_active_) {
            return;
        }
        if (snr_db < VERTICAL_TRIGGER_SNR_DB) {
            have_vertical_trigger_start_ = false;
            return;
        }
        if (!have_vertical_trigger_start_) {
            vertical_trigger_start_ = stamp;
            have_vertical_trigger_start_ = true;
            return;
        }
        if ((stamp - vertical_trigger_start_).seconds() >= VERTICAL_TRIGGER_HOLD_S) {
            vertical_search_requested_ = true;
            publish_vertical_search_request(true);
            RCLCPP_INFO(
                get_logger(),
                "Vertical search requested after SNR >= %.1f dB for %.1f s.",
                VERTICAL_TRIGGER_SNR_DB,
                VERTICAL_TRIGGER_HOLD_S);
        }
    }

    // [수직 탐색 요청 발행] Controller가 놓치지 않도록 transient-local Bool 상태를 갱신한다.
    void publish_vertical_search_request(const bool requested)
    {
        std_msgs::msg::Bool msg;
        msg.data = requested;
        vertical_search_request_pub_->publish(msg);
    }

    // [외부 Reset] 미션 재시작 요청에서 최근 상태와 장기 SNR map을 모두 초기화한다.
    void reset_callback(const std_msgs::msg::Empty::ConstSharedPtr)
    {
        reset_measurements();
        publish_status(false);
        RCLCPP_INFO(get_logger(), "SNR homing V2 mission state and SNR map reset.");
    }

    // [완전 초기화] 미션 재시작이나 rosbag 시간 역행 때 최근 상태와 SNR map을 모두 지운다.
    void reset_measurements()
    {
        pending_snr_.clear();
        raw_samples_.clear();
        grid_.clear();
        direction_history_.clear();
        have_filtered_direction_ = false;
        have_last_processed_stamp_ = false;
        vertical_bins_.clear();
        have_last_vertical_sample_z_ = false;
        vertical_search_active_ = false;
        vertical_search_requested_ = false;
        have_vertical_trigger_start_ = false;
        publish_vertical_search_request(false);
    }

    // [SNR-위치 동기화] 대기 중인 각 SNR timestamp에 대응하는 odometry 위치를 찾아 처리한다.
    void process_pending_snr()
    {
        if (!have_odometry_) {
            return;
        }
        while (!pending_snr_.empty()) {
            const PendingSnr pending = pending_snr_.front();
            Eigen::Vector3d position;
            const PoseLookup lookup = lookup_odometry(pending.stamp, position);
            if (lookup == PoseLookup::WAIT_FOR_FUTURE ||
                lookup == PoseLookup::EMPTY)
            {
                if (pending_snr_.size() > 1 &&
                    (pending_snr_.back().stamp - pending.stamp).seconds() >
                    MAX_PENDING_SNR_AGE_S)
                {
                    pending_snr_.pop_front();
                    continue;
                }
                return;
            }
            pending_snr_.pop_front();
            if (lookup == PoseLookup::TOO_OLD) {
                continue;
            }
            if (have_last_processed_stamp_ &&
                pending.stamp <= last_processed_stamp_)
            {
                continue;
            }
            last_processed_stamp_ = pending.stamp;
            have_last_processed_stamp_ = true;
            process_measurement(pending, position);
        }
    }

    // [Odometry 조회] SNR 측정 시각의 위치를 이력에서 보간하거나 제한적으로 외삽한다.
    PoseLookup lookup_odometry(
        const rclcpp::Time & stamp, Eigen::Vector3d & position) const
    {
        if (odometry_history_.empty()) {
            return PoseLookup::EMPTY;
        }
        if (stamp <= odometry_history_.front().stamp) {
            if ((odometry_history_.front().stamp - stamp).seconds() >
                MAX_ODOMETRY_EXTRAPOLATION_S)
            {
                return PoseLookup::TOO_OLD;
            }
            position = odometry_history_.front().position_m;
            return PoseLookup::FOUND;
        }
        if (stamp >= odometry_history_.back().stamp) {
            if ((stamp - odometry_history_.back().stamp).seconds() >
                MAX_ODOMETRY_EXTRAPOLATION_S)
            {
                return PoseLookup::WAIT_FOR_FUTURE;
            }
            position = odometry_history_.back().position_m;
            return PoseLookup::FOUND;
        }
        for (std::size_t i = 1; i < odometry_history_.size(); ++i) {
            if (odometry_history_[i].stamp >= stamp) {
                const PoseSample & before = odometry_history_[i - 1];
                const PoseSample & after = odometry_history_[i];
                const double duration = (after.stamp - before.stamp).seconds();
                const double alpha = duration > 0.0 ?
                    (stamp - before.stamp).seconds() / duration : 0.0;
                position =
                    (1.0 - alpha) * before.position_m + alpha * after.position_m;
                return PoseLookup::FOUND;
            }
        }
        return PoseLookup::WAIT_FOR_FUTURE;
    }

    // [공간 표본 처리] 충분히 떨어진 위치-SNR 표본을 저장하고 격자와 방향 추정기를 갱신한다.
    void process_measurement(
        const PendingSnr & pending, const Eigen::Vector3d & position)
    {
        if (vertical_search_active_) {
            process_vertical_measurement(position.z(), pending.snr_db);
            return;
        }
        prune_grid(position.head<2>(), pending.stamp);
        if (!raw_samples_.empty() &&
            (position - raw_samples_.back().position_m).head<2>().norm() <
            MIN_SAMPLE_SPACING_M)
        {
            return;
        }

        raw_samples_.push_back({pending.stamp, position, pending.snr_db});
        while (raw_samples_.size() > MAX_RAW_SAMPLES ||
            (!raw_samples_.empty() &&
            (pending.stamp - raw_samples_.front().stamp).seconds() >
            RAW_SAMPLE_AGE_S))
        {
            raw_samples_.pop_front();
        }
        update_grid(raw_samples_.back());
        update_direction(pending.stamp);
    }

    // [수직 SNR 표본] sweep 중 충분히 떨어진 z-SNR 표본만 전용 bin에 저장한다.
    void process_vertical_measurement(const double z_m, const double snr_db)
    {
        if (have_last_vertical_sample_z_ &&
            std::abs(z_m - last_vertical_sample_z_m_) <
            MIN_VERTICAL_SAMPLE_SPACING_M)
        {
            return;
        }
        last_vertical_sample_z_m_ = z_m;
        have_last_vertical_sample_z_ = true;
        const int bin = static_cast<int>(std::floor(z_m / VERTICAL_BIN_SIZE_M));
        std::deque<double> & values = vertical_bins_[bin];
        values.push_back(snr_db);
        while (values.size() > MAX_VERTICAL_BIN_VALUES) {
            values.pop_front();
        }
        update_vertical_best_z();
    }

    // [최고 SNR 수심] 인접 z bin median을 평활화해 sweep에서 가장 강한 odometry z를 발행한다.
    void update_vertical_best_z()
    {
        bool found = false;
        int best_bin = 0;
        double best_smoothed_snr_db = -std::numeric_limits<double>::infinity();
        for (const auto & entry : vertical_bins_) {
            if (entry.second.size() < MIN_VERTICAL_BIN_VALUES) {
                continue;
            }
            double sum = 0.0;
            std::size_t count = 0;
            for (int offset = -1; offset <= 1; ++offset) {
                const auto neighbor = vertical_bins_.find(entry.first + offset);
                if (neighbor != vertical_bins_.end() &&
                    neighbor->second.size() >= MIN_VERTICAL_BIN_VALUES)
                {
                    sum += median(neighbor->second);
                    ++count;
                }
            }
            if (count == 0) {
                continue;
            }
            const double smoothed_snr_db = sum / static_cast<double>(count);
            if (smoothed_snr_db > best_smoothed_snr_db) {
                best_smoothed_snr_db = smoothed_snr_db;
                best_bin = entry.first;
                found = true;
            }
        }
        if (!found) {
            return;
        }
        const double vertical_best_z_m =
            (static_cast<double>(best_bin) + 0.5) * VERTICAL_BIN_SIZE_M;
        std_msgs::msg::Float64 msg;
        msg.data = vertical_best_z_m;
        vertical_best_z_pub_->publish(msg);
    }

    // [격자 갱신] 측정 위치의 셀에 최근 SNR 값들을 누적해 공간 median 계산을 준비한다.
    void update_grid(const Sample & sample)
    {
        int ix = 0;
        int iy = 0;
        if (!world_to_cell(sample.position_m.head<2>(), ix, iy)) {
            return;
        }
        GridCell & cell = grid_[{ix, iy}];
        cell.snr_db_values.push_back(sample.snr_db);
        while (cell.snr_db_values.size() > MAX_CELL_VALUES) {
            cell.snr_db_values.pop_front();
        }
        cell.last_stamp = sample.stamp;
        ++cell.total_visits;
    }

    // [World→Cell 변환] 절대 원점과 무관하게 odometry 평면을 고정 크기 격자로 양자화한다.
    bool world_to_cell(
        const Eigen::Vector2d & world, int & ix, int & iy) const
    {
        ix = static_cast<int>(std::floor(world.x() / map_cell_size_m_));
        iy = static_cast<int>(std::floor(world.y() / map_cell_size_m_));
        return true;
    }

    // [Cell→World 변환] 격자 인덱스 중심점을 odometry 좌표계의 2차원 위치로 복원한다.
    Eigen::Vector2d cell_center_world(const int ix, const int iy) const
    {
        return Eigen::Vector2d(
            (static_cast<double>(ix) + 0.5) * map_cell_size_m_,
            (static_cast<double>(iy) + 0.5) * map_cell_size_m_);
    }

    // [Rolling grid 정리] 현재 주변 보존 반경 밖이거나 오래된 셀을 지워 stale 전역 지도를 막는다.
    void prune_grid(const Eigen::Vector2d & center, const rclcpp::Time & stamp)
    {
        const double keep_radius = map_radius_m_ + GRID_KEEP_MARGIN_M;
        for (auto it = grid_.begin(); it != grid_.end();) {
            const Eigen::Vector2d position =
                cell_center_world(it->first.first, it->first.second);
            const double age_s = std::max(0.0, (stamp - it->second.last_stamp).seconds());
            if ((position - center).norm() > keep_radius || age_s > GRID_MAX_AGE_S) {
                it = grid_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // [Local Gradient] 현재 위치 주변의 최근 원시 SNR 표본으로 단거리 강건 gradient를 추정한다.
    FitResult estimate_local_gradient(const rclcpp::Time & stamp) const
    {
        std::vector<Observation> observations;
        observations.reserve(raw_samples_.size());
        const Eigen::Vector2d center = raw_samples_.back().position_m.head<2>();
        for (const Sample & sample : raw_samples_) {
            const double age_s = std::max(0.0, (stamp - sample.stamp).seconds());
            const double distance = (sample.position_m.head<2>() - center).norm();
            if (distance > local_radius_m_ || age_s > RAW_SAMPLE_AGE_S) {
                continue;
            }
            const double spatial_weight =
                std::exp(-0.5 * distance * distance /
                std::max(local_radius_m_ * local_radius_m_, 1.0e-9));
            const double temporal_weight = std::exp(-age_s / 20.0);
            observations.push_back({
                sample.position_m.head<2>(), sample.snr_db,
                spatial_weight * temporal_weight});
        }
        return robust_plane_fit(
            observations, MIN_LOCAL_OBSERVATIONS, local_radius_m_);
    }

    // [Map Gradient] 방문 격자들의 median SNR로 멀티패스가 평활화된 장거리 gradient를 추정한다.
    FitResult estimate_map_gradient() const
    {
        std::vector<Observation> observations;
        observations.reserve(grid_.size());
        const Eigen::Vector2d center = raw_samples_.back().position_m.head<2>();
        for (const auto & entry : grid_) {
            const GridCell & cell = entry.second;
            if (cell.snr_db_values.size() < 2) {
                continue;
            }
            const Eigen::Vector2d position =
                cell_center_world(entry.first.first, entry.first.second);
            const double distance = (position - center).norm();
            if (distance > map_radius_m_) {
                continue;
            }
            const double spatial_weight =
                std::exp(-0.5 * distance * distance /
                std::max(map_radius_m_ * map_radius_m_, 1.0e-9));
            const double visit_weight =
                std::sqrt(static_cast<double>(
                    std::min<std::size_t>(cell.total_visits, 16)));
            observations.push_back({
                position, median(cell.snr_db_values),
                spatial_weight * visit_weight});
        }
        return robust_plane_fit(observations, MIN_MAP_CELLS, map_radius_m_);
    }

    // [강건 평면 회귀] Huber IRLS로 SNR 평면과 잔차·관측성 기반 신뢰도를 계산한다.
    FitResult robust_plane_fit(
        const std::vector<Observation> & observations,
        const std::size_t minimum_count,
        const double spatial_scale_m) const
    {
        FitResult result;
        if (observations.size() < minimum_count) {
            return result;
        }

        std::vector<double> robust_weights(observations.size(), 1.0);
        Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
        Eigen::Vector2d mean_position = Eigen::Vector2d::Zero();
        double mean_value = 0.0;
        double residual_scale = MIN_RESIDUAL_SCALE_DB;

        for (std::size_t iteration = 0; iteration < IRLS_ITERATIONS; ++iteration) {
            double weight_sum = 0.0;
            mean_position.setZero();
            mean_value = 0.0;
            for (std::size_t i = 0; i < observations.size(); ++i) {
                const double weight =
                    observations[i].base_weight * robust_weights[i];
                weight_sum += weight;
                mean_position += weight * observations[i].position_m;
                mean_value += weight * observations[i].value_db;
            }
            if (weight_sum <= 1.0e-9) {
                return result;
            }
            mean_position /= weight_sum;
            mean_value /= weight_sum;

            Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity() * 1.0e-6;
            Eigen::Vector2d rhs = Eigen::Vector2d::Zero();
            for (std::size_t i = 0; i < observations.size(); ++i) {
                const double weight =
                    observations[i].base_weight * robust_weights[i];
                const Eigen::Vector2d delta =
                    observations[i].position_m - mean_position;
                covariance += weight * delta * delta.transpose();
                rhs += weight * delta * (observations[i].value_db - mean_value);
            }
            gradient = covariance.ldlt().solve(rhs);
            if (!gradient.allFinite()) {
                return result;
            }

            std::vector<double> residuals;
            residuals.reserve(observations.size());
            for (const Observation & observation : observations) {
                residuals.push_back(
                    observation.value_db - mean_value -
                    gradient.dot(observation.position_m - mean_position));
            }
            const double residual_median = median(residuals);
            std::vector<double> absolute_deviations;
            absolute_deviations.reserve(residuals.size());
            for (const double residual : residuals) {
                absolute_deviations.push_back(std::abs(residual - residual_median));
            }
            residual_scale = std::max(
                MIN_RESIDUAL_SCALE_DB, 1.4826 * median(absolute_deviations));
            for (std::size_t i = 0; i < residuals.size(); ++i) {
                const double normalized =
                    std::abs(residuals[i] - residual_median) /
                    std::max(HUBER_K * residual_scale, 1.0e-9);
                robust_weights[i] = normalized <= 1.0 ? 1.0 : 1.0 / normalized;
            }
        }

        double weight_sum = 0.0;
        double weight_squared_sum = 0.0;
        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        double rss = 0.0;
        double tss = 0.0;
        for (std::size_t i = 0; i < observations.size(); ++i) {
            const double weight = observations[i].base_weight * robust_weights[i];
            const Eigen::Vector2d delta =
                observations[i].position_m - mean_position;
            const double residual =
                observations[i].value_db - mean_value - gradient.dot(delta);
            const double centered = observations[i].value_db - mean_value;
            weight_sum += weight;
            weight_squared_sum += weight * weight;
            covariance += weight * delta * delta.transpose();
            rss += weight * residual * residual;
            tss += weight * centered * centered;
        }
        if (weight_sum <= 1.0e-9) {
            return result;
        }
        covariance /= weight_sum;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(covariance);
        if (eigen_solver.info() != Eigen::Success) {
            return result;
        }
        const double smallest = std::max(0.0, eigen_solver.eigenvalues().x());
        const double largest = std::max(1.0e-12, eigen_solver.eigenvalues().y());
        result.coverage_ratio = std::clamp(smallest / largest, 0.0, 1.0);
        result.effective_count =
            weight_squared_sum > 1.0e-12 ?
            weight_sum * weight_sum / weight_squared_sum : 0.0;
        result.robust_r2 =
            tss > 1.0e-9 ? std::clamp(1.0 - rss / tss, 0.0, 1.0) : 0.0;
        result.residual_scale_db = residual_scale;
        result.gradient = gradient;

        const double gradient_norm = gradient.norm();
        if (!std::isfinite(gradient_norm) ||
            gradient_norm < MIN_GRADIENT_DB_PER_M ||
            result.coverage_ratio < MIN_COVERAGE_RATIO)
        {
            return result;
        }
        result.direction = gradient / gradient_norm;

        const double count_factor = std::clamp(
            (result.effective_count - 0.6 * static_cast<double>(minimum_count)) /
            std::max(1.0, 1.4 * static_cast<double>(minimum_count)), 0.0, 1.0);
        const double coverage_factor = std::clamp(
            (result.coverage_ratio - MIN_COVERAGE_RATIO) / 0.30, 0.0, 1.0);
        const double fit_factor = std::clamp(result.robust_r2 / 0.65, 0.0, 1.0);
        const double signal_factor = std::clamp(
            gradient_norm * spatial_scale_m /
            std::max(2.5 * residual_scale, 0.25), 0.0, 1.0);
        result.confidence = std::pow(
            std::max(
                0.0, count_factor * coverage_factor * fit_factor * signal_factor),
            0.25);
        result.valid = result.confidence >= MIN_OUTPUT_CONFIDENCE;
        return result;
    }

    // [최종 방향 갱신] Local/Map gradient를 검증·융합하고 상태, 방향, 음원 후보를 발행한다.
    void update_direction(const rclcpp::Time & stamp)
    {
        if (raw_samples_.empty()) {
            return;
        }
        const FitResult local = estimate_local_gradient(stamp);
        const FitResult map = estimate_map_gradient();
        if (local.valid) {
            publish_model_direction(stamp, local.direction, local_direction_pub_);
        }
        if (map.valid) {
            publish_model_direction(stamp, map.direction, map_direction_pub_);
        }

        Eigen::Vector2d direction;
        double confidence = 0.0;
        double model_dot = std::numeric_limits<double>::quiet_NaN();
        bool have_direction = false;

        if (local.valid && map.valid) {
            model_dot = std::clamp(local.direction.dot(map.direction), -1.0, 1.0);
            if (model_dot >= FUSION_MIN_DOT) {
                direction =
                    local.confidence * local.direction + map.confidence * map.direction;
                if (direction.norm() > 1.0e-9) {
                    direction.normalize();
                    const double agreement =
                        std::clamp((model_dot - FUSION_MIN_DOT) /
                        (1.0 - FUSION_MIN_DOT), 0.0, 1.0);
                    confidence =
                        (0.5 * local.confidence + 0.5 * map.confidence) *
                        (0.65 + 0.35 * agreement);
                    have_direction = true;
                }
            } else if (
                local.confidence >= STRONGER_MODEL_RATIO * map.confidence &&
                local.confidence >= 0.55)
            {
                direction = local.direction;
                confidence = 0.55 * local.confidence;
                have_direction = true;
            } else if (
                map.confidence >= STRONGER_MODEL_RATIO * local.confidence &&
                map.confidence >= 0.55)
            {
                direction = map.direction;
                confidence = 0.60 * map.confidence;
                have_direction = true;
            }
        } else if (local.valid) {
            direction = local.direction;
            confidence = 0.65 * local.confidence;
            have_direction = true;
        } else if (map.valid) {
            direction = map.direction;
            confidence = 0.70 * map.confidence;
            have_direction = true;
        }

        Eigen::Vector2d high_snr_region_position;
        double region_contrast = 0.0;
        const bool have_high_snr_region =
            estimate_high_snr_region(high_snr_region_position, region_contrast);

        if (!have_direction) {
            prune_direction_history(stamp);
            publish_status(false);
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "V2 direction withheld: local=%.2f map=%.2f dot=%s cells=%zu.",
                local.confidence, map.confidence,
                std::isfinite(model_dot) ? std::to_string(model_dot).c_str() : "n/a",
                grid_.size());
            if (have_high_snr_region) {
                publish_high_snr_region(stamp, high_snr_region_position);
            }
            return;
        }

        update_direction_history(stamp, direction);
        const bool stable = direction_is_stable();
        confidence *= stability_confidence();
        const bool ready = stable && confidence >= READY_CONFIDENCE;
        publish_status(ready);
        publish_direction(stamp, filter_direction(direction), confidence);
        if (have_high_snr_region) {
            publish_high_snr_region(stamp, high_snr_region_position);
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "V2 local[c=%.2f r2=%.2f cov=%.2f] map[c=%.2f r2=%.2f cov=%.2f] "
            "dot=%.2f out=%.2f ready=%s region_contrast=%.2f.",
            local.confidence, local.robust_r2, local.coverage_ratio,
            map.confidence, map.robust_r2, map.coverage_ratio,
            std::isfinite(model_dot) ? model_dot : -2.0,
            confidence, ready ? "true" : "false", region_contrast);
    }

    // [고-SNR 영역 추정] 방문한 격자 중 최고 SNR 셀 주변의 우세 영역 중심을 진단값으로 구한다.
    bool estimate_high_snr_region(
        Eigen::Vector2d & region, double & contrast) const
    {
        struct CellEstimate
        {
            Eigen::Vector2d position;
            double value = 0.0;
            std::size_t visits = 0;
        };
        std::vector<CellEstimate> cells;
        std::vector<double> values;
        for (const auto & entry : grid_) {
            if (entry.second.snr_db_values.size() < 2) {
                continue;
            }
            const double value = median(entry.second.snr_db_values);
            cells.push_back({
                cell_center_world(entry.first.first, entry.first.second),
                value, entry.second.total_visits});
            values.push_back(value);
        }
        if (cells.size() < MIN_HIGH_SNR_REGION_CELLS) {
            return false;
        }

        std::sort(values.begin(), values.end());
        const std::size_t threshold_index = static_cast<std::size_t>(
            std::floor(0.85 * static_cast<double>(values.size() - 1)));
        const double threshold = values[threshold_index];
        const double median_value = values[values.size() / 2];

        const auto peak_it = std::max_element(
            cells.begin(), cells.end(),
            [](const CellEstimate & left, const CellEstimate & right) {
                return left.value < right.value;
            });
        if (peak_it == cells.end()) {
            return false;
        }
        const double peak_value = peak_it->value;
        const Eigen::Vector2d peak_position = peak_it->position;
        // 서로 떨어진 반사파 peak들의 허위 중간점을 만들지 않도록 최고 셀 주변의
        // 연결된 고-SNR 영역만 hotspot 진단값에 포함한다.
        const double cluster_radius_m = std::max(3.0 * map_cell_size_m_, 0.35);
        region.setZero();
        double weight_sum = 0.0;
        for (const CellEstimate & cell : cells) {
            if (cell.value < threshold ||
                (cell.position - peak_position).norm() > cluster_radius_m)
            {
                continue;
            }
            const double value_weight =
                std::exp(std::clamp((cell.value - threshold) / 3.0, 0.0, 3.0));
            const double visit_weight =
                std::sqrt(static_cast<double>(std::min<std::size_t>(cell.visits, 16)));
            const double weight = value_weight * visit_weight;
            region += weight * cell.position;
            weight_sum += weight;
        }
        if (weight_sum <= 1.0e-9) {
            return false;
        }
        region /= weight_sum;
        contrast = std::clamp((peak_value - median_value) / 6.0, 0.0, 1.0);
        return contrast >= 0.20;
    }

    // [방향 이력 갱신] 방향에 timestamp를 결합하고 2초보다 오래된 이력을 제거한다.
    void update_direction_history(
        const rclcpp::Time & stamp, const Eigen::Vector2d & direction)
    {
        prune_direction_history(stamp);
        direction_history_.push_back({stamp, direction});
        while (direction_history_.size() > MAX_DIRECTION_HISTORY) {
            direction_history_.pop_front();
        }
    }

    // [방향 이력 정리] 장시간 방향 손실 뒤 과거 방향이 안정성 판정에 재사용되지 않도록 한다.
    void prune_direction_history(const rclcpp::Time & stamp)
    {
        while (!direction_history_.empty() &&
            (stamp - direction_history_.front().stamp).seconds() >
            DIRECTION_HISTORY_MAX_AGE_S)
        {
            direction_history_.pop_front();
        }
    }

    // [방향 안정성 판정] 최근 방향 표준편차와 필요한 이력 길이가 준비 조건을 만족하는지 확인한다.
    bool direction_is_stable() const
    {
        return direction_history_.size() >= MIN_DIRECTION_STABILITY_SAMPLES &&
            direction_std_rad() <= MAX_STABLE_STD_RAD;
    }

    // [안정성 신뢰도] 최근 방향 분산을 최종 confidence에 곱할 0~1 계수로 변환한다.
    double stability_confidence() const
    {
        if (direction_history_.size() < 3) {
            return 0.45;
        }
        return std::clamp(
            1.0 - direction_std_rad() / MAX_STABLE_STD_RAD, 0.15, 1.0);
    }

    // [방향 분산 계산] 각도 wrap을 고려한 최근 world-frame 방향의 표준편차를 반환한다.
    double direction_std_rad() const
    {
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (const TimedDirection & sample : direction_history_) {
            mean += sample.direction;
        }
        if (mean.norm() < 1.0e-9) {
            return PI;
        }
        const double mean_angle = std::atan2(mean.y(), mean.x());
        double squared_error = 0.0;
        for (const TimedDirection & sample : direction_history_) {
            const double angle =
                std::atan2(sample.direction.y(), sample.direction.x());
            const double error =
                std::atan2(std::sin(angle - mean_angle), std::cos(angle - mean_angle));
            squared_error += error * error;
        }
        return std::sqrt(
            squared_error / static_cast<double>(direction_history_.size()));
    }

    // [방향 저역통과] 급격한 방향 변화를 완화하도록 이전 결과와 새 단위 벡터를 혼합한다.
    Eigen::Vector2d filter_direction(const Eigen::Vector2d & direction)
    {
        if (!have_filtered_direction_) {
            filtered_direction_ = direction;
            have_filtered_direction_ = true;
            return filtered_direction_;
        }
        filtered_direction_ =
            (1.0 - DIRECTION_FILTER_ALPHA) * filtered_direction_ +
            DIRECTION_FILTER_ALPHA * direction;
        if (filtered_direction_.norm() < 1.0e-9) {
            filtered_direction_ = direction;
        } else {
            filtered_direction_.normalize();
        }
        return filtered_direction_;
    }

    // [방향 발행] world 방향을 현재 yaw로 base_link에 변환하고 confidence와 함께 발행한다.
    void publish_direction(
        const rclcpp::Time & stamp,
        const Eigen::Vector2d & world_direction,
        const double confidence)
    {
        const double c = std::cos(current_yaw_rad_);
        const double s = std::sin(current_yaw_rad_);
        geometry_msgs::msg::Vector3Stamped direction_msg;
        direction_msg.header.stamp = stamp;
        direction_msg.header.frame_id = "base_link";
        direction_msg.vector.x =
            c * world_direction.x() + s * world_direction.y();
        direction_msg.vector.y =
            -s * world_direction.x() + c * world_direction.y();
        direction_msg.vector.z = 0.0;
        direction_pub_->publish(direction_msg);

        std_msgs::msg::Float64 confidence_msg;
        confidence_msg.data = std::clamp(confidence, 0.0, 1.0);
        confidence_pub_->publish(confidence_msg);
    }

    // [고-SNR 영역 발행] 방문 영역에서 관측된 SNR hotspot 중심을 진단용 점으로 발행한다.
    void publish_high_snr_region(
        const rclcpp::Time & stamp, const Eigen::Vector2d & region)
    {
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = odometry_frame_id_;
        msg.point.x = region.x();
        msg.point.y = region.y();
        msg.point.z = current_position_m_.z();
        high_snr_region_pub_->publish(msg);
    }

    // [모델 방향 발행] Local/Map world gradient를 body-frame 디버그 화살표로 변환한다.
    void publish_model_direction(
        const rclcpp::Time & stamp,
        const Eigen::Vector2d & world_direction,
        const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr & publisher)
    {
        const double c = std::cos(current_yaw_rad_);
        const double s = std::sin(current_yaw_rad_);
        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "base_link";
        msg.vector.x = c * world_direction.x() + s * world_direction.y();
        msg.vector.y = -s * world_direction.x() + c * world_direction.y();
        msg.vector.z = 0.0;
        publisher->publish(msg);
    }

    // [추정 상태 발행] 방향 homing 제어기가 사용하는 estimator-ready 상태만 발행한다.
    void publish_status(const bool ready)
    {
        std_msgs::msg::Bool ready_msg;
        ready_msg.data = ready;
        estimator_ready_pub_->publish(ready_msg);
    }

    // [중앙값 계산] 임의 컨테이너 값을 복사해 홀수·짝수 표본 모두의 median을 반환한다.
    template<typename Container>
    static double median(const Container & values)
    {
        if (values.empty()) {
            return 0.0;
        }
        std::vector<double> sorted(values.begin(), values.end());
        const std::size_t middle = sorted.size() / 2;
        std::nth_element(sorted.begin(), sorted.begin() + middle, sorted.end());
        const double upper = sorted[middle];
        if (sorted.size() % 2 != 0) {
            return upper;
        }
        std::nth_element(sorted.begin(), sorted.begin() + middle - 1, sorted.end());
        return 0.5 * (sorted[middle - 1] + upper);
    }

    // [Yaw 변환] odometry quaternion에서 평면 방향 변환에 필요한 yaw를 계산한다.
    static double yaw_from_quaternion(
        const double w, const double x, const double y, const double z)
    {
        const double sin_yaw = 2.0 * (w * z + x * y);
        const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(sin_yaw, cos_yaw);
    }

    double map_cell_size_m_ = 0.12;
    double local_radius_m_ = 0.75;
    double map_radius_m_ = 2.0;

    Eigen::Vector3d current_position_m_{0.0, 0.0, 0.0};
    double current_yaw_rad_ = 0.0;
    std::string odometry_frame_id_ = "odom";
    bool have_odometry_ = false;

    std::deque<Sample> raw_samples_;
    std::deque<PoseSample> odometry_history_;
    std::deque<PendingSnr> pending_snr_;
    std::map<std::pair<int, int>, GridCell> grid_;
    std::deque<TimedDirection> direction_history_;
    Eigen::Vector2d filtered_direction_{1.0, 0.0};
    bool have_filtered_direction_ = false;
    rclcpp::Time last_processed_stamp_;
    bool have_last_processed_stamp_ = false;
    std::map<int, std::deque<double>> vertical_bins_;
    double last_vertical_sample_z_m_ = 0.0;
    bool have_last_vertical_sample_z_ = false;
    bool vertical_search_active_ = false;
    bool vertical_search_requested_ = false;
    rclcpp::Time vertical_trigger_start_;
    bool have_vertical_trigger_start_ = false;

    rclcpp::Subscription<audio_common_msgs::msg::Float64Stamped>::SharedPtr snr_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vertical_search_active_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr confidence_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr high_snr_region_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr local_direction_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr map_direction_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estimator_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vertical_search_request_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vertical_best_z_pub_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::SnrGradientHomingNodeV2)
