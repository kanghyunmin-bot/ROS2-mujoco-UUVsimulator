#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

namespace audio_capture
{
// Region Scan과 Homing의 거리 기반 SNR 표본으로 각각 2D Gradient를 계산한다.
class RegionLocalGradientEstimatorNode : public rclcpp::Node
{
public:
    explicit RegionLocalGradientEstimatorNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("region_local_gradient_estimator", options)
    {
        const auto snr_topic = declare_parameter<std::string>(
            "snr_topic", "/audio_frequency_detector/snr_db_stamped");
        const auto odometry_topic = declare_parameter<std::string>(
            "odometry_topic", "/odometry/filtered");
        const auto state_topic = declare_parameter<std::string>(
            "state_topic", "/homing/control_state");
        const auto region_gradient_topic = declare_parameter<std::string>(
            "region_gradient_topic", "/homing/region_gradient");
        const auto rolling_gradient_topic = declare_parameter<std::string>(
            "rolling_gradient_topic", "/homing/rolling_gradient");
        region_sample_spacing_m_ = std::max(
            0.01, declare_parameter<double>("region_sample_spacing_m", 0.15));
        homing_gradient_window_size_ =
            static_cast<std::size_t>(std::max<std::int64_t>(
            3,
            declare_parameter<std::int64_t>("homing_gradient_window_size", 12)));
        min_homing_gradient_samples_ = static_cast<std::size_t>(
            std::clamp<std::int64_t>(
                declare_parameter<std::int64_t>("min_homing_gradient_samples", 8),
                3, static_cast<std::int64_t>(homing_gradient_window_size_)));
        min_region_gradient_magnitude_ = std::max(
            0.0, declare_parameter<double>("min_region_gradient_magnitude", 0.05));
        min_region_lateral_spread_m_ = std::max(
            0.0, declare_parameter<double>("min_region_lateral_spread_m", 0.10));
        odometry_timeout_s_ = std::max(
            0.05, declare_parameter<double>("odometry_timeout_s", 0.5));
        max_snr_odom_skew_s_ = std::max(
            0.0, declare_parameter<double>("max_snr_odom_skew_s", 0.15));

        snr_sub_ = create_subscription<audio_common_msgs::msg::Float64Stamped>(
            snr_topic, 20,
            std::bind(&RegionLocalGradientEstimatorNode::snr_callback, this,
                std::placeholders::_1));
        odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 30,
            std::bind(&RegionLocalGradientEstimatorNode::odometry_callback, this,
                std::placeholders::_1));
        state_sub_ = create_subscription<std_msgs::msg::String>(
            state_topic, rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&RegionLocalGradientEstimatorNode::state_callback, this,
                std::placeholders::_1));
        region_gradient_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            region_gradient_topic, rclcpp::QoS(1).reliable().transient_local());
        rolling_gradient_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>(
                rolling_gradient_topic, 10);

        RCLCPP_INFO(
            get_logger(),
            "Region/Rolling Gradient estimator ready: spacing=%.2f m "
            "homing_window=%zu samples.",
            region_sample_spacing_m_, homing_gradient_window_size_);
    }



private:
    struct Sample
    {
        Eigen::Vector2d position{0.0, 0.0};
        double snr_db = 0.0;
    };

    struct PoseSample
    {
        rclcpp::Time stamp;
        Eigen::Vector2d position{0.0, 0.0};
    };



    // 오돔 메세지를 받아 해당 지점의 위치와 시각 히스토리를 업데이트하는 콜백. --> odometry_history_ 업데이트 콜백
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        const Eigen::Vector2d position(
            msg->pose.pose.position.x, msg->pose.pose.position.y); // 오돔 메세지에서 현재 위치 추출
        if (!position.allFinite()) { // 위치가 유효하지 않으면 무시.
            return;
        }
        const rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() <= 0) { // 타임스탬프가 없어도 무시.
            return;
        }
        if (!odometry_history_.empty() && stamp < odometry_history_.back().stamp) { //현재의 시각이 최근 시각보다 이전이면 히스토리를 초기화한다.
            odometry_history_.clear(); 
        }
        odometry_history_.push_back({stamp, position}); // 타임스탬프와 위치를 히스토리에 추가한다.
        while (odometry_history_.size() > 300) { // 히스토리 길이를 최대 300개로 제한한다.
            odometry_history_.pop_front();
        }
        odometry_frame_ = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id; // 오돔 메세지가 어느 좌표계 기준인지 업데이트한다 (빈 문자열이면 odom 기준).
        last_odometry_receive_time_ = now(); // 마지막 오도메트리 수신 시간을 업데이트한다.
    }



    // 제어기 상태를 받아 상태 변이 시 준비 작업을 수행하는 콜백. --> 제어기 상태 변이 콜백
    void state_callback(const std_msgs::msg::String::ConstSharedPtr msg)
    {
        if (msg->data == state_) { // 현재 상태와 받은 상태가 같으면 무시.
            return;
        }
        state_ = msg->data;
        if (state_ == "REGION_SCAN") {
            scan_samples_.clear();
        }
        if (state_ == "REGION_SCAN" || state_ == "REGION_HOMING") {
            reset_homing_window();
        }
    }


    // SNR 메세지를 받아 해당 지점의 위치, 시각, snr을 업데이트하는 콜백. --> snr 수신 콜백
    void snr_callback(const audio_common_msgs::msg::Float64Stamped::ConstSharedPtr msg)
    {
        if (!std::isfinite(msg->data) || !odometry_is_fresh()) {
            return;
        }
        Eigen::Vector2d position;
        const rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() <= 0 || !position_at(stamp, position)) {
            return;
        }

        if (state_ == "REGION_SCAN") {
            record_scan_sample(stamp, position, msg->data);
        } else if (state_ == "REGION_HOMING") {
            record_homing_sample(stamp, position, msg->data);
        }
    }


    // REGION_SCAN 이동 중 일정 거리마다 SNR을 저장하고 최신 Region Gradient를 계산한다.
    void record_scan_sample(
        const rclcpp::Time & stamp, const Eigen::Vector2d & position, const double snr_db)
    {
        if (!scan_samples_.empty() &&
            (position - scan_samples_.back().position).norm() < region_sample_spacing_m_)
        {
            return;
        }
        scan_samples_.push_back({position, snr_db});
        RCLCPP_DEBUG(
            get_logger(), "REGION_SCAN sample stored: count=%zu position=(%.2f, %.2f) "
            "snr=%.2f dB", scan_samples_.size(),
            position.x(), position.y(), snr_db);
        if (scan_samples_.size() < 3) {
            return;
        }

        Eigen::Vector2d gradient;
        double magnitude = 0.0;
        double lateral_spread = 0.0;
        const bool valid = fit_gradient(
            scan_samples_, gradient, magnitude, lateral_spread);
        publish_gradient(region_gradient_pub_, stamp, valid ? gradient : Eigen::Vector2d::Zero());
        RCLCPP_DEBUG(
            get_logger(),
            "Region Scan fit: samples=%zu valid=%s magnitude=%.3f spread=%.3f "
            "gradient=(%.3f, %.3f)",
            scan_samples_.size(), valid ? "true" : "false",
            magnitude, lateral_spread,
            valid ? gradient.x() : 0.0, valid ? gradient.y() : 0.0);
    }

    void record_homing_sample(
        const rclcpp::Time & stamp, const Eigen::Vector2d & position, const double snr_db)
    {
        if (!last_homing_position_) {
            last_homing_position_ = position;
        } else {
            const double displacement =
                (position - *last_homing_position_).norm();
            if (displacement < region_sample_spacing_m_) {
                return;
            }
            last_homing_position_ = position;
        }

        homing_samples_.push_back({position, snr_db});
        while (homing_samples_.size() > homing_gradient_window_size_) {
            homing_samples_.pop_front();
        }
        RCLCPP_DEBUG(
            get_logger(),
            "REGION_HOMING sample stored: window=%zu/%zu snr=%.2f dB",
            homing_samples_.size(), homing_gradient_window_size_, snr_db);
        if (homing_samples_.size() >= min_homing_gradient_samples_) {
            const std::vector<Sample> samples(
                homing_samples_.begin(), homing_samples_.end());
            Eigen::Vector2d gradient;
            double magnitude = 0.0;
            double lateral_spread = 0.0;
            if (fit_gradient(samples, gradient, magnitude, lateral_spread)) {
                publish_gradient(rolling_gradient_pub_, stamp, gradient);
            }
        }
    }

    void reset_homing_window()
    {
        homing_samples_.clear();
        last_homing_position_.reset();
    }

    bool fit_gradient(
        const std::vector<Sample> & samples,
        Eigen::Vector2d & normalized_gradient,
        double & magnitude,
        double & lateral_spread) const
    {
        if (samples.size() < 3) {
            return false;
        }
        Eigen::Vector2d center;
        if (!sample_geometry(samples, center, lateral_spread)) {
            return false;
        }

        Eigen::MatrixXd design(samples.size(), 3);
        Eigen::VectorXd values(samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const Eigen::Vector2d delta = samples[i].position - center;
            design(static_cast<Eigen::Index>(i), 0) = delta.x();
            design(static_cast<Eigen::Index>(i), 1) = delta.y();
            design(static_cast<Eigen::Index>(i), 2) = 1.0;
            values(static_cast<Eigen::Index>(i)) = samples[i].snr_db;
        }
        const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
        if (qr.rank() < 3) {
            return false;
        }
        const Eigen::VectorXd coefficients = qr.solve(values);
        const Eigen::Vector2d gradient(coefficients(0), coefficients(1));
        magnitude = gradient.norm();
        if (!gradient.allFinite() || !std::isfinite(magnitude) ||
            magnitude < min_region_gradient_magnitude_ ||
            lateral_spread < min_region_lateral_spread_m_)
        {
            return false;
        }
        normalized_gradient = gradient / magnitude;
        return true;
    }

    bool sample_geometry(
        const std::vector<Sample> & samples,
        Eigen::Vector2d & center,
        double & lateral_spread) const
    {
        if (samples.empty()) {
            return false;
        }
        center = Eigen::Vector2d::Zero();
        for (const Sample & sample : samples) {
            center += sample.position;
        }
        center /= static_cast<double>(samples.size());

        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        for (const Sample & sample : samples) {
            const Eigen::Vector2d delta = sample.position - center;
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<double>(samples.size());
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(covariance);
        if (eigen_solver.info() != Eigen::Success) {
            return false;
        }
        lateral_spread =
            std::sqrt(std::max(0.0, eigen_solver.eigenvalues().minCoeff()));
        return std::isfinite(lateral_spread);
    }



    //SNR 타임스탬프에 근접한 2D 위치를 odometry 히스토리에서 찾아 주는 함수
    bool position_at(const rclcpp::Time & stamp, Eigen::Vector2d & position) const
    {
        if (odometry_history_.empty()) { //히스토리가 비어있으면 무시.
            return false;
        }
        if (stamp <= odometry_history_.front().stamp) {
            if ((odometry_history_.front().stamp - stamp).seconds() > max_snr_odom_skew_s_) {
                return false;
            }
            // 현재 들어온 snr의 시각의 오돔 시각보다 늦은 경우(사실상 거의 없지만) 허용오차 내면 가장 가까운 오돔 위치를 사용
            position = odometry_history_.front().position;
            return true;
        }
        if (stamp >= odometry_history_.back().stamp) { 
            if ((stamp - odometry_history_.back().stamp).seconds() > max_snr_odom_skew_s_) {
                return false;
            }
            //현재 들어온 snr의 시각의 오돔 시각보다 빠른 경우 허용오차 내면 가장 최근 오돔 위치를 사용
            position = odometry_history_.back().position;
            return true;
        }
        // 중간에 오돔 히스토리 내에 snr시각이 존재하면 중간 지점으로 보간
        for (std::size_t i = 1; i < odometry_history_.size(); ++i) {
            if (odometry_history_[i].stamp >= stamp) {
                const PoseSample & before = odometry_history_[i - 1];
                const PoseSample & after = odometry_history_[i];
                const double duration = (after.stamp - before.stamp).seconds();
                const double alpha = duration > 0.0 ?
                    (stamp - before.stamp).seconds() / duration : 0.0;
                position = (1.0 - alpha) * before.position + alpha * after.position;
                return true;
            }
        }
        return false;
    }

    bool odometry_is_fresh() const
    {
        return !odometry_history_.empty() &&
            (now() - last_odometry_receive_time_).seconds() <= odometry_timeout_s_;
    }

    void publish_gradient(
        const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr & publisher,
        const rclcpp::Time & stamp,
        const Eigen::Vector2d & gradient) const
    {
        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = odometry_frame_;
        msg.vector.x = gradient.x();
        msg.vector.y = gradient.y();
        publisher->publish(msg);
    }

    // TODO: 안정된 수평 최대 SNR 영역 판단과 Depth Sweep은 다음 단계에서 추가한다.

    std::size_t homing_gradient_window_size_ = 12;
    std::size_t min_homing_gradient_samples_ = 8;
    double region_sample_spacing_m_ = 0.15;
    double min_region_gradient_magnitude_ = 0.05;
    double min_region_lateral_spread_m_ = 0.10;
    double odometry_timeout_s_ = 0.5;
    double max_snr_odom_skew_s_ = 0.15;

    std::string state_;
    std::string odometry_frame_ = "odom";
    rclcpp::Time last_odometry_receive_time_;
    std::optional<Eigen::Vector2d> last_homing_position_;
    std::deque<PoseSample> odometry_history_; //위치와 해당 시각을 저장하는 큐
    std::vector<Sample> scan_samples_;
    std::deque<Sample> homing_samples_;

    rclcpp::Subscription<audio_common_msgs::msg::Float64Stamped>::SharedPtr snr_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr region_gradient_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rolling_gradient_pub_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::RegionLocalGradientEstimatorNode)
