#include <cstdint>

#include <algorithm>
#include <cmath>
#include <complex>
#include <deque>
#include <memory>
#include <vector>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <audio_common_msgs/msg/audio_data_stamped.hpp>
#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>

namespace audio_capture
{
class AudioPhaseEstimatorNode : public rclcpp::Node
{
public:
    // [노드 초기화] 오디오·위치 입력, 분석 파라미터, 진단 출력을 구성하고 분석 스레드를 시작한다.
    explicit AudioPhaseEstimatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("audio_phase_estimator", options)
    {
        audio_topic_ = this->declare_parameter<std::string>("audio_topic", "/audio");
        audio_stamped_topic_ =
            this->declare_parameter<std::string>("audio_stamped_topic", "/audio_stamped");
        use_stamped_audio_ = this->declare_parameter<bool>("use_stamped_audio", false);
        odometry_topic_ =
            this->declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
        depth_topic_ = this->declare_parameter<std::string>("depth_topic", "/depth/pose");
        audio_input_latency_s_ = std::max(
            0.0, this->declare_parameter<double>("audio_input_latency_s", 0.0));
        if (use_stamped_audio_) {
            audio_stamped_sub_ =
                this->create_subscription<audio_common_msgs::msg::AudioDataStamped>(
                    audio_stamped_topic_,
                    10,
                    std::bind(
                        &AudioPhaseEstimatorNode::audio_stamped_callback,
                        this,
                        std::placeholders::_1));
        } else {
            audio_sub_ = this->create_subscription<audio_common_msgs::msg::AudioData>(
                audio_topic_,
                10,
                std::bind(&AudioPhaseEstimatorNode::audio_callback, this, std::placeholders::_1));
        }
        dvl_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_,
            10,
            std::bind(&AudioPhaseEstimatorNode::dvl_odometry_callback, this, std::placeholders::_1));
        depth_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            depth_topic_,
            10,
            std::bind(&AudioPhaseEstimatorNode::depth_pose_callback, this, std::placeholders::_1));
        homing_direction_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/homing/direction", 10);
        demodulation_frequency_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/audio_phase_estimator/demodulation_frequency_hz", 10);
        iq_snr_ratio_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/audio_phase_estimator/iq_snr_ratio", 10);
        iq_snr_ratio_stamped_pub_ =
            this->create_publisher<audio_common_msgs::msg::Float64Stamped>(
                "/audio_phase_estimator/iq_snr_ratio_stamped", 10);
        iq_coherence_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/audio_phase_estimator/iq_coherence", 10);
        reference_frequency_hz_ =
            this->declare_parameter<double>("reference_frequency_hz", reference_frequency_hz_);
        demodulation_frequency_hz_ = this->declare_parameter<double>(
            "initial_demodulation_frequency_hz",
            reference_frequency_hz_);
        sound_speed_mps_ = this->declare_parameter<double>("sound_speed_mps", sound_speed_mps_);
        const int window_size_param = static_cast<int>(
            this->declare_parameter<int>("window_size", static_cast<int>(window_size_)));
        window_size_ = static_cast<std::size_t>(std::max(1, window_size_param));
        const int hop_size_param = static_cast<int>(
            this->declare_parameter<int>("hop_size", static_cast<int>(hop_size_)));
        hop_size_ = static_cast<std::size_t>(std::max(1, hop_size_param));
        hop_size_ = std::min(hop_size_, window_size_);
        sync_delay_s_ = this->declare_parameter<double>("sync_delay_s", 0.10);
        min_iq_magnitude_ = this->declare_parameter<double>("min_iq_magnitude", 1.0e-5);
        min_iq_snr_ratio_ = this->declare_parameter<double>("min_iq_snr_ratio", min_iq_snr_ratio_);
        min_iq_coherence_ = this->declare_parameter<double>("min_iq_coherence", min_iq_coherence_);
        const int coherence_segments_param = static_cast<int>(
            this->declare_parameter<int>("coherence_segments", static_cast<int>(coherence_segments_)));
        coherence_segments_ = static_cast<std::size_t>(std::max(1, coherence_segments_param));
        direction_filter_alpha_ = this->declare_parameter<double>("direction_filter_alpha", 0.12);
        homing_accumulation_time_s_ =
            this->declare_parameter<double>("homing_accumulation_time_s", homing_accumulation_time_s_);
        publish_homing_direction_ =
            this->declare_parameter<bool>("publish_homing_direction", publish_homing_direction_);
        enable_frequency_acquisition_ =
            this->declare_parameter<bool>("enable_frequency_acquisition", enable_frequency_acquisition_);
        frequency_search_half_width_hz_ = this->declare_parameter<double>(
            "frequency_search_half_width_hz",
            frequency_search_half_width_hz_);
        frequency_search_step_hz_ = this->declare_parameter<double>(
            "frequency_search_step_hz",
            frequency_search_step_hz_);
        frequency_reacquire_threshold_hz_ = this->declare_parameter<double>(
            "frequency_reacquire_threshold_hz",
            frequency_reacquire_threshold_hz_);
        const int frequency_lock_required_windows_param = static_cast<int>(
            this->declare_parameter<int>(
                "frequency_lock_required_windows",
                frequency_lock_required_windows_));
        frequency_lock_required_windows_ = std::max(1, frequency_lock_required_windows_param);
        frequency_lock_tolerance_hz_ = this->declare_parameter<double>(
            "frequency_lock_tolerance_hz",
            frequency_lock_tolerance_hz_);

        RCLCPP_INFO(
            get_logger(),
            "Audio phase estimator ready. audio=%s stamped=%s odom=%s depth=%s input_latency=%.3fs",
            use_stamped_audio_ ? audio_stamped_topic_.c_str() : audio_topic_.c_str(),
            use_stamped_audio_ ? "true" : "false",
            odometry_topic_.c_str(),
            depth_topic_.c_str(),
            audio_input_latency_s_);

        worker_thread_ = std::thread(&AudioPhaseEstimatorNode::analysis_loop, this);
    }

    // [분석 스레드 종료] 대기 중인 worker를 깨운 뒤 join하여 노드를 안전하게 종료한다.
    ~AudioPhaseEstimatorNode()
    {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            stop_worker_ = true;
        }
        buffer_cv_.notify_one();

        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

private:
    // 상태 x=[u_x,u_y,u_z,b]^T, 관측 z=Delta r=-u^T Delta p + b*Delta t.
    // b는 단일 하이드로폰에서 주파수/클럭 drift가 거리 변화처럼 보이는 bias range-rate[m/s]이다.
    class HomingDirectionEkf
    {
    public:
        // [EKF 관측 갱신] AUV 이동량과 음원 거리 변화량으로 방향 상태와 drift bias를 보정한다.
        bool update(
            const Eigen::Vector3d & delta_position_m,
            const double delta_range_m,
            const double delta_time_s)
        {
            if (delta_time_s <= 0.0) {
                return false;
            }

            covariance_ += process_noise_;

            Eigen::Matrix<double, 1, 4> measurement_jacobian;
            measurement_jacobian << -delta_position_m.x(), -delta_position_m.y(), -delta_position_m.z(), delta_time_s;
            const double innovation =
                delta_range_m - static_cast<double>(measurement_jacobian * state_);
            const double innovation_covariance =
                static_cast<double>(measurement_jacobian * covariance_ * measurement_jacobian.transpose()) +
                range_noise_variance_m2_;
            if (innovation_covariance <= 1.0e-12) {
                return false;
            }

            const Eigen::Matrix<double, 4, 1> kalman_gain =
                covariance_ * measurement_jacobian.transpose() / innovation_covariance;

            state_ += kalman_gain * innovation;
            covariance_ =
                (Eigen::Matrix4d::Identity() - kalman_gain * measurement_jacobian) * covariance_;
            normalize_direction_state();
            return delta_position_m.squaredNorm() >= min_motion_squared_m2_;
        }

        // [EKF 방향 조회] 내부 방향 상태를 단위 벡터로 반환하고 영벡터 상태는 거부한다.
        Eigen::Vector3d normalized_direction() const
        {
            const Eigen::Vector3d direction = state_.head<3>();
            const double norm = direction.norm();
            if (norm < min_direction_norm_) {
                return Eigen::Vector3d::Zero();
            }
            return direction / norm;
        }

    private:
        // [EKF 상태 정규화] 갱신된 방향 3축의 크기를 1로 맞춰 방향 벡터 제약을 유지한다.
        void normalize_direction_state()
        {
            Eigen::Vector3d direction = state_.head<3>();
            const double norm = direction.norm();
            if (norm < min_direction_norm_) {
                return;
            }
            state_.head<3>() = direction / norm;
        }

        Eigen::Matrix<double, 4, 1> state_{1.0, 0.0, 0.0, 0.0};
        Eigen::Matrix4d covariance_ = Eigen::Matrix4d::Identity() * 10.0;
        Eigen::Matrix4d process_noise_ = []() {
            Eigen::Matrix4d noise = Eigen::Matrix4d::Identity() * 1.0e-4;
            noise(3, 3) = 1.0e-3;
            return noise;
        }();
        double range_noise_variance_m2_ = 1.0e-3;
        double min_motion_squared_m2_ = 1.0e-6;
        double min_direction_norm_ = 1.0e-9;
    };
    struct TimedSample
    {
        double value = 0.0;
        rclcpp::Time stamp;
    };

    struct TimedVector2
    {
        rclcpp::Time stamp;
        Eigen::Vector2d value{0.0, 0.0};
    };

    struct TimedScalar
    {
        rclcpp::Time stamp;
        double value = 0.0;
    };

    struct IqQuality
    {
        double magnitude = 0.0;
        double noise_magnitude = 0.0;
        double snr_ratio = 0.0;
        double coherence = 0.0;
    };

    // [무타임스탬프 오디오 수신] 수신 시각과 버퍼 길이로 시작 시각을 추정해 PCM 큐에 넣는다.
    void audio_callback(const audio_common_msgs::msg::AudioData::ConstSharedPtr msg)
    {
        const rclcpp::Time buffer_start_stamp = estimate_audio_buffer_start_stamp(msg->data.size());
        append_audio_buffer(msg->data, buffer_start_stamp);
    }

    // [타임스탬프 오디오 수신] 메시지 header 시각을 우선 사용하고 0이면 수신 시각으로 대체한다.
    void audio_stamped_callback(
        const audio_common_msgs::msg::AudioDataStamped::ConstSharedPtr msg)
    {
        rclcpp::Time buffer_start_stamp(msg->header.stamp);
        if (buffer_start_stamp.nanoseconds() <= 0) {
            buffer_start_stamp = estimate_audio_buffer_start_stamp(msg->audio.data.size());
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Stamped audio has a zero header; using receive-time fallback.");
        }
        append_audio_buffer(msg->audio.data, buffer_start_stamp);
    }

    // [오디오 버퍼 적재] PCM 변환을 mutex로 보호하고 분석 worker에 새 데이터 도착을 알린다.
    void append_audio_buffer(
        const std::vector<uint8_t> & data, const rclcpp::Time & buffer_start_stamp)
    {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            append_samples_from_pcm(data, buffer_start_stamp);
        }
        buffer_cv_.notify_one();
    }

    // [수평 위치 수신] DVL odometry의 x·y를 시각과 함께 보관하고 오래된 버퍼를 정리한다.
    void dvl_odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(dvl_mutex_);
        odometry_buffer_.push_back({
            this->now(),
            Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y)});
        trim_old_samples(odometry_buffer_);
    }

    // [수심 위치 수신] 별도 수심 토픽의 z를 시각과 함께 보관하고 오래된 버퍼를 정리한다.
    void depth_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(dvl_mutex_);
        depth_buffer_.push_back({this->now(), msg->pose.pose.position.z});
        trim_old_samples(depth_buffer_);
    }

    // [3차원 위치 동기화] 요청 시각의 보간 x·y와 최신 유효 z를 결합해 AUV 위치를 만든다.
    bool interpolate_position(const rclcpp::Time & stamp, Eigen::Vector3d & position_m) const
    {
        Eigen::Vector2d xy;
        double z = 0.0;
        if (!interpolate_xy(stamp, xy) || !hold_depth(stamp, z)) {
            return false;
        }
        position_m = Eigen::Vector3d(xy.x(), xy.y(), z);
        return true;
    }

    // [수평 위치 보간] 요청 시각을 둘러싼 odometry 두 점 사이에서 x·y를 선형 보간한다.
    bool interpolate_xy(const rclcpp::Time & stamp, Eigen::Vector2d & value) const
    {
        if (odometry_buffer_.size() < 2 ||
            stamp < odometry_buffer_.front().stamp ||
            stamp > odometry_buffer_.back().stamp)
        {
            return false;
        }

        for (std::size_t i = 1; i < odometry_buffer_.size(); ++i) {
            if (stamp <= odometry_buffer_[i].stamp) {
                const double dt = (odometry_buffer_[i].stamp - odometry_buffer_[i - 1].stamp).seconds();
                if (dt <= 0.0) {
                    return false;
                }
                const double alpha = (stamp - odometry_buffer_[i - 1].stamp).seconds() / dt;
                value = odometry_buffer_[i - 1].value +
                    alpha * (odometry_buffer_[i].value - odometry_buffer_[i - 1].value);
                return true;
            }
        }
        return false;
    }

    // [수심 영차 유지] 요청 시각 이전에 도착한 가장 최근 수심값을 선택한다.
    bool hold_depth(const rclcpp::Time & stamp, double & value) const
    {
        if (depth_buffer_.empty() || stamp < depth_buffer_.front().stamp) {
            return false;
        }

        value = depth_buffer_.front().value;
        for (const auto & sample : depth_buffer_) {
            if (sample.stamp > stamp) {
                break;
            }
            value = sample.value;
        }
        return true;
    }

    template<typename SampleT>
    // [위치 버퍼 제한] 위치·수심 이력의 최대 개수를 넘은 가장 오래된 표본을 제거한다.
    void trim_old_samples(std::deque<SampleT> & buffer) const
    {
        while (buffer.size() > max_pose_buffer_size_) {
            buffer.pop_front();
        }
    }
    // [오디오 분석 루프] 충분한 샘플을 기다려 겹치는 window를 만들고 주기적으로 분석한다.
    void analysis_loop()
    {
        while (rclcpp::ok()) {
            std::vector<double> window;
            rclcpp::Time window_start_stamp;
            std::uint64_t window_start_sample = 0;

            {
                std::unique_lock<std::mutex> lock(buffer_mutex_);
                buffer_cv_.wait(lock, [this]() {
                    return stop_worker_ || sample_buffer_.size() >= window_size_ + sync_delay_samples();
                });

                if (stop_worker_) {
                    break;
                }

                window.reserve(window_size_);
                window_start_stamp = sample_buffer_.front().stamp;
                for (std::size_t i = 0; i < window_size_; ++i) {
                    window.push_back(sample_buffer_[i].value);
                }
                sample_buffer_.erase(sample_buffer_.begin(), sample_buffer_.begin() + hop_size_);
                window_start_sample = next_window_start_sample_;
                next_window_start_sample_ += hop_size_;
            }

            analyze_window(window, window_start_sample, window_start_stamp);
        }
    }

    // [동기화 지연 환산] 설정된 지연 시간을 현재 sampling rate 기준 샘플 개수로 바꾼다.
    std::size_t sync_delay_samples() const
    {
        return static_cast<std::size_t>(
            std::ceil(std::max(sync_delay_s_, 0.0) * static_cast<double>(sampling_rate_)));
    }

    // [단일 window 분석] 주파수 lock, IQ 품질, 위상차와 거리 변화량을 순서대로 계산한다.
    void analyze_window(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const rclcpp::Time & window_start_stamp)
    {
        const auto window_duration = rclcpp::Duration::from_seconds(
            static_cast<double>(window.size()) / static_cast<double>(sampling_rate_));
        const rclcpp::Time window_center_stamp =
            window_start_stamp + rclcpp::Duration::from_seconds(0.5 * window_duration.seconds());

        update_demodulation_frequency(window, window_start_sample);
        if (enable_frequency_acquisition_ && !have_frequency_lock_) {
            have_previous_iq_ = false;
            reset_homing_accumulator();
            return;
        }

        // coarse lock된 주파수로 복조해 baseband 복소값 z_k = I + jQ를 얻는다.
        const std::complex<double> iq = demodulate_iq(window, window_start_sample, demodulation_frequency_hz_);//Z_k = x[n] * exp(-j 2*pi*f_demod*t)
        const IqQuality iq_quality =
            estimate_iq_quality(window, window_start_sample, demodulation_frequency_hz_, iq);
        publish_iq_quality_debug(iq_quality, window_center_stamp);
        if (iq_quality.magnitude < min_iq_magnitude_) {
            have_previous_iq_ = false;
            reset_homing_accumulator();
            return;
        }
        if (iq_quality.snr_ratio < min_iq_snr_ratio_ || iq_quality.coherence < min_iq_coherence_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "phase update skipped: weak IQ quality, |z| %.6f, snr %.2f, coherence %.2f",
                iq_quality.magnitude,
                iq_quality.snr_ratio,
                iq_quality.coherence);
            have_previous_iq_ = false;
            reset_homing_accumulator();
            return;
        }

        double delta_phase_rad = 0.0;  //delta_theta_k = theta_k - theta_k-1
        double delta_range_m = 0.0; // Delta r = -lambda * Delta theta / (2*pi)
        if (have_previous_iq_) {  //theta_k-1가 있으면
            double delta_time_s = (window_center_stamp - previous_iq_stamp_).seconds();
            if (delta_time_s <= 0.0) {
                delta_time_s = window_duration.seconds();
            }

            // 두 window의 켤레곱을 쓰면 -pi~pi 범위의 안정적인 위상차를 바로 얻을 수 있다.
            const std::complex<double> phase_step = iq * std::conj(previous_iq_);//Z_k * Z_k-1^*
            delta_phase_rad = std::atan2(std::imag(phase_step), std::real(phase_step));
            delta_range_m = -current_wavelength_m() * delta_phase_rad / (2.0 * M_PI);
            accumulate_homing_observation(
                delta_range_m,
                delta_time_s,
                previous_iq_stamp_,
                window_center_stamp);
        }
        previous_iq_ = iq;
        previous_iq_stamp_ = window_center_stamp;
        have_previous_iq_ = true;
    }

    // [전체 window IQ 복조] Hann window와 복소 혼합을 적용해 기준 주파수의 baseband 평균을 구한다.
    std::complex<double> demodulate_iq(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const double frequency_hz) const
    {
        std::complex<double> baseband_sum(0.0, 0.0);
        double weight_sum = 0.0;
        const double phase_step = 2.0 * M_PI * frequency_hz / static_cast<double>(sampling_rate_);
        const double hann_denominator = static_cast<double>(std::max<std::size_t>(1, window.size() - 1));

        for (std::size_t n = 0; n < window.size(); ++n) {
            const double weight =
                0.5 * (1.0 - std::cos((2.0 * M_PI * static_cast<double>(n)) / hann_denominator));
            // 1) I/Q 복조: x[n] * exp(-j 2*pi*f_ref*t)로 carrier를 baseband로 내린다.
            const double phase = phase_step * static_cast<double>(window_start_sample + n);
            const std::complex<double> mixed_sample =
                weight * window[n] * std::complex<double>(std::cos(phase), -std::sin(phase));
            baseband_sum += mixed_sample;
            weight_sum += weight;
        }

        // 2) LPF: 한 window 동안의 baseband 평균을 내서 2*f_ref 성분과 빠른 흔들림을 제거한다.
        return baseband_sum / std::max(weight_sum, 1.0e-12);
    }

    // [구간 IQ 복조] coherence 계산용으로 window 일부 구간의 복소 평균을 구한다.
    std::complex<double> demodulate_iq_segment(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const std::size_t offset,
        const std::size_t count,
        const double frequency_hz) const
    {
        std::complex<double> baseband_sum(0.0, 0.0);
        const double phase_step = 2.0 * M_PI * frequency_hz / static_cast<double>(sampling_rate_);
        const std::size_t end = std::min(window.size(), offset + count);
        for (std::size_t n = offset; n < end; ++n) {
            const double phase = phase_step * static_cast<double>(window_start_sample + n);
            baseband_sum += window[n] * std::complex<double>(std::cos(phase), -std::sin(phase));
        }
        const std::size_t used_count = end > offset ? end - offset : 0;
        if (used_count == 0) {
            return {0.0, 0.0};
        }
        return baseband_sum / static_cast<double>(used_count);
    }

    // [IQ 위상 일관성 계산] 여러 구간 IQ가 같은 위상을 유지하는 정도를 0~1로 산출한다.
    double estimate_iq_coherence(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const double frequency_hz) const
    {
        const std::size_t segments = std::max<std::size_t>(1, std::min(coherence_segments_, window.size()));
        const std::size_t segment_size = std::max<std::size_t>(1, window.size() / segments);
        std::complex<double> vector_sum(0.0, 0.0);
        double magnitude_sum = 0.0;

        for (std::size_t segment = 0; segment < segments; ++segment) {
            const std::size_t offset = segment * segment_size;
            if (offset >= window.size()) {
                break;
            }
            const std::size_t count = segment == segments - 1 ? window.size() - offset : segment_size;
            const std::complex<double> segment_iq =
                demodulate_iq_segment(window, window_start_sample, offset, count, frequency_hz);
            vector_sum += segment_iq;
            magnitude_sum += std::abs(segment_iq);
        }

        if (magnitude_sum <= 1.0e-12) {
            return 0.0;
        }
        return std::clamp(std::abs(vector_sum) / magnitude_sum, 0.0, 1.0);
    }

    // [IQ 품질 평가] target 크기, 주변 주파수 noise, SNR 비율과 coherence를 한 번에 계산한다.
    IqQuality estimate_iq_quality(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        const double frequency_hz,
        const std::complex<double> & target_iq) const
    {
        IqQuality quality;
        quality.magnitude = std::abs(target_iq);

        std::vector<double> noise_magnitudes;
        noise_magnitudes.reserve(6);
        const double offsets_hz[] = {-700.0, -450.0, -250.0, 250.0, 450.0, 700.0};
        for (const double offset_hz : offsets_hz) {
            const double probe_frequency_hz = frequency_hz + offset_hz;
            if (probe_frequency_hz <= 1.0) {
                continue;
            }
            noise_magnitudes.push_back(
                std::abs(demodulate_iq(window, window_start_sample, probe_frequency_hz)));
        }

        if (!noise_magnitudes.empty()) {
            std::sort(noise_magnitudes.begin(), noise_magnitudes.end());
            quality.noise_magnitude = noise_magnitudes[noise_magnitudes.size() / 2];
        }
        quality.snr_ratio = quality.magnitude / std::max(quality.noise_magnitude, 1.0e-12);
        quality.coherence = estimate_iq_coherence(window, window_start_sample, frequency_hz);
        return quality;
    }

    // [현재 파장 계산] 음속을 현재 복조 주파수로 나누어 위상차→거리 변환 파장을 반환한다.
    double current_wavelength_m() const
    {
        return sound_speed_mps_ / std::max(demodulation_frequency_hz_, 1.0);
    }

    // [복조 주파수 획득] 연속 window의 peak 후보가 안정될 때 실제 복조 주파수를 lock한다.
    void update_demodulation_frequency(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample)
    {
        if (!enable_frequency_acquisition_ || have_frequency_lock_) {
            return;
        }

        double acquired_frequency_hz = demodulation_frequency_hz_;
        if (!estimate_peak_frequency_hz(window, window_start_sample, acquired_frequency_hz)) {
            return;
        }

        if (pending_frequency_count_ == 0 ||
            std::abs(acquired_frequency_hz - pending_frequency_hz_) > frequency_lock_tolerance_hz_)
        {
            pending_frequency_hz_ = acquired_frequency_hz;
            pending_frequency_count_ = 1;
        } else {
            ++pending_frequency_count_;
        }

        if (pending_frequency_count_ < frequency_lock_required_windows_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "frequency lock pending: %.3f Hz (%d/%d)",
                pending_frequency_hz_,
                pending_frequency_count_,
                frequency_lock_required_windows_);
            return;
        }

        acquired_frequency_hz = pending_frequency_hz_;
        const double frequency_step_hz = acquired_frequency_hz - demodulation_frequency_hz_;
        if (std::abs(frequency_step_hz) < frequency_reacquire_threshold_hz_) {
            demodulation_frequency_hz_ = acquired_frequency_hz;
            have_frequency_lock_ = true;
            publish_demodulation_frequency_debug();
            RCLCPP_WARN(
                this->get_logger(),
                "Acquired demodulation frequency %.3f Hz.",
                demodulation_frequency_hz_);
            return;
        }

        demodulation_frequency_hz_ = acquired_frequency_hz;
        have_frequency_lock_ = true;
        have_previous_iq_ = false;
        publish_demodulation_frequency_debug();
        RCLCPP_WARN(
            this->get_logger(),
            "Acquired demodulation frequency %.3f Hz; phase tracking reset.",
            demodulation_frequency_hz_);
    }

    // [주파수 peak 탐색] 기준 주파수 주변을 훑어 IQ 품질 기준을 통과한 최대 응답을 찾는다.
    bool estimate_peak_frequency_hz(
        const std::vector<double> & window,
        const std::uint64_t window_start_sample,
        double & peak_frequency_hz)
    {
        if (frequency_search_half_width_hz_ <= 0.0 || frequency_search_step_hz_ <= 0.0) {
            return false;
        }

        const double search_start_hz =
            std::max(1.0, reference_frequency_hz_ - frequency_search_half_width_hz_);
        const double search_end_hz = reference_frequency_hz_ + frequency_search_half_width_hz_;
        double best_magnitude = -1.0;
        double best_frequency_hz = reference_frequency_hz_;

        for (double frequency_hz = search_start_hz;
            frequency_hz <= search_end_hz + 0.5 * frequency_search_step_hz_;
            frequency_hz += frequency_search_step_hz_)
        {
            const double magnitude = std::abs(demodulate_iq(window, window_start_sample, frequency_hz));
            if (magnitude > best_magnitude) {
                best_magnitude = magnitude;
                best_frequency_hz = frequency_hz;
            }
        }

        if (best_magnitude < min_iq_magnitude_) {
            return false;
        }

        const std::complex<double> best_iq =
            demodulate_iq(window, window_start_sample, best_frequency_hz);
        const IqQuality quality =
            estimate_iq_quality(window, window_start_sample, best_frequency_hz, best_iq);
        if (quality.snr_ratio < min_iq_snr_ratio_ || quality.coherence < min_iq_coherence_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "frequency candidate rejected: %.3f Hz, |z| %.6f, snr %.2f, coherence %.2f",
                best_frequency_hz,
                quality.magnitude,
                quality.snr_ratio,
                quality.coherence);
            return false;
        }

        peak_frequency_hz = best_frequency_hz;
        return true;
    }

    // [PCM 디코딩] S32LE interleaved 입력에서 선택 채널을 정규화하고 샘플별 시각을 부여한다.
    void append_samples_from_pcm(const std::vector<uint8_t> & data, const rclcpp::Time & buffer_start_stamp)
    {
        // /audio는 S32LE 2채널 interleaved PCM이므로 선택한 채널만 double 샘플로 변환한다.
        const std::size_t channel_offset = channel_index_ * bytes_per_sample_;
        std::uint64_t frame_index = 0;
        for (std::size_t frame_start = 0; frame_start + frame_size_ <= data.size();
            frame_start += frame_size_)
        {
            const int32_t sample = read_int32_little_endian(data, frame_start + channel_offset);
            const auto sample_offset = rclcpp::Duration::from_seconds(
                static_cast<double>(frame_index) / static_cast<double>(sampling_rate_));
            sample_buffer_.push_back({
                static_cast<double>(sample) / 2147483648.0,
                buffer_start_stamp + sample_offset});
            ++frame_index;
        }
    }

    // [오디오 시작 시각 추정] 수신 시각에서 설정 latency와 버퍼 재생 시간을 빼서 PTS를 근사한다.
    rclcpp::Time estimate_audio_buffer_start_stamp(const std::size_t byte_count)
    {
        const std::size_t frame_count = byte_count / frame_size_;
        const auto buffer_duration = rclcpp::Duration::from_seconds(
            static_cast<double>(frame_count) / static_cast<double>(sampling_rate_));
        const auto configured_latency =
            rclcpp::Duration::from_seconds(audio_input_latency_s_);
        return this->now() - configured_latency - buffer_duration;
    }

    // [S32LE 샘플 읽기] 지정 byte offset의 4바이트 little-endian 값을 signed 32비트로 복원한다.
    int32_t read_int32_little_endian(const std::vector<uint8_t> & data, const std::size_t offset) const
    {
        const uint32_t raw =
            static_cast<uint32_t>(data[offset]) |
            (static_cast<uint32_t>(data[offset + 1]) << 8) |
            (static_cast<uint32_t>(data[offset + 2]) << 16) |
            (static_cast<uint32_t>(data[offset + 3]) << 24);
        return static_cast<int32_t>(raw);
    }

    // [IQ 진단 발행] SNR, stamped SNR, coherence 토픽을 동일 분석 결과로 발행한다.
    void publish_iq_quality_debug(
        const IqQuality & iq_quality, const rclcpp::Time & measurement_stamp)
    {
        std_msgs::msg::Float64 snr_ratio_msg;
        snr_ratio_msg.data = iq_quality.snr_ratio;
        iq_snr_ratio_pub_->publish(snr_ratio_msg);

        audio_common_msgs::msg::Float64Stamped stamped_msg;
        stamped_msg.header.stamp = measurement_stamp;
        stamped_msg.data = iq_quality.snr_ratio;
        iq_snr_ratio_stamped_pub_->publish(stamped_msg);

        std_msgs::msg::Float64 coherence_msg;
        coherence_msg.data = iq_quality.coherence;
        iq_coherence_pub_->publish(coherence_msg);
    }

    // [복조 주파수 진단 발행] 현재 lock된 복조 주파수를 모니터링 토픽으로 내보낸다.
    void publish_demodulation_frequency_debug()
    {
        std_msgs::msg::Float64 demodulation_frequency_msg;
        demodulation_frequency_msg.data = demodulation_frequency_hz_;
        demodulation_frequency_pub_->publish(demodulation_frequency_msg);
    }

    // [위상 관측 누적] 짧은 window별 거리 변화를 설정 시간만큼 합쳐 EKF 갱신 구간을 만든다.
    void accumulate_homing_observation(
        const double delta_range_m,
        const double delta_time_s,
        const rclcpp::Time & step_start_stamp,
        const rclcpp::Time & step_end_stamp)
    {
        if (delta_time_s <= 0.0) {
            reset_homing_accumulator();
            return;
        }

        if (!have_homing_accumulator_) {
            accumulated_homing_start_stamp_ = step_start_stamp;
            accumulated_homing_delta_range_m_ = 0.0;
            accumulated_homing_delta_time_s_ = 0.0;
            accumulated_homing_step_count_ = 0;
            have_homing_accumulator_ = true;
        }

        accumulated_homing_delta_range_m_ += delta_range_m;
        accumulated_homing_delta_time_s_ += delta_time_s;
        accumulated_homing_end_stamp_ = step_end_stamp;
        ++accumulated_homing_step_count_;

        if (accumulated_homing_delta_time_s_ < homing_accumulation_time_s_) {
            return;
        }

        update_homing_estimate(
            accumulated_homing_delta_range_m_,
            accumulated_homing_delta_time_s_,
            accumulated_homing_start_stamp_,
            accumulated_homing_end_stamp_);
        reset_homing_accumulator();
    }

    // [위상 관측 초기화] 누적 거리·시간·window 수를 지워 다음 homing 구간을 준비한다.
    void reset_homing_accumulator()
    {
        have_homing_accumulator_ = false;
        accumulated_homing_delta_range_m_ = 0.0;
        accumulated_homing_delta_time_s_ = 0.0;
        accumulated_homing_step_count_ = 0;
    }

    // [위상 기반 방향 갱신] 구간 양끝 위치와 누적 거리 변화로 EKF 방향을 계산해 선택적으로 발행한다.
    void update_homing_estimate(
        const double delta_range_m,
        const double delta_time_s,
        const rclcpp::Time & window_start_stamp,
        const rclcpp::Time & window_end_stamp)
    {
        Eigen::Vector3d start_position_m;
        Eigen::Vector3d end_position_m;
        {
            std::lock_guard<std::mutex> lock(dvl_mutex_);
            if (!interpolate_position(window_start_stamp, start_position_m) ||
                !interpolate_position(window_end_stamp, end_position_m))
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "homing update skipped: receive-time odometry/depth buffer does not cover homing segment [%.6f, %.6f]",
                    window_start_stamp.seconds(),
                    window_end_stamp.seconds());
                return;
            }
        }

        const Eigen::Vector3d delta_position_m = end_position_m - start_position_m;
        const bool direction_observable =
            homing_direction_ekf_.update(delta_position_m, delta_range_m, delta_time_s);
        if (!direction_observable) {
            return;
        }

        const Eigen::Vector3d direction = homing_direction_ekf_.normalized_direction();
        if (direction.isZero()) {
            return;
        }
        const Eigen::Vector3d filtered_direction = filter_direction(direction);
        if (!publish_homing_direction_) {
            return;
        }

        geometry_msgs::msg::Vector3Stamped direction_msg;
        direction_msg.header.stamp = this->now();
        direction_msg.header.frame_id = "dvl";
        direction_msg.vector.x = filtered_direction.x();
        direction_msg.vector.y = filtered_direction.y();
        direction_msg.vector.z = filtered_direction.z();
        homing_direction_pub_->publish(direction_msg);
    }

    // [방향 저역통과 필터] 이전 결과와 새 단위 벡터를 혼합하고 다시 정규화한다.
    Eigen::Vector3d filter_direction(const Eigen::Vector3d & direction)
    {
        const double alpha = std::clamp(direction_filter_alpha_, 0.0, 1.0);
        if (!have_filtered_direction_) {
            filtered_direction_ = direction;
            have_filtered_direction_ = true;
            return filtered_direction_;
        }

        filtered_direction_ = (1.0 - alpha) * filtered_direction_ + alpha * direction;
        const double norm = filtered_direction_.norm();
        if (norm < 1.0e-9) {
            filtered_direction_ = direction;
        } else {
            filtered_direction_ /= norm;
        }
        return filtered_direction_;
    }

    std::string audio_topic_ = "/audio";
    std::string audio_stamped_topic_ = "/audio_stamped";
    std::string odometry_topic_ = "/odometry/filtered";
    std::string depth_topic_ = "/depth/pose";
    bool use_stamped_audio_ = false;
    double audio_input_latency_s_ = 0.0;

    rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
    rclcpp::Subscription<audio_common_msgs::msg::AudioDataStamped>::SharedPtr audio_stamped_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr homing_direction_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr demodulation_frequency_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr iq_snr_ratio_pub_;
    rclcpp::Publisher<audio_common_msgs::msg::Float64Stamped>::SharedPtr iq_snr_ratio_stamped_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr iq_coherence_pub_;

    std::vector<TimedSample> sample_buffer_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::thread worker_thread_;
    bool stop_worker_ = false;

    std::size_t channels_ = 2;
    std::size_t channel_index_ = 0;
    std::size_t bytes_per_sample_ = 4;
    std::size_t frame_size_ = channels_ * bytes_per_sample_;

    std::size_t window_size_ = 4096;
    std::size_t hop_size_ = 1024;
    std::uint64_t next_window_start_sample_ = 0;
    std::size_t sampling_rate_ = 96000;
    double reference_frequency_hz_ = 21164.0; //27211.0도 있을 수 있음.
    double sound_speed_mps_ = 1500.0; // 수조/해역에 맞춰 보정할 음속.
    double demodulation_frequency_hz_ = 21164.0;
    bool enable_frequency_acquisition_ = true;
    bool have_frequency_lock_ = false;
    double frequency_search_half_width_hz_ = 1000.0;
    double frequency_search_step_hz_ = 10.0;
    double frequency_reacquire_threshold_hz_ = 50.0;
    int frequency_lock_required_windows_ = 5;
    double frequency_lock_tolerance_hz_ = 20.0;
    double pending_frequency_hz_ = 0.0;
    int pending_frequency_count_ = 0;
    double min_iq_magnitude_ = 1.0e-8;
    double min_iq_snr_ratio_ = 2.0;
    double min_iq_coherence_ = 0.25;
    std::size_t coherence_segments_ = 8;
    double sync_delay_s_ = 0.10;
    double direction_filter_alpha_ = 0.12;
    double homing_accumulation_time_s_ = 1.0;
    bool publish_homing_direction_ = true;

    bool have_previous_iq_ = false;
    std::complex<double> previous_iq_{0.0, 0.0};
    rclcpp::Time previous_iq_stamp_;
    bool have_homing_accumulator_ = false;
    rclcpp::Time accumulated_homing_start_stamp_;
    rclcpp::Time accumulated_homing_end_stamp_;
    double accumulated_homing_delta_range_m_ = 0.0;
    double accumulated_homing_delta_time_s_ = 0.0;
    int accumulated_homing_step_count_ = 0;
    HomingDirectionEkf homing_direction_ekf_;
    Eigen::Vector3d filtered_direction_{1.0, 0.0, 0.0};
    bool have_filtered_direction_ = false;
    std::mutex dvl_mutex_;
    std::deque<TimedVector2> odometry_buffer_;
    std::deque<TimedScalar> depth_buffer_;
    std::size_t max_pose_buffer_size_ = 200;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::AudioPhaseEstimatorNode)
