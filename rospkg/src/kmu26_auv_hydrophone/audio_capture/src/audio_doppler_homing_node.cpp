#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace audio_capture
{
class AudioDopplerHomingNode : public rclcpp::Node
{
public:
    explicit AudioDopplerHomingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("audio_doppler_homing", options)
    {
        audio_topic_ = this->declare_parameter<std::string>("audio_topic", audio_topic_);
        sampling_rate_ = static_cast<std::size_t>(
            std::max(1, static_cast<int>(this->declare_parameter<int>("sampling_rate", static_cast<int>(sampling_rate_)))));
        channels_ = static_cast<std::size_t>(
            std::max(1, static_cast<int>(this->declare_parameter<int>("channels", static_cast<int>(channels_)))));
        channel_index_ = static_cast<std::size_t>(
            std::clamp(
                static_cast<int>(this->declare_parameter<int>("channel_index", static_cast<int>(channel_index_))),
                0,
                static_cast<int>(channels_ - 1)));
        frame_size_ = channels_ * bytes_per_sample_;

        reference_frequency_hz_ =
            this->declare_parameter<double>("reference_frequency_hz", reference_frequency_hz_);
        baseline_frequency_hz_ = reference_frequency_hz_;
        frequency_search_half_width_hz_ =
            this->declare_parameter<double>("frequency_search_half_width_hz", frequency_search_half_width_hz_);
        tracking_search_half_width_hz_ =
            this->declare_parameter<double>("tracking_search_half_width_hz", tracking_search_half_width_hz_);
        frequency_tracking_alpha_ =
            this->declare_parameter<double>("frequency_tracking_alpha", frequency_tracking_alpha_);
        coarse_frequency_step_hz_ =
            this->declare_parameter<double>("coarse_frequency_step_hz", coarse_frequency_step_hz_);
        fine_frequency_step_hz_ =
            this->declare_parameter<double>("fine_frequency_step_hz", fine_frequency_step_hz_);
        fine_search_half_width_hz_ =
            this->declare_parameter<double>("fine_search_half_width_hz", fine_search_half_width_hz_);
        snapshot_duration_s_ =
            this->declare_parameter<double>("snapshot_duration_s", snapshot_duration_s_);
        snapshot_hop_s_ =
            this->declare_parameter<double>("snapshot_hop_s", snapshot_hop_s_);
        control_period_s_ =
            this->declare_parameter<double>("control_period_s", control_period_s_);
        snr_threshold_ =
            this->declare_parameter<double>("snr_threshold", snr_threshold_);
        detection_ratio_threshold_ =
            this->declare_parameter<double>("detection_ratio_threshold", detection_ratio_threshold_);
        bad_frequency_rate_hzps_ =
            this->declare_parameter<double>("bad_frequency_rate_hzps", bad_frequency_rate_hzps_);
        trend_deadband_hzps_ =
            this->declare_parameter<double>("trend_deadband_hzps", trend_deadband_hzps_);
        const int trend_history_cycles_param = static_cast<int>(
            this->declare_parameter<int>("trend_history_cycles", trend_history_cycles_));
        trend_history_cycles_ = std::max(2, trend_history_cycles_param);
        const int baseline_cycles_param = static_cast<int>(
            this->declare_parameter<int>("baseline_cycles", baseline_cycles_));
        baseline_cycles_ = std::max(1, baseline_cycles_param);
        doppler_sign_ =
            this->declare_parameter<double>("doppler_sign", doppler_sign_);
        turn_command_magnitude_ =
            this->declare_parameter<double>("turn_command_magnitude", turn_command_magnitude_);
        search_turn_command_ =
            this->declare_parameter<double>("search_turn_command", search_turn_command_);

        snapshot_size_ = seconds_to_samples(snapshot_duration_s_);
        hop_size_ = std::min(seconds_to_samples(snapshot_hop_s_), snapshot_size_);

        audio_sub_ = this->create_subscription<audio_common_msgs::msg::AudioData>(
            audio_topic_,
            10,
            std::bind(&AudioDopplerHomingNode::audio_callback, this, std::placeholders::_1));

        worker_thread_ = std::thread(&AudioDopplerHomingNode::analysis_loop, this);
    }

    ~AudioDopplerHomingNode()
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
    struct FrequencyEstimate
    {
        bool valid = false;
        double frequency_hz = 0.0;
        double snr_ratio = 0.0;
        double magnitude = 0.0;
    };

    struct CycleEstimate
    {
        rclcpp::Time stamp;
        double frequency_hz = 0.0;
        double snr_ratio = 0.0;
    };

    void audio_callback(const audio_common_msgs::msg::AudioData::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        append_samples_from_pcm(msg->data);
        buffer_cv_.notify_one();
    }

    void analysis_loop()
    {
        while (rclcpp::ok()) {
            std::vector<double> snapshot;
            {
                std::unique_lock<std::mutex> lock(buffer_mutex_);
                buffer_cv_.wait(lock, [this]() {
                    return stop_worker_ || sample_buffer_.size() >= snapshot_size_;
                });

                if (stop_worker_) {
                    break;
                }

                snapshot.assign(sample_buffer_.begin(), sample_buffer_.begin() + snapshot_size_);
                sample_buffer_.erase(sample_buffer_.begin(), sample_buffer_.begin() + hop_size_);
            }

            process_snapshot(snapshot);
        }
    }

    void process_snapshot(const std::vector<double> & snapshot)
    {
        const FrequencyEstimate estimate = estimate_frequency(snapshot);
        if (estimate.valid) {
            update_tracked_frequency(estimate.frequency_hz);
        }
        update_control_cycle(estimate);
    }

    FrequencyEstimate estimate_frequency(const std::vector<double> & snapshot) const
    {
        FrequencyEstimate estimate;
        const double coarse_step_hz = std::max(coarse_frequency_step_hz_, 0.1);
        const double search_center_hz = have_tracked_frequency_ ?
            tracked_frequency_hz_ :
            reference_frequency_hz_;
        const double search_half_width_hz = have_tracked_frequency_ ?
            tracking_search_half_width_hz_ :
            frequency_search_half_width_hz_;
        const double search_start_hz =
            std::max(1.0, search_center_hz - std::max(search_half_width_hz, 0.0));
        const double search_end_hz =
            search_center_hz + std::max(search_half_width_hz, 0.0);

        double best_frequency_hz = reference_frequency_hz_;
        double best_magnitude = -1.0;
        for (double frequency_hz = search_start_hz;
            frequency_hz <= search_end_hz + 0.5 * coarse_step_hz;
            frequency_hz += coarse_step_hz)
        {
            const double magnitude = demodulate_magnitude(snapshot, frequency_hz);
            if (magnitude > best_magnitude) {
                best_magnitude = magnitude;
                best_frequency_hz = frequency_hz;
            }
        }

        const double fine_step_hz = std::max(fine_frequency_step_hz_, 0.1);
        const double fine_start_hz = std::max(
            search_start_hz,
            best_frequency_hz - std::max(fine_search_half_width_hz_, 0.0));
        const double fine_end_hz = std::min(
            search_end_hz,
            best_frequency_hz + std::max(fine_search_half_width_hz_, 0.0));
        for (double frequency_hz = fine_start_hz;
            frequency_hz <= fine_end_hz + 0.5 * fine_step_hz;
            frequency_hz += fine_step_hz)
        {
            const double magnitude = demodulate_magnitude(snapshot, frequency_hz);
            if (magnitude > best_magnitude) {
                best_magnitude = magnitude;
                best_frequency_hz = frequency_hz;
            }
        }

        const double noise_magnitude = estimate_noise_magnitude(snapshot, best_frequency_hz);
        estimate.frequency_hz = best_frequency_hz;
        estimate.magnitude = best_magnitude;
        estimate.snr_ratio = best_magnitude / std::max(noise_magnitude, 1.0e-12);
        estimate.valid = estimate.snr_ratio >= snr_threshold_;
        return estimate;
    }

    void update_tracked_frequency(const double frequency_hz)
    {
        const double alpha = std::clamp(frequency_tracking_alpha_, 0.0, 1.0);
        if (!have_tracked_frequency_) {
            tracked_frequency_hz_ = frequency_hz;
            have_tracked_frequency_ = true;
            return;
        }
        tracked_frequency_hz_ = (1.0 - alpha) * tracked_frequency_hz_ + alpha * frequency_hz;
    }

    double demodulate_magnitude(const std::vector<double> & snapshot, const double frequency_hz) const
    {
        std::complex<double> sum(0.0, 0.0);
        double weight_sum = 0.0;
        const double phase_step = 2.0 * M_PI * frequency_hz / static_cast<double>(sampling_rate_);
        const double hann_denominator = static_cast<double>(std::max<std::size_t>(1, snapshot.size() - 1));

        for (std::size_t n = 0; n < snapshot.size(); ++n) {
            const double weight =
                0.5 * (1.0 - std::cos((2.0 * M_PI * static_cast<double>(n)) / hann_denominator));
            const double phase = phase_step * static_cast<double>(n);
            sum += weight * snapshot[n] * std::complex<double>(std::cos(phase), -std::sin(phase));
            weight_sum += weight;
        }

        return std::abs(sum) / std::max(weight_sum, 1.0e-12);
    }

    double estimate_noise_magnitude(const std::vector<double> & snapshot, const double center_frequency_hz) const
    {
        std::vector<double> magnitudes;
        magnitudes.reserve(6);
        const double offsets_hz[] = {-700.0, -450.0, -250.0, 250.0, 450.0, 700.0};
        for (const double offset_hz : offsets_hz) {
            const double probe_frequency_hz = center_frequency_hz + offset_hz;
            if (probe_frequency_hz <= 1.0) {
                continue;
            }
            magnitudes.push_back(demodulate_magnitude(snapshot, probe_frequency_hz));
        }

        if (magnitudes.empty()) {
            return 1.0e-12;
        }
        std::sort(magnitudes.begin(), magnitudes.end());
        return magnitudes[magnitudes.size() / 2];
    }

    void update_control_cycle(const FrequencyEstimate & estimate)
    {
        const rclcpp::Time now = this->now();
        if (!cycle_started_) {
            cycle_started_ = true;
            cycle_start_time_ = now;
        }

        ++cycle_snapshot_count_;
        if (estimate.valid) {
            cycle_frequencies_hz_.push_back(estimate.frequency_hz);
            cycle_snr_ratios_.push_back(estimate.snr_ratio);
        }

        if ((now - cycle_start_time_).seconds() < control_period_s_) {
            return;
        }

        finish_control_cycle(now);
        reset_control_cycle(now);
    }

    void finish_control_cycle(const rclcpp::Time & now)
    {
        const double detection_ratio = cycle_snapshot_count_ > 0 ?
            static_cast<double>(cycle_frequencies_hz_.size()) / static_cast<double>(cycle_snapshot_count_) :
            0.0;
        const bool detected =
            !cycle_frequencies_hz_.empty() && detection_ratio >= detection_ratio_threshold_;

        double mean_frequency_hz = baseline_frequency_hz_;
        double mean_snr_ratio = 0.0;
        double signed_trend_hzps = 0.0;
        double doppler_shift_hz = 0.0;
        if (detected) {
            mean_frequency_hz = median(cycle_frequencies_hz_);
            mean_snr_ratio = mean(cycle_snr_ratios_);
            if (!have_baseline_frequency_) {
                update_baseline_frequency(mean_frequency_hz);
                RCLCPP_INFO(
                    this->get_logger(),
                    "doppler: calibrating f0, valid %zu/%d, f %.2f Hz, samples %zu/%d, snr %.2f",
                    cycle_frequencies_hz_.size(),
                    cycle_snapshot_count_,
                    mean_frequency_hz,
                    baseline_frequency_candidates_hz_.size(),
                    baseline_cycles_,
                    mean_snr_ratio);
                return;
            }

            update_frequency_history(now, mean_frequency_hz, mean_snr_ratio);
            doppler_shift_hz = doppler_sign_ * (mean_frequency_hz - baseline_frequency_hz_);
            signed_trend_hzps = doppler_sign_ * estimate_frequency_trend_hzps();
            const bool trend_bad =
                have_frequency_trend_ &&
                signed_trend_hzps < bad_frequency_rate_hzps_ &&
                std::abs(signed_trend_hzps) > trend_deadband_hzps_;
            if (trend_bad && !was_trend_bad_) {
                turn_sign_ *= -1.0;
            }
            was_trend_bad_ = trend_bad;
        } else {
            was_trend_bad_ = false;
        }

        const double turn_command = detected ?
            turn_sign_ * std::clamp(turn_command_magnitude_, 0.0, 1.0) :
            turn_sign_ * std::clamp(search_turn_command_, 0.0, 1.0);

        RCLCPP_INFO(
            this->get_logger(),
            "doppler: det %d, valid %zu/%d, f %.2f Hz, f0 %.2f Hz, shift %.2f Hz, snr %.2f, trend %.2f Hz/s, turn %.2f",
            detected ? 1 : 0,
            cycle_frequencies_hz_.size(),
            cycle_snapshot_count_,
            mean_frequency_hz,
            baseline_frequency_hz_,
            doppler_shift_hz,
            mean_snr_ratio,
            signed_trend_hzps,
            turn_command);
    }

    void update_frequency_history(
        const rclcpp::Time & stamp,
        const double frequency_hz,
        const double snr_ratio)
    {
        frequency_history_.push_back({stamp, frequency_hz, snr_ratio});
        while (static_cast<int>(frequency_history_.size()) > trend_history_cycles_) {
            frequency_history_.erase(frequency_history_.begin());
        }
    }

    void update_baseline_frequency(const double frequency_hz)
    {
        if (have_baseline_frequency_) {
            return;
        }

        baseline_frequency_candidates_hz_.push_back(frequency_hz);
        if (static_cast<int>(baseline_frequency_candidates_hz_.size()) >= baseline_cycles_) {
            baseline_frequency_hz_ = median(baseline_frequency_candidates_hz_);
            have_baseline_frequency_ = true;
            RCLCPP_WARN(
                this->get_logger(),
                "Doppler baseline locked: f0 %.2f Hz.",
                baseline_frequency_hz_);
        }
    }

    double estimate_frequency_trend_hzps()
    {
        have_frequency_trend_ = false;
        if (frequency_history_.size() < 2) {
            return 0.0;
        }

        const std::size_t half_count = frequency_history_.size() / 2;
        if (half_count == 0) {
            return 0.0;
        }

        std::vector<double> first_frequencies;
        std::vector<double> second_frequencies;
        first_frequencies.reserve(half_count);
        second_frequencies.reserve(frequency_history_.size() - half_count);
        for (std::size_t i = 0; i < frequency_history_.size(); ++i) {
            if (i < half_count) {
                first_frequencies.push_back(frequency_history_[i].frequency_hz);
            } else {
                second_frequencies.push_back(frequency_history_[i].frequency_hz);
            }
        }

        const double first_frequency_hz = median(first_frequencies);
        const double second_frequency_hz = median(second_frequencies);
        const double dt = std::max(
            (frequency_history_.back().stamp - frequency_history_.front().stamp).seconds(),
            1.0e-6);
        have_frequency_trend_ = true;
        return (second_frequency_hz - first_frequency_hz) / dt;
    }

    void reset_control_cycle(const rclcpp::Time & now)
    {
        cycle_start_time_ = now;
        cycle_snapshot_count_ = 0;
        cycle_frequencies_hz_.clear();
        cycle_snr_ratios_.clear();
    }

    double mean(const std::vector<double> & values) const
    {
        if (values.empty()) {
            return 0.0;
        }
        return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
    }

    double median(std::vector<double> values) const
    {
        if (values.empty()) {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        const std::size_t middle = values.size() / 2;
        if (values.size() % 2 == 1) {
            return values[middle];
        }
        return 0.5 * (values[middle - 1] + values[middle]);
    }

    void append_samples_from_pcm(const std::vector<uint8_t> & data)
    {
        const std::size_t channel_offset = channel_index_ * bytes_per_sample_;
        for (std::size_t frame_start = 0; frame_start + frame_size_ <= data.size();
            frame_start += frame_size_)
        {
            const int32_t sample = read_int32_little_endian(data, frame_start + channel_offset);
            sample_buffer_.push_back(static_cast<double>(sample) / 2147483648.0);
        }
    }

    int32_t read_int32_little_endian(const std::vector<uint8_t> & data, const std::size_t offset) const
    {
        const uint32_t raw =
            static_cast<uint32_t>(data[offset]) |
            (static_cast<uint32_t>(data[offset + 1]) << 8) |
            (static_cast<uint32_t>(data[offset + 2]) << 16) |
            (static_cast<uint32_t>(data[offset + 3]) << 24);
        return static_cast<int32_t>(raw);
    }

    std::size_t seconds_to_samples(const double seconds) const
    {
        return static_cast<std::size_t>(
            std::max(1.0, std::round(std::max(seconds, 0.0) * static_cast<double>(sampling_rate_))));
    }

    std::string audio_topic_ = "/audio";
    rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;

    std::vector<double> sample_buffer_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    std::thread worker_thread_;
    bool stop_worker_ = false;

    std::size_t sampling_rate_ = 96000;
    std::size_t channels_ = 2;
    std::size_t channel_index_ = 0;
    std::size_t bytes_per_sample_ = 4;
    std::size_t frame_size_ = channels_ * bytes_per_sample_;

    double reference_frequency_hz_ = 21164.0;
    double frequency_search_half_width_hz_ = 80.0;
    double tracking_search_half_width_hz_ = 6.0;
    double frequency_tracking_alpha_ = 0.1;
    double coarse_frequency_step_hz_ = 5.0;
    double fine_frequency_step_hz_ = 0.5;
    double fine_search_half_width_hz_ = 5.0;
    double snapshot_duration_s_ = 0.5;
    double snapshot_hop_s_ = 0.1;
    double control_period_s_ = 1.0;
    double snr_threshold_ = 1.0;
    double detection_ratio_threshold_ = 0.5;
    double bad_frequency_rate_hzps_ = -3.0;
    double trend_deadband_hzps_ = 1.5;
    int trend_history_cycles_ = 7;
    int baseline_cycles_ = 3;
    double doppler_sign_ = 1.0;
    double turn_command_magnitude_ = 0.35;
    double search_turn_command_ = 0.25;
    std::size_t snapshot_size_ = 48000;
    std::size_t hop_size_ = 9600;

    bool cycle_started_ = false;
    rclcpp::Time cycle_start_time_;
    int cycle_snapshot_count_ = 0;
    std::vector<double> cycle_frequencies_hz_;
    std::vector<double> cycle_snr_ratios_;

    std::vector<CycleEstimate> frequency_history_;
    std::vector<double> baseline_frequency_candidates_hz_;
    bool have_tracked_frequency_ = false;
    double tracked_frequency_hz_ = 21164.0;
    double turn_sign_ = 1.0;
    bool was_trend_bad_ = false;
    bool have_baseline_frequency_ = false;
    double baseline_frequency_hz_ = 21164.0;
    bool have_frequency_trend_ = false;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::AudioDopplerHomingNode)
