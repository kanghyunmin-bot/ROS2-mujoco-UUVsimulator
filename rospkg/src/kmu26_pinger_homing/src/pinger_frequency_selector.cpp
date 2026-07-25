#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <iostream>
#include <map>
#include <numeric>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <unsupported/Eigen/FFT>

namespace auv_pinger_homing {

// This is a selector, not the Phase estimator.  It deliberately shares the
// proven FFT detection contract from kmu26_auv_hydrophone: DC removal, Hann
// windowing, a band-limited median noise floor, an SNR gate, and repeated
// windows.  The old selector evaluated only 100-Hz Goertzel-like probes and
// refined a candidate from the final PCM frame, which made a short noise burst
// look just as credible as a pinger.
class FingerFrequencySelector final : public rclcpp::Node {
 public:
  FingerFrequencySelector() : Node("pinger_frequency_selector") {
    audio_topic_ = declare_parameter<std::string>("audio_topic", "/audio");
    selected_topic_ = declare_parameter<std::string>(
        "selected_frequency_topic", "/pinger_homing/selected_frequency_hz");
    candidate_topic_ = declare_parameter<std::string>(
        "candidate_topic", "/pinger_homing/frequency_candidates");
    manual_selection_topic_ = declare_parameter<std::string>(
        "manual_selection_topic", "/pinger_homing/manual_selection");
    sample_rate_ = std::max(
        8000, static_cast<int>(declare_parameter<int>("sample_rate", 96000)));
    channels_ = std::max(1, static_cast<int>(declare_parameter<int>("channels", 2)));
    channel_index_ = std::clamp(
        static_cast<int>(declare_parameter<int>("channel_index", 0)), 0, channels_ - 1);
    combine_channels_ = declare_parameter<bool>("combine_channels", true);
    monitor_s_ = std::clamp(declare_parameter<double>("monitor_s", 10.0), 1.0, 30.0);
    min_frequency_ = declare_parameter<double>("min_frequency_hz", 19000.0);
    max_frequency_ = declare_parameter<double>("max_frequency_hz", 22000.0);
    if (max_frequency_ <= min_frequency_) {
      throw std::runtime_error("max_frequency_hz must be greater than min_frequency_hz");
    }

    // window_size/hop_size remain supported for old launch files.  New names
    // make their FFT role explicit and default to 11.72-Hz bins at 96 kHz.
    const int legacy_window_size = static_cast<int>(declare_parameter<int>("window_size", 0));
    const int legacy_hop_size = static_cast<int>(declare_parameter<int>("hop_size", 0));
    const int fft_default = legacy_window_size > 0 ? legacy_window_size : 8192;
    fft_size_ = std::clamp(
        static_cast<int>(declare_parameter<int>("fft_size", fft_default)), 1024, 32768);
    const int hop_default = legacy_hop_size > 0 ? legacy_hop_size : fft_size_ / 2;
    fft_hop_size_ = std::clamp(
        static_cast<int>(declare_parameter<int>("fft_hop_size", hop_default)), 256, fft_size_);

    min_snr_db_ = std::clamp(declare_parameter<double>("min_snr_db", 9.0), 0.0, 80.0);
    min_peak_prominence_db_ = std::clamp(
        declare_parameter<double>("min_peak_prominence_db", 4.5), 0.0, 80.0);
    minimum_candidate_hits_ = std::max(
        1, static_cast<int>(declare_parameter<int>("minimum_candidate_hits", 4)));
    tracking_min_snr_db_ = std::clamp(
        declare_parameter<double>("tracking_min_snr_db", 0.5), 0.0, 30.0);
    tracking_min_prominence_db_ = std::clamp(
        declare_parameter<double>("tracking_min_prominence_db", 0.25), 0.0, 30.0);
    persistent_min_ratio_ = std::clamp(
        declare_parameter<double>("persistent_min_ratio", 0.30), 0.01, 1.0);
    persistent_min_snr_db_ = std::clamp(
        declare_parameter<double>("persistent_min_snr_db", 1.0), 0.0, 30.0);
    persistent_min_prominence_db_ = std::clamp(
        declare_parameter<double>("persistent_min_prominence_db", 0.5), 0.0, 30.0);
    max_tracked_peaks_per_window_ = std::clamp(
        static_cast<int>(declare_parameter<int>("max_tracked_peaks_per_window", 12)),
        1, 50);
    candidate_cluster_hz_ = std::clamp(
        declare_parameter<double>("candidate_cluster_hz", 25.0), 1.0, 500.0);
    candidate_separation_hz_ = std::clamp(
        declare_parameter<double>("candidate_separation_hz", 75.0), 1.0, 2000.0);
    relative_to_top_snr_db_ = std::clamp(
        declare_parameter<double>("relative_to_top_snr_db", 18.0), 0.0, 80.0);
    max_candidates_ = std::clamp(
        static_cast<int>(declare_parameter<int>("max_candidates", 5)), 1, 10);
    blacklist_frequency_hz_ = declare_parameter<double>("blacklist_frequency_hz", 0.0);
    blacklist_half_width_hz_ = std::max(
        0.0, declare_parameter<double>("blacklist_half_width_hz", 0.0));
    auto_select_top_ = declare_parameter<bool>("auto_select_top", false);
    stdin_selection_enabled_ = declare_parameter<bool>("stdin_selection_enabled", true);

    // A choice is volatile by design. A later test-tank launch must not obtain
    // an old DDS selection while its Phase accumulator is starting afresh.
    selected_pub_ = create_publisher<std_msgs::msg::Float64>(selected_topic_, rclcpp::QoS(1));
    candidate_pub_ = create_publisher<std_msgs::msg::String>(
        candidate_topic_, rclcpp::QoS(1).transient_local());
    audio_sub_ = create_subscription<audio_common_msgs::msg::AudioData>(
        audio_topic_, rclcpp::SensorDataQoS(),
        [this](const audio_common_msgs::msg::AudioData::SharedPtr msg) { on_audio(*msg); });
    manual_selection_sub_ = create_subscription<std_msgs::msg::String>(
        manual_selection_topic_, 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { select_from_text(msg->data); });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { poll_selection(); });
    channel_samples_.resize(static_cast<std::size_t>(channels_));
    started_ = Clock::now();
    RCLCPP_INFO(
        get_logger(),
        "FFT scan %.0f--%.0f Hz for %.1fs: N=%d, bin=%.3f Hz, hop=%d, "
        "channels=%d mode=%s min SNR=%.1f dB",
        min_frequency_, max_frequency_, monitor_s_, fft_size_, frequency_resolution_hz(),
        fft_hop_size_, channels_, combine_channels_ ? "normalized-power-sum" : "single",
        min_snr_db_);
  }

 private:
  using Clock = std::chrono::steady_clock;

  struct WindowPeak {
    double frequency{0.0};
    double power{0.0};
    double snr_db{0.0};
    double prominence_db{0.0};
  };

  struct AccumulatedCandidate {
    double weighted_frequency_sum{0.0};
    double weight_sum{0.0};
    double snr_sum{0.0};
    int hits{0};
  };

  struct Candidate {
    double frequency{0.0};
    double score{0.0};  // average-spectrum SNR in dB
    double prominence_db{0.0};
    int hits{0};
    double persistence_ratio{0.0};
    bool qualified{false};
  };

  static int32_t read_s32(const std::vector<uint8_t> &data, std::size_t i) {
    const uint32_t value = static_cast<uint32_t>(data[i]) |
        (static_cast<uint32_t>(data[i + 1]) << 8) |
        (static_cast<uint32_t>(data[i + 2]) << 16) |
        (static_cast<uint32_t>(data[i + 3]) << 24);
    return static_cast<int32_t>(value);
  }

  static double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

  double frequency_resolution_hz() const {
    return static_cast<double>(sample_rate_) / static_cast<double>(fft_size_);
  }

  bool blacklisted(double frequency) const {
    return blacklist_half_width_hz_ > 0.0 &&
        std::abs(frequency - blacklist_frequency_hz_) <= blacklist_half_width_hz_;
  }

  std::pair<std::size_t, std::size_t> band_limits(const std::size_t nyquist_bin) const {
    const double resolution = frequency_resolution_hz();
    const auto min_bin = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(min_frequency_ / resolution)));
    const auto max_bin = std::min<std::size_t>(
        nyquist_bin - 1U, static_cast<std::size_t>(std::floor(max_frequency_ / resolution)));
    return {min_bin, max_bin};
  }

  WindowPeak make_peak(
      const std::vector<double> &power, std::size_t bin, std::size_t min_bin,
      std::size_t max_bin, double noise_floor) const {
    const double center = power[bin];
    const double left = power[bin - 1U];
    const double right = power[bin + 1U];
    const double denominator = left - 2.0 * center + right;
    double fractional_bin = 0.0;
    if (std::abs(denominator) > 1.0e-30) {
      fractional_bin = std::clamp(0.5 * (left - right) / denominator, -0.5, 0.5);
    }

    // The surrounding median deliberately skips the centre bin.  It rejects
    // a broad tonal shoulder that is high relative to the band but not a
    // distinct spectral peak.
    std::vector<double> local;
    constexpr std::size_t kLocalRadius = 8U;
    const auto local_low = std::max(min_bin, bin > kLocalRadius ? bin - kLocalRadius : min_bin);
    const auto local_high = std::min(max_bin, bin + kLocalRadius);
    local.reserve(local_high - local_low);
    for (std::size_t index = local_low; index <= local_high; ++index) {
      if (index != bin) local.push_back(power[index]);
    }
    const double local_floor = std::max(1.0e-24, median(std::move(local)));
    constexpr double kEpsilon = 1.0e-24;
    return WindowPeak{
        (static_cast<double>(bin) + fractional_bin) * frequency_resolution_hz(),
        center,
        10.0 * std::log10((center + kEpsilon) / (noise_floor + kEpsilon)),
        10.0 * std::log10((center + kEpsilon) / (local_floor + kEpsilon))};
  }

  std::vector<WindowPeak> spectral_peaks(
      const std::vector<double> &power, bool require_quality) const {
    if (power.size() < 3U) return {};
    const auto [min_bin, max_bin] = band_limits(power.size() - 1U);
    if (min_bin + 2U >= max_bin) return {};

    std::vector<double> band_power;
    band_power.reserve(max_bin - min_bin + 1U);
    for (std::size_t bin = min_bin; bin <= max_bin; ++bin) {
      band_power.push_back(power[bin]);
    }
    const double noise_floor = std::max(1.0e-24, median(std::move(band_power)));
    std::vector<WindowPeak> found;
    for (std::size_t bin = min_bin + 1U; bin < max_bin; ++bin) {
      if (power[bin] < power[bin - 1U] || power[bin] < power[bin + 1U]) continue;
      const auto peak = make_peak(power, bin, min_bin, max_bin, noise_floor);
      if (blacklisted(peak.frequency)) continue;
      if (require_quality &&
          (peak.snr_db < min_snr_db_ || peak.prominence_db < min_peak_prominence_db_)) {
        continue;
      }
      found.push_back(peak);
    }
    std::sort(found.begin(), found.end(), [](const WindowPeak &a, const WindowPeak &b) {
      return a.snr_db > b.snr_db;
    });
    std::vector<WindowPeak> separated;
    separated.reserve(found.size());
    for (const auto &peak : found) {
      const bool close_to_better_peak = std::any_of(
          separated.begin(), separated.end(), [this, &peak](const WindowPeak &accepted) {
            return std::abs(accepted.frequency - peak.frequency) < candidate_separation_hz_;
          });
      if (!close_to_better_peak) separated.push_back(peak);
    }
    return separated;
  }

  void on_audio(const audio_common_msgs::msg::AudioData &msg) {
    const std::size_t frame_bytes = static_cast<std::size_t>(channels_) * 4U;
    for (std::size_t frame = 0; frame + frame_bytes <= msg.data.size(); frame += frame_bytes) {
      for (int channel = 0; channel < channels_; ++channel) {
        const std::size_t offset = frame + static_cast<std::size_t>(channel) * 4U;
        channel_samples_[static_cast<std::size_t>(channel)].push_back(
            static_cast<double>(read_s32(msg.data, offset)) / 2147483648.0);
      }
    }
    while (!monitor_finished_ && channels_have_window()) {
      analyze_channels();
      for (auto &samples : channel_samples_) {
        const std::size_t remove = std::min<std::size_t>(fft_hop_size_, samples.size());
        samples.erase(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(remove));
      }
    }
  }

  bool channels_have_window() const {
    if (channel_samples_.empty()) return false;
    return std::all_of(
        channel_samples_.begin(), channel_samples_.end(), [this](const auto &samples) {
          return samples.size() >= static_cast<std::size_t>(fft_size_);
        });
  }

  std::vector<double> channel_power(std::vector<double> window) {
    const double mean = std::accumulate(window.begin(), window.end(), 0.0) /
        static_cast<double>(window.size());
    const double denominator = static_cast<double>(std::max(1, fft_size_ - 1));
    for (std::size_t index = 0; index < window.size(); ++index) {
      const double hann = 0.5 * (1.0 - std::cos(2.0 * M_PI * static_cast<double>(index) / denominator));
      window[index] = (window[index] - mean) * hann;
    }

    std::vector<std::complex<double>> bins;
    fft_.fwd(bins, window);
    const std::size_t nyquist_bin = bins.size() / 2U;
    std::vector<double> power(nyquist_bin + 1U, 0.0);
    for (std::size_t bin = 0; bin <= nyquist_bin; ++bin) {
      power[bin] = std::norm(bins[bin]);
    }
    return power;
  }

  void analyze_channels() {
    std::vector<double> combined;
    const int first_channel = combine_channels_ ? 0 : channel_index_;
    const int final_channel = combine_channels_ ? channels_ : channel_index_ + 1;
    for (int channel = first_channel; channel < final_channel; ++channel) {
      const auto &samples = channel_samples_[static_cast<std::size_t>(channel)];
      std::vector<double> window(
          samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(fft_size_));
      std::vector<double> power = channel_power(std::move(window));
      if (combined.empty()) combined.assign(power.size(), 0.0);

      // Hydrophone channel gains can be very different and their carrier
      // phases can oppose.  Normalize each channel by its own in-band median
      // noise floor, then add powers (never PCM amplitudes), so a valid tone
      // on channel 1 is not hidden by channel 0 or cancelled by phase.
      const auto [min_bin, max_bin] = band_limits(power.size() - 1U);
      std::vector<double> band;
      band.reserve(max_bin - min_bin + 1U);
      for (std::size_t bin = min_bin; bin <= max_bin; ++bin) band.push_back(power[bin]);
      const double channel_floor = std::max(1.0e-24, median(std::move(band)));
      for (std::size_t bin = 0; bin < power.size(); ++bin) {
        combined[bin] += power[bin] / channel_floor;
      }
    }
    if (average_power_.empty()) average_power_.assign(combined.size(), 0.0);
    for (std::size_t bin = 0; bin < combined.size(); ++bin) {
      average_power_[bin] += combined[bin];
    }
    ++fft_windows_;

    // Track relaxed per-window peaks separately from final qualification.
    // A submerged pinger can be only 1--3 dB above a coloured physical noise
    // floor, but a real carrier remains in the same frequency cluster for a
    // large fraction of the ten-second scan. Random noise does not.
    int tracked = 0;
    for (const auto &peak : spectral_peaks(combined, false)) {
      if (tracked >= max_tracked_peaks_per_window_) break;
      if (peak.snr_db < tracking_min_snr_db_ ||
          peak.prominence_db < tracking_min_prominence_db_) {
        continue;
      }
      const double key = std::round(peak.frequency / candidate_cluster_hz_) * candidate_cluster_hz_;
      auto &candidate = repeated_candidates_[key];
      const double weight = std::max(1.0, peak.snr_db);
      candidate.weighted_frequency_sum += peak.frequency * weight;
      candidate.weight_sum += weight;
      candidate.snr_sum += peak.snr_db;
      ++candidate.hits;
      ++tracked;
    }
  }

  int repeated_hits_near(double frequency) const {
    int hits = 0;
    for (const auto &[key, candidate] : repeated_candidates_) {
      (void)key;
      if (candidate.weight_sum > 0.0 &&
          std::abs(candidate.weighted_frequency_sum / candidate.weight_sum - frequency) <=
              candidate_cluster_hz_) {
        hits += candidate.hits;
      }
    }
    return hits;
  }

  std::vector<Candidate> ranked(bool require_quality) const {
    if (fft_windows_ == 0U || average_power_.empty()) return {};
    std::vector<double> averaged = average_power_;
    for (double &power : averaged) power /= static_cast<double>(fft_windows_);

    std::vector<Candidate> values;
    // Always enumerate averaged peaks with relaxed spectral filtering here;
    // qualification below can be earned either by the strong instantaneous
    // thresholds or by ten-second frequency persistence.
    for (const auto &peak : spectral_peaks(averaged, false)) {
      const int hits = repeated_hits_near(peak.frequency);
      const double persistence_ratio = fft_windows_ > 0U
          ? std::min(1.0, static_cast<double>(hits) / static_cast<double>(fft_windows_))
          : 0.0;
      const bool strong_quality = peak.snr_db >= min_snr_db_ &&
          peak.prominence_db >= min_peak_prominence_db_ &&
          hits >= minimum_candidate_hits_;
      const bool persistent_quality = peak.snr_db >= persistent_min_snr_db_ &&
          peak.prominence_db >= persistent_min_prominence_db_ &&
          hits >= minimum_candidate_hits_ && persistence_ratio >= persistent_min_ratio_;
      const bool qualified = strong_quality || persistent_quality;
      if (require_quality && !qualified) continue;
      values.push_back(Candidate{
          peak.frequency, peak.snr_db, peak.prominence_db, hits,
          persistence_ratio, qualified});
    }
    std::sort(values.begin(), values.end(), [](const Candidate &a, const Candidate &b) {
      if (a.qualified != b.qualified) return a.qualified;
      if (a.hits != b.hits) return a.hits > b.hits;
      return a.score > b.score;
    });
    // A persistent simulator/vehicle tone can meet an absolute SNR floor
    // even though it is far below the selected pinger. Keep genuinely close
    // competing transmitters but hide such residual peaks from the operator.
    if (require_quality && relative_to_top_snr_db_ > 0.0 && !values.empty()) {
      const double minimum_relative_score = values.front().score - relative_to_top_snr_db_;
      values.erase(
          std::remove_if(values.begin(), values.end(), [minimum_relative_score](const Candidate &candidate) {
            return candidate.score < minimum_relative_score;
          }),
          values.end());
    }
    if (values.size() > static_cast<std::size_t>(max_candidates_)) {
      values.resize(static_cast<std::size_t>(max_candidates_));
    }
    return values;
  }

  void finish_monitor() {
    if (monitor_finished_) return;
    monitor_finished_ = true;
    final_candidates_ = ranked(true);
    if (final_candidates_.empty()) {
      final_candidates_ = ranked(false);
      RCLCPP_WARN(
          get_logger(),
          "no FFT peak met strong thresholds or persistent ratio %.0f%%; "
          "showing unqualified spectrum peaks for manual diagnosis",
          100.0 * persistent_min_ratio_);
    }

    std::ostringstream json;
    json << "{\"ready\":true,\"fft_windows\":" << fft_windows_
         << ",\"frequency_resolution_hz\":" << frequency_resolution_hz()
         << ",\"candidates\":[";
    for (std::size_t index = 0; index < final_candidates_.size(); ++index) {
      if (index) json << ',';
      const auto &candidate = final_candidates_[index];
      json << "{\"frequency_hz\":" << candidate.frequency
           << ",\"hits\":" << candidate.hits
           << ",\"score\":" << candidate.score
           << ",\"snr_db\":" << candidate.score
           << ",\"prominence_db\":" << candidate.prominence_db
           << ",\"persistence_ratio\":" << candidate.persistence_ratio
           << ",\"qualified\":" << (candidate.qualified ? "true" : "false") << '}';
    }
    json << "]}";
    candidate_pub_->publish(std_msgs::msg::String().set__data(json.str()));

    RCLCPP_INFO(
        get_logger(), "FFT candidates after %zu windows (%.3f Hz/bin, within %.1f dB of strongest):",
        fft_windows_, frequency_resolution_hz(), relative_to_top_snr_db_);
    for (std::size_t index = 0; index < final_candidates_.size(); ++index) {
      const auto &candidate = final_candidates_[index];
      RCLCPP_INFO(
          get_logger(), "  [%zu] %.2f Hz (hits=%d, persistence=%.0f%%, "
          "SNR=%.1f dB, prominence=%.1f dB, %s)",
          index + 1U, candidate.frequency, candidate.hits,
          100.0 * candidate.persistence_ratio, candidate.score, candidate.prominence_db,
          candidate.qualified ? "qualified" : "manual-only");
    }
    if (!auto_select_top_ && stdin_selection_enabled_) {
      RCLCPP_INFO(
          get_logger(), "Enter candidate 1-%zu or an exact frequency in Hz in this terminal.",
          final_candidates_.size());
    }
    if (auto_select_top_ && !final_candidates_.empty() && final_candidates_.front().qualified) {
      select(final_candidates_.front().frequency);
    } else if (auto_select_top_ && !final_candidates_.empty()) {
      RCLCPP_WARN(get_logger(), "auto-select withheld because the strongest FFT peak is not qualified");
    }
  }

  void select(double frequency) {
    if (selected_) return;
    if (!std::isfinite(frequency) || frequency < min_frequency_ || frequency > max_frequency_) {
      RCLCPP_WARN(get_logger(), "selection %.2f Hz is outside monitored band", frequency);
      return;
    }
    selected_pub_->publish(std_msgs::msg::Float64().set__data(frequency));
    selected_ = true;
    RCLCPP_INFO(get_logger(), "frequency %.2f Hz selected; homing controller may start", frequency);
  }

  void select_from_text(const std::string &text) {
    if (!monitor_finished_ || selected_) return;
    try {
      const double value = std::stod(text);
      if (value >= 1.0 && value <= static_cast<double>(final_candidates_.size()) &&
          std::floor(value) == value) {
        select(final_candidates_[static_cast<std::size_t>(value) - 1U].frequency);
      } else {
        select(value);
      }
    } catch (...) {
      RCLCPP_WARN(get_logger(), "enter candidate number 1-%zu or frequency in Hz", final_candidates_.size());
    }
  }

  void poll_selection() {
    if (!monitor_finished_ &&
        std::chrono::duration<double>(Clock::now() - started_).count() >= monitor_s_) {
      finish_monitor();
    }
    if (!monitor_finished_ || selected_ || auto_select_top_ || !stdin_selection_enabled_) return;
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};
    if (::poll(&descriptor, 1, 0) <= 0) return;
    std::string line;
    std::getline(std::cin, line);
    if (!line.empty()) select_from_text(line);
  }

  std::string audio_topic_, selected_topic_, candidate_topic_, manual_selection_topic_;
  int sample_rate_{96000}, channels_{2}, channel_index_{0}, fft_size_{8192}, fft_hop_size_{4096};
  bool combine_channels_{true};
  double monitor_s_{10.0}, min_frequency_{19000.0}, max_frequency_{22000.0};
  double min_snr_db_{9.0}, min_peak_prominence_db_{4.5};
  int minimum_candidate_hits_{4}, max_candidates_{5};
  double tracking_min_snr_db_{0.5}, tracking_min_prominence_db_{0.25};
  double persistent_min_ratio_{0.30}, persistent_min_snr_db_{1.0};
  double persistent_min_prominence_db_{0.5};
  int max_tracked_peaks_per_window_{12};
  double candidate_cluster_hz_{25.0}, candidate_separation_hz_{75.0};
  double relative_to_top_snr_db_{18.0};
  double blacklist_frequency_hz_{0.0}, blacklist_half_width_hz_{0.0};
  bool auto_select_top_{false}, stdin_selection_enabled_{true}, monitor_finished_{false}, selected_{false};
  std::vector<std::deque<double>> channel_samples_;
  std::vector<double> average_power_;
  std::map<double, AccumulatedCandidate> repeated_candidates_;
  std::vector<Candidate> final_candidates_;
  std::size_t fft_windows_{0U};
  Eigen::FFT<double> fft_;
  Clock::time_point started_;
  rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_selection_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr selected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace auv_pinger_homing

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_pinger_homing::FingerFrequencySelector>());
  rclcpp::shutdown();
  return 0;
}
