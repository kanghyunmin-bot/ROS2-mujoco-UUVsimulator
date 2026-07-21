#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace kmu26_pinger_homing {

class FingerAudioEstimator final : public rclcpp::Node {
 public:
  FingerAudioEstimator() : Node("pinger_audio_estimator") {
    audio_topic_ = declare_parameter<std::string>("audio_topic", "/audio");
    selected_topic_ = declare_parameter<std::string>(
        "selected_frequency_topic", "/pinger_homing/selected_frequency_hz");
    sample_rate_ = declare_parameter<int>("sample_rate", 96000);
    channels_ = std::max(1, static_cast<int>(declare_parameter<int>("channels", 2)));
    channel_index_ = std::clamp(static_cast<int>(declare_parameter<int>("channel_index", 0)), 0, channels_ - 1);
    window_size_ = std::clamp(static_cast<int>(declare_parameter<int>("window_size", 4096)), 512, 16384);
    hop_size_ = std::clamp(static_cast<int>(declare_parameter<int>("hop_size", 2048)), 128, window_size_);
    sound_speed_ = declare_parameter<double>("sound_speed_mps", 1500.0);
    min_snr_db_ = declare_parameter<double>("min_snr_db", 3.0);
    require_selection_ = declare_parameter<bool>("require_frequency_selection", true);
    reference_frequency_ = declare_parameter<double>("reference_frequency_hz", 21164.0);
    selected_frequency_ = !require_selection_;

    delta_range_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/pinger_homing/delta_range_m", 20);
    iq_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/pinger_homing/iq_magnitude", 20);
    snr_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/pinger_homing/iq_snr_db", 20);
    snr_stamped_pub_ = create_publisher<audio_common_msgs::msg::Float64Stamped>(
        "/pinger_homing/iq_snr_db_stamped", 20);
    selected_sub_ = create_subscription<std_msgs::msg::Float64>(
        selected_topic_, rclcpp::QoS(1),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (std::isfinite(msg->data) && msg->data > 1000.0) {
            reference_frequency_ = msg->data;
            selected_frequency_ = true;
            have_previous_iq_ = false;
            RCLCPP_INFO(get_logger(), "selected frequency %.2f Hz", reference_frequency_);
          }
        });
    audio_sub_ = create_subscription<audio_common_msgs::msg::AudioData>(
        audio_topic_, rclcpp::SensorDataQoS(),
        [this](const audio_common_msgs::msg::AudioData::SharedPtr msg) { on_audio(*msg); });
    RCLCPP_INFO(get_logger(), "standalone estimator ready; waiting for frequency selection=%s",
                require_selection_ ? "true" : "false");
  }

 private:
  static double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
  }

  static int32_t read_s32(const std::vector<uint8_t> &data, std::size_t i) {
    const uint32_t value = static_cast<uint32_t>(data[i]) |
        (static_cast<uint32_t>(data[i + 1]) << 8) |
        (static_cast<uint32_t>(data[i + 2]) << 16) |
        (static_cast<uint32_t>(data[i + 3]) << 24);
    return static_cast<int32_t>(value);
  }

  std::complex<double> demodulate(const std::vector<double> &window, double frequency) const {
    std::complex<double> sum(0.0, 0.0);
    double weight_sum = 0.0;
    const double denom = static_cast<double>(std::max(1, window_size_ - 1));
    const double step = 2.0 * M_PI * frequency / static_cast<double>(sample_rate_);
    for (std::size_t n = 0; n < window.size(); ++n) {
      const double weight = 0.5 * (1.0 - std::cos(2.0 * M_PI * n / denom));
      const double phase = step * static_cast<double>(sample_cursor_ + n);
      sum += weight * window[n] * std::complex<double>(std::cos(phase), -std::sin(phase));
      weight_sum += weight;
    }
    return sum / std::max(weight_sum, 1.0e-12);
  }

  void on_audio(const audio_common_msgs::msg::AudioData &msg) {
    const std::size_t bytes_per_frame = static_cast<std::size_t>(channels_) * 4U;
    if (bytes_per_frame == 0U) return;
    for (std::size_t frame = 0; frame + bytes_per_frame <= msg.data.size(); frame += bytes_per_frame) {
      const std::size_t offset = frame + static_cast<std::size_t>(channel_index_) * 4U;
      samples_.push_back(static_cast<double>(read_s32(msg.data, offset)) / 2147483648.0);
    }
    while (samples_.size() >= static_cast<std::size_t>(window_size_)) {
      if (selected_frequency_) analyze_window();
      const std::size_t remove = std::min<std::size_t>(hop_size_, samples_.size());
      samples_.erase(samples_.begin(), samples_.begin() + static_cast<std::ptrdiff_t>(remove));
      sample_cursor_ += remove;
    }
    if (samples_.size() > static_cast<std::size_t>(window_size_) * 4U) {
      samples_.erase(samples_.begin(), samples_.end() - static_cast<std::ptrdiff_t>(window_size_ * 2));
    }
  }

  void analyze_window() {
    const std::vector<double> window(samples_.begin(),
                                     samples_.begin() + static_cast<std::ptrdiff_t>(window_size_));
    const auto iq = demodulate(window, reference_frequency_);
    std::vector<double> noise;
    for (const double offset : {-700.0, -450.0, -250.0, 250.0, 450.0, 700.0}) {
      noise.push_back(std::abs(demodulate(window, reference_frequency_ + offset)));
    }
    const double magnitude = std::abs(iq);
    const double noise_magnitude = std::max(median(noise), 1.0e-12);
    const double snr_db = 20.0 * std::log10(std::max(magnitude, 1.0e-12) / noise_magnitude);
    std_msgs::msg::Float64 iq_msg;
    iq_msg.data = magnitude;
    iq_pub_->publish(iq_msg);
    std_msgs::msg::Float64 snr_msg;
    snr_msg.data = snr_db;
    snr_pub_->publish(snr_msg);
    audio_common_msgs::msg::Float64Stamped stamped;
    stamped.header.stamp = now();
    stamped.data = snr_db;
    snr_stamped_pub_->publish(stamped);
    if (snr_db < min_snr_db_ || magnitude < 1.0e-8) {
      have_previous_iq_ = false;
      return;
    }
    if (have_previous_iq_) {
      const double phase = std::atan2(std::imag(iq * std::conj(previous_iq_)),
                                      std::real(iq * std::conj(previous_iq_)));
      std_msgs::msg::Float64 msg;
      msg.data = -sound_speed_ / reference_frequency_ * phase / (2.0 * M_PI);
      delta_range_pub_->publish(msg);
    }
    previous_iq_ = iq;
    have_previous_iq_ = true;
  }

  std::string audio_topic_;
  std::string selected_topic_;
  int sample_rate_{96000};
  int channels_{2};
  int channel_index_{0};
  int window_size_{4096};
  int hop_size_{2048};
  double sound_speed_{1500.0};
  double min_snr_db_{3.0};
  double reference_frequency_{21164.0};
  bool require_selection_{true};
  bool selected_frequency_{false};
  bool have_previous_iq_{false};
  std::complex<double> previous_iq_{0.0, 0.0};
  std::uint64_t sample_cursor_{0};
  std::deque<double> samples_;
  rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr selected_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr delta_range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr iq_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr snr_pub_;
  rclcpp::Publisher<audio_common_msgs::msg::Float64Stamped>::SharedPtr snr_stamped_pub_;
};

}  // namespace kmu26_pinger_homing

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kmu26_pinger_homing::FingerAudioEstimator>());
  rclcpp::shutdown();
  return 0;
}
