#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <unsupported/Eigen/FFT>

#include "audio_common_msgs/msg/audio_data.hpp"
#include "audio_common_msgs/msg/audio_info.hpp"
// ㅣㅣ
namespace audio_capture
{
namespace
{
constexpr double kPi = 3.14159265358979323846;  //Hann window 계산 시 필요한 원주율 pi
constexpr double kUltrasonicMinFrequencyHz = 10000.0; //탐지 최소 주파수
constexpr double kUltrasonicMaxFrequencyHz = 30000.0; //탐지 최대 주파수

//한 번의 FFT 분석 결과를 담는 "중간 결과 상자"
struct DetectionResult
{
  double detected_frequency_hz {0.0}; //탐색 대역에서 가장 강했던 피크 주파수
  double detected_amplitude {0.0};  //그 피크의 선형 진폭
  double detected_level_db {0.0}; //피크를 dB로 변환한 값
  double noise_floor_db {0.0};  //주변 평균 노이즈 레벨(dB)
  double snr_db {0.0}; //신호 대 노이즈 비율(dB)
  double frequency_error_hz {0.0}; //탐지된 주파수와 목표 주파수 간의 오차(Hz)
  bool target_detected {false}; //최종 판정 결과(true: 성공, false: 실패)
};
}  // namespace

class AudioFrequencyDetectorNode : public rclcpp::Node
{
public:
  AudioFrequencyDetectorNode()
  : Node("audio_frequency_detector_node"),
    audio_topic_(this->declare_parameter<std::string>("audio_topic", "/audio")),
    audio_info_topic_(this->declare_parameter<std::string>("audio_info_topic", "/audio_info")),
    sample_rate_(this->declare_parameter<int>("sample_rate", 96000)),
    channels_(this->declare_parameter<int>("channels", 2)),
    channel_index_(this->declare_parameter<int>("channel_index", 0)),// 0으로 해야 왼쪽 채널 사용 --> 하이드로폰 1개기 때문
    sample_format_(this->declare_parameter<std::string>("sample_format", "S32LE")),
    target_frequency_hz_(this->declare_parameter<double>("target_frequency_hz", 18000.0)),//탐지 목표 주파수
    target_tolerance_hz_(this->declare_parameter<double>("target_tolerance_hz", 150.0)),//탐지 오차 범위
    search_half_width_hz_(this->declare_parameter<double>("search_half_width_hz", 500.0)),//목표 주파수 주변 탐지 범위(Hz)
    report_frequencies_hz_(
      this->declare_parameter<std::vector<double>>("report_frequencies_hz", std::vector<double>{})), // 탐지 결과 보고 주파수 목록(빈 벡터로 설정하면 목표 주파수만 보고)
    report_neighbor_bins_(this->declare_parameter<int>("report_neighbor_bins", 1)), // 탐지 결과 보고 주파수 주변 빈도 범위(Hz)
    report_period_ms_(this->declare_parameter<int>("report_period_ms", 500)),
    noise_exclusion_bins_(this->declare_parameter<int>("noise_exclusion_bins", 2)),
    window_size_(static_cast<std::size_t>(this->declare_parameter<int>("window_size", 4096))),
    hop_size_(static_cast<std::size_t>(this->declare_parameter<int>("hop_size", 2048))),
    min_detected_level_db_(this->declare_parameter<double>("min_detected_level_db", -80.0)),
    min_snr_db_(this->declare_parameter<double>("min_snr_db", 8.0)),
    waterfall_enabled_(this->declare_parameter<bool>("waterfall_enabled", true)),
    waterfall_topic_(
      this->declare_parameter<std::string>("waterfall_topic", "waterfall_spectrogram")),
    waterfall_height_(this->declare_parameter<int>("waterfall_height", 256)),
    waterfall_min_frequency_hz_(
      this->declare_parameter<double>("waterfall_min_frequency_hz", kUltrasonicMinFrequencyHz)),
    waterfall_max_frequency_hz_(
      this->declare_parameter<double>("waterfall_max_frequency_hz", kUltrasonicMaxFrequencyHz)),
    waterfall_min_db_(this->declare_parameter<double>("waterfall_min_db", -110.0)),
    waterfall_max_db_(this->declare_parameter<double>("waterfall_max_db", -20.0)),
    warned_bad_channel_(false),
    warned_non_wave_format_(false),
    warned_unsupported_format_(false),
    warned_nyquist_(false),
    warned_no_audio_input_(false),
    coding_format_("wave"),
    window_sum_(0.0),
    fft_(),
    received_audio_messages_(0),
    waterfall_width_bins_(0),
    waterfall_min_bin_(0),
    waterfall_max_bin_(0)
  {
    sanitizeParameters();
    updateWindow();
    updateWaterfallBinRange();
    createPublishers();
    createSubscriptions();
    createTimers();
    logStartupConfiguration();
  }

private:
  void sanitizeParameters()
  {
    if (hop_size_ == 0 || hop_size_ > window_size_) {
      const std::size_t adjusted_hop_size = std::max<std::size_t>(1, window_size_ / 2U);
      RCLCPP_WARN(
        this->get_logger(),
        "hop_size %zu is invalid for window_size %zu. Using %zu instead.",
        hop_size_, window_size_, adjusted_hop_size);
      hop_size_ = adjusted_hop_size;
    }

    if (channels_ < 1) {
      RCLCPP_WARN(this->get_logger(), "channels must be >= 1. Using 1.");
      channels_ = 1;
    }

    if (channel_index_ < 0) {
      RCLCPP_WARN(this->get_logger(), "channel_index must be >= 0. Using channel 0.");
      channel_index_ = 0;
    }

    if (target_frequency_hz_ < kUltrasonicMinFrequencyHz ||
      target_frequency_hz_ > kUltrasonicMaxFrequencyHz)
    {
      const double clamped_target_frequency_hz = std::clamp(
        target_frequency_hz_, kUltrasonicMinFrequencyHz, kUltrasonicMaxFrequencyHz);
      RCLCPP_WARN(
        this->get_logger(),
        "target_frequency_hz must stay in the ultrasonic band %.0f-%.0f Hz. Using %.0f Hz.",
        kUltrasonicMinFrequencyHz, kUltrasonicMaxFrequencyHz, clamped_target_frequency_hz);
      target_frequency_hz_ = clamped_target_frequency_hz;
    }

    if (target_tolerance_hz_ < 0.0) {
      RCLCPP_WARN(this->get_logger(), "target_tolerance_hz must be >= 0. Using 0.");
      target_tolerance_hz_ = 0.0;
    }

    if (search_half_width_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "search_half_width_hz must be > 0. Using 500 Hz.");
      search_half_width_hz_ = 500.0;
    }

    if (search_half_width_hz_ < target_tolerance_hz_) {
      RCLCPP_WARN(
        this->get_logger(),
        "search_half_width_hz %.1f Hz is smaller than target_tolerance_hz %.1f Hz. Expanding the search width.",
        search_half_width_hz_, target_tolerance_hz_);
      search_half_width_hz_ = target_tolerance_hz_;
    }

    if (report_neighbor_bins_ < 0) {
      RCLCPP_WARN(this->get_logger(), "report_neighbor_bins must be >= 0. Using 0.");
      report_neighbor_bins_ = 0;
    }

    if (report_period_ms_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "report_period_ms must be > 0. Using 500 ms.");
      report_period_ms_ = 500;
    }

    if (noise_exclusion_bins_ < 0) {
      RCLCPP_WARN(this->get_logger(), "noise_exclusion_bins must be >= 0. Using 0.");
      noise_exclusion_bins_ = 0;
    }

    if (waterfall_height_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "waterfall_height must be > 0. Using 256.");
      waterfall_height_ = 256;
    }

    if (waterfall_min_frequency_hz_ < 0.0) {
      RCLCPP_WARN(this->get_logger(), "waterfall_min_frequency_hz must be >= 0. Using 0 Hz.");
      waterfall_min_frequency_hz_ = 0.0;
    }

    if (waterfall_max_frequency_hz_ <= waterfall_min_frequency_hz_) {
      RCLCPP_WARN(
        this->get_logger(),
        "waterfall_max_frequency_hz must be greater than waterfall_min_frequency_hz. Using %.0f-%.0f Hz.",
        kUltrasonicMinFrequencyHz,
        kUltrasonicMaxFrequencyHz);
      waterfall_min_frequency_hz_ = kUltrasonicMinFrequencyHz;
      waterfall_max_frequency_hz_ = kUltrasonicMaxFrequencyHz;
    }

    if (waterfall_max_db_ <= waterfall_min_db_) {
      RCLCPP_WARN(
        this->get_logger(),
        "waterfall_max_db must be greater than waterfall_min_db. Using -110 dB to -20 dB.");
      waterfall_min_db_ = -110.0;
      waterfall_max_db_ = -20.0;
    }

    sanitizeReportFrequencies();
  }

  void createPublishers()
  {
    detected_frequency_pub_ = this->create_publisher<std_msgs::msg::Float32>("detected_frequency_hz", 10);
    detected_amplitude_pub_ = this->create_publisher<std_msgs::msg::Float32>("detected_amplitude", 10);
    frequency_error_pub_ = this->create_publisher<std_msgs::msg::Float32>("frequency_error_hz", 10);
    target_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("target_detected", 10);
    if (waterfall_enabled_) {
      waterfall_pub_ = this->create_publisher<sensor_msgs::msg::Image>(waterfall_topic_, 10);
    }
  }

  void createSubscriptions()
  {
    audio_sub_ = this->create_subscription<audio_common_msgs::msg::AudioData>(
      audio_topic_, 10,
      std::bind(&AudioFrequencyDetectorNode::onAudio, this, std::placeholders::_1));

    const auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    audio_info_sub_ = this->create_subscription<audio_common_msgs::msg::AudioInfo>(
      audio_info_topic_, info_qos,
      std::bind(&AudioFrequencyDetectorNode::onAudioInfo, this, std::placeholders::_1));
  }

  void createTimers()
  {
    input_watchdog_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&AudioFrequencyDetectorNode::warnIfNoAudioInput, this));
  }

  void logStartupConfiguration() const
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Listening on %s and tracking %.0f Hz within +/- %.0f Hz. Detection tolerance is +/- %.0f Hz with a %zu-sample window.",
      audio_topic_.c_str(), target_frequency_hz_, search_half_width_hz_, target_tolerance_hz_,
      window_size_);
  }

  void onAudioInfo(const audio_common_msgs::msg::AudioInfo::ConstSharedPtr msg)
  {
    if (msg->channels == 0 || msg->sample_rate == 0) {
      RCLCPP_WARN(this->get_logger(), "Ignoring invalid audio_info with zero channels or sample rate.");
      return;
    }

    updateAudioMetadata(*msg);
    validateAudioMetadata();
    if (updateWaterfallBinRange()) {
      resetWaterfallImage();
    }
  }

  void onAudio(const audio_common_msgs::msg::AudioData::ConstSharedPtr msg)
  {
    ++received_audio_messages_;
    if (!appendSamples(msg->data)) {
      return;
    }

    while (sample_buffer_.size() >= window_size_) {
      analyzeWindow();
      for (std::size_t i = 0; i < hop_size_ && !sample_buffer_.empty(); ++i) {
        sample_buffer_.pop_front();
      }
    }
  }

  bool appendSamples(const std::vector<uint8_t> & bytes)
  {
    const std::size_t bytes_per_sample = getBytesPerSample();
    if (bytes_per_sample == 0U) {
      if (!warned_unsupported_format_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Unsupported sample_format '%s'. Supported formats are S16LE and S32LE.",
          sample_format_.c_str());
        warned_unsupported_format_ = true;
      }
      return false;
    }

    if (channels_ < 1) {
      return false;
    }

    clampChannelIndexIfNeeded();

    const std::size_t frame_size = bytes_per_sample * static_cast<std::size_t>(channels_);
    if (frame_size == 0U || bytes.size() < frame_size) {
      return false;
    }

    const std::size_t frame_count = bytes.size() / frame_size;
    for (std::size_t frame = 0; frame < frame_count; ++frame) {
      const std::uint8_t * sample_ptr = bytes.data() + (frame * frame_size) +
        (static_cast<std::size_t>(channel_index_) * bytes_per_sample);

      float sample = 0.0F;
      if (!decodeSample(sample_ptr, bytes_per_sample, sample)) {
        return false;
      }
      sample_buffer_.push_back(sample);
    }

    return true;
  }

  std::size_t getBytesPerSample() const
  {
    if (sample_format_ == "S16LE") {
      return 2U;
    }
    if (sample_format_ == "S32LE") {
      return 4U;
    }
    return 0U;
  }

  bool decodeSample(const std::uint8_t * sample_ptr, std::size_t bytes_per_sample, float & sample) const
  {
    if (bytes_per_sample == 2U) {
      std::int16_t raw_sample = 0;
      std::memcpy(&raw_sample, sample_ptr, sizeof(raw_sample));
      sample = static_cast<float>(raw_sample) / 32768.0F;
      return true;
    }

    if (bytes_per_sample == 4U) {
      std::int32_t raw_sample = 0;
      std::memcpy(&raw_sample, sample_ptr, sizeof(raw_sample));
      sample = static_cast<float>(raw_sample) / 2147483648.0F;
      return true;
    }

    return false;
  }

  void analyzeWindow()
  {
    if (!isReadyForAnalysis()) {
      return;
    }

    const std::vector<float> fft_input = buildWindowedInput();
    const std::vector<std::complex<float>> spectrum = computeSpectrum(fft_input);
    publishWaterfall(spectrum);

    DetectionResult result;
    if (!detectTargetFrequency(spectrum, result)) {
      return;
    }

    publishDetection(result);
    reportSignalStrengths(spectrum, result);
  }

  void updateWindow()
  {
    window_.resize(window_size_);
    window_sum_ = 0.0;

    if (window_size_ == 1U) {
      window_[0] = 1.0;
      window_sum_ = 1.0;
      return;
    }

    for (std::size_t i = 0; i < window_size_; ++i) {
      const double window_value = 0.5 - (0.5 * std::cos((2.0 * kPi * i) / (window_size_ - 1U)));
      window_[i] = window_value;
      window_sum_ += window_value;
    }
  }

  void updateAudioMetadata(const audio_common_msgs::msg::AudioInfo & msg)
  {
    channels_ = static_cast<int>(msg.channels);
    sample_rate_ = static_cast<int>(msg.sample_rate);
    sample_format_ = msg.sample_format;
    coding_format_ = msg.coding_format;
  }

  void validateAudioMetadata()
  {
    clampChannelIndexIfNeeded();

    if (coding_format_ != "wave" && !warned_non_wave_format_) {
      RCLCPP_WARN(
        this->get_logger(),
        "audio_info reports coding_format='%s'. Frequency detection expects raw PCM, so run audio_capture with format:=wave.",
        coding_format_.c_str());
      warned_non_wave_format_ = true;
    }

    const double search_upper_hz = target_frequency_hz_ + search_half_width_hz_;
    if (sample_rate_ / 2.0 < search_upper_hz && !warned_nyquist_) {
      RCLCPP_WARN(
        this->get_logger(),
        "sample_rate=%d Hz cannot fully observe %.0f Hz +/- %.0f Hz. Increase sample_rate or narrow the search width.",
        sample_rate_, target_frequency_hz_, search_half_width_hz_);
      warned_nyquist_ = true;
    }
  }

  void sanitizeReportFrequencies()
  {
    if (report_frequencies_hz_.empty()) {
      report_frequencies_hz_.push_back(target_frequency_hz_);
      return;
    }

    for (double & frequency_hz : report_frequencies_hz_) {
      if (frequency_hz < kUltrasonicMinFrequencyHz || frequency_hz > kUltrasonicMaxFrequencyHz) {
        const double clamped_frequency_hz = std::clamp(
          frequency_hz, kUltrasonicMinFrequencyHz, kUltrasonicMaxFrequencyHz);
        RCLCPP_WARN(
          this->get_logger(),
          "report_frequencies_hz entry %.1f Hz is outside %.0f-%.0f Hz. Using %.1f Hz.",
          frequency_hz, kUltrasonicMinFrequencyHz, kUltrasonicMaxFrequencyHz, clamped_frequency_hz);
        frequency_hz = clamped_frequency_hz;
      }
    }
  }

  void clampChannelIndexIfNeeded()
  {
    if (channel_index_ >= channels_) {
      if (!warned_bad_channel_) {
        RCLCPP_WARN(
          this->get_logger(),
          "channel_index %d is outside the %d available channels. Using channel 0.",
          channel_index_, channels_);
        warned_bad_channel_ = true;
      }
      channel_index_ = 0;
    }
  }

  bool isReadyForAnalysis() const
  {
    return sample_rate_ > 0 && window_size_ > 0 && window_sum_ > 0.0;
  }

  std::vector<float> buildWindowedInput() const
  {
    std::vector<float> fft_input(window_size_);
    for (std::size_t i = 0; i < window_size_; ++i) {
      fft_input[i] = static_cast<float>(sample_buffer_[i] * window_[i]);
    }
    return fft_input;
  }

  std::vector<std::complex<float>> computeSpectrum(const std::vector<float> & fft_input)
  {
    std::vector<std::complex<float>> spectrum;
    fft_.fwd(spectrum, fft_input);
    return spectrum;
  }

  bool detectTargetFrequency(
    const std::vector<std::complex<float>> & spectrum,
    DetectionResult & result) const
  {
    int min_bin = 0;
    int max_bin = 0;
    if (!computeSearchBinRange(min_bin, max_bin)) {
      return false;
    }

    int peak_bin = min_bin;
    double peak_amplitude = 0.0;
    for (int bin = min_bin; bin <= max_bin; ++bin) {
      const double amplitude = amplitudeForBin(spectrum, bin);
      if (amplitude > peak_amplitude) {
        peak_amplitude = amplitude;
        peak_bin = bin;
      }
    }

    result.detected_frequency_hz = frequencyForBin(peak_bin);
    result.detected_amplitude = peak_amplitude;
    result.detected_level_db = amplitudeToDb(peak_amplitude);
    result.noise_floor_db = estimateNoiseFloorDb(spectrum, min_bin, max_bin, peak_bin);
    result.snr_db = signalToNoiseRatioDb(peak_bin, spectrum, min_bin, max_bin, peak_bin);
    result.frequency_error_hz = result.detected_frequency_hz - target_frequency_hz_;
    result.target_detected =
      result.detected_level_db >= min_detected_level_db_ &&
      result.snr_db >= min_snr_db_ &&
      std::abs(result.frequency_error_hz) <= target_tolerance_hz_;
    return true;
  }

  bool computeSearchBinRange(int & min_bin, int & max_bin) const
  {
    const double nyquist_hz = sampleRateNyquistHz();
    if (nyquist_hz <= 0.0) {
      return false;
    }

    const double lower_frequency_hz = std::max(
      kUltrasonicMinFrequencyHz, target_frequency_hz_ - search_half_width_hz_);
    const double upper_frequency_hz = std::min(
      {kUltrasonicMaxFrequencyHz, target_frequency_hz_ + search_half_width_hz_, nyquist_hz});

    if (upper_frequency_hz <= lower_frequency_hz) {
      return false;
    }

    min_bin = std::max(1, static_cast<int>(std::ceil(lower_frequency_hz * window_size_ / sample_rate_)));
    max_bin = std::min(
      static_cast<int>(window_size_ / 2U),
      static_cast<int>(std::floor(upper_frequency_hz * window_size_ / sample_rate_)));
    return max_bin >= min_bin;
  }

  int frequencyToNearestBin(double frequency_hz) const
  {
    if (sample_rate_ <= 0 || window_size_ == 0U) {
      return 0;
    }

    return static_cast<int>(std::lround(frequency_hz * window_size_ / sample_rate_));
  }

  double amplitudeForBin(const std::vector<std::complex<float>> & spectrum, int bin) const
  {
    return (2.0 * std::abs(spectrum[bin])) / window_sum_;
  }

  double amplitudeToDb(double amplitude) const
  {
    return 20.0 * std::log10(std::max(amplitude, 1.0e-12));
  }

  double powerForBin(const std::vector<std::complex<float>> & spectrum, int bin) const
  {
    const double amplitude = amplitudeForBin(spectrum, bin);
    return amplitude * amplitude;
  }

  double powerToDb(double power) const
  {
    return 10.0 * std::log10(std::max(power, 1.0e-24));
  }

  double estimateNoiseFloorDb(
    const std::vector<std::complex<float>> & spectrum,
    int min_bin,
    int max_bin,
    int peak_bin) const
  {
    double noise_power_sum = 0.0;
    int noise_bin_count = 0;

    for (int bin = min_bin; bin <= max_bin; ++bin) {
      if (std::abs(bin - peak_bin) <= noise_exclusion_bins_) {
        continue;
      }

      noise_power_sum += powerForBin(spectrum, bin);
      ++noise_bin_count;
    }

    if (noise_bin_count == 0) {
      return -120.0;
    }

    const double average_noise_power = noise_power_sum / static_cast<double>(noise_bin_count);
    return powerToDb(average_noise_power);
  }

  double signalToNoiseRatioDb(
    int signal_bin,
    const std::vector<std::complex<float>> & spectrum,
    int min_bin,
    int max_bin,
    int peak_bin) const
  {
    const double signal_power = powerForBin(spectrum, signal_bin);
    double noise_power_sum = 0.0;
    int noise_bin_count = 0;

    for (int bin = min_bin; bin <= max_bin; ++bin) {
      if (std::abs(bin - peak_bin) <= noise_exclusion_bins_) {
        continue;
      }
      noise_power_sum += powerForBin(spectrum, bin);
      ++noise_bin_count;
    }

    if (noise_bin_count == 0) {
      return 0.0;
    }

    const double average_noise_power = noise_power_sum / static_cast<double>(noise_bin_count);
    return powerToDb(signal_power) - powerToDb(average_noise_power);
  }

  bool updateWaterfallBinRange()
  {
    if (!waterfall_enabled_ || sample_rate_ <= 0 || window_size_ == 0U) {
      return false;
    }

    const int nyquist_bin = static_cast<int>(window_size_ / 2U);
    const double upper_hz = std::min(waterfall_max_frequency_hz_, sampleRateNyquistHz());
    const int min_bin = std::max(1, frequencyToNearestBin(waterfall_min_frequency_hz_));
    const int max_bin = std::min(nyquist_bin, frequencyToNearestBin(upper_hz));
    if (max_bin <= min_bin) {
      return false;
    }

    const int width_bins = max_bin - min_bin + 1;
    const bool changed =
      min_bin != waterfall_min_bin_ ||
      max_bin != waterfall_max_bin_ ||
      width_bins != waterfall_width_bins_;

    waterfall_min_bin_ = min_bin;
    waterfall_max_bin_ = max_bin;
    waterfall_width_bins_ = width_bins;
    return changed;
  }

  void resetWaterfallImage()
  {
    if (!waterfall_enabled_ || waterfall_width_bins_ <= 0 || waterfall_height_ <= 0) {
      waterfall_image_.clear();
      return;
    }

    waterfall_image_.assign(
      static_cast<std::size_t>(waterfall_width_bins_) * static_cast<std::size_t>(waterfall_height_),
      0U);
  }

  std::uint8_t normalizeDbToPixel(double db) const
  {
    const double clamped_db = std::clamp(db, waterfall_min_db_, waterfall_max_db_);
    const double normalized = (clamped_db - waterfall_min_db_) / (waterfall_max_db_ - waterfall_min_db_);
    return static_cast<std::uint8_t>(std::lround(normalized * 255.0));
  }

  void publishWaterfall(const std::vector<std::complex<float>> & spectrum)
  {
    if (!waterfall_enabled_ || !waterfall_pub_) {
      return;
    }

    if (updateWaterfallBinRange()) {
      resetWaterfallImage();
    }
    if (waterfall_width_bins_ <= 0 || waterfall_height_ <= 0) {
      return;
    }

    const std::size_t width = static_cast<std::size_t>(waterfall_width_bins_);
    const std::size_t height = static_cast<std::size_t>(waterfall_height_);
    const std::size_t required_size = width * height;
    if (waterfall_image_.size() != required_size) {
      waterfall_image_.assign(required_size, 0U);
    }

    if (height > 1U) {
      std::memmove(
        waterfall_image_.data(),
        waterfall_image_.data() + width,
        width * (height - 1U));
    }

    const std::size_t last_row_offset = width * (height - 1U);
    for (int x = 0; x < waterfall_width_bins_; ++x) {
      const int bin = waterfall_min_bin_ + x;
      const double db = powerToDb(powerForBin(spectrum, bin));
      waterfall_image_[last_row_offset + static_cast<std::size_t>(x)] = normalizeDbToPixel(db);
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = this->get_clock()->now();
    image_msg.header.frame_id = "hydrophone";
    image_msg.height = static_cast<std::uint32_t>(height);
    image_msg.width = static_cast<std::uint32_t>(width);
    image_msg.encoding = "mono8";
    image_msg.is_bigendian = 0;
    image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(width);
    image_msg.data = waterfall_image_;
    waterfall_pub_->publish(image_msg);
  }

  double frequencyForBin(int bin) const
  {
    return static_cast<double>(bin) * static_cast<double>(sample_rate_) / window_size_;
  }

  double sampleRateNyquistHz() const
  {
    return static_cast<double>(sample_rate_) / 2.0;
  }

  double reportDbForFrequency(
    const std::vector<std::complex<float>> & spectrum,
    double frequency_hz) const
  {
    const int center_bin = frequencyToNearestBin(frequency_hz);
    const int min_bin = std::max(1, center_bin - report_neighbor_bins_);
    const int max_bin = std::min(static_cast<int>(window_size_ / 2U), center_bin + report_neighbor_bins_);

    double peak_magnitude = 1.0e-12;
    for (int bin = min_bin; bin <= max_bin; ++bin) {
      peak_magnitude = std::max(peak_magnitude, static_cast<double>(std::abs(spectrum[bin])));
    }

    return 20.0 * std::log10(peak_magnitude);
  }

  void reportSignalStrengths(
    const std::vector<std::complex<float>> & spectrum,
    const DetectionResult & result)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "Signal Strength -> ";

    for (std::size_t i = 0; i < report_frequencies_hz_.size(); ++i) {
      if (i > 0U) {
        stream << ", ";
      }

      const double frequency_khz = report_frequencies_hz_[i] / 1000.0;
      const double magnitude_db = reportDbForFrequency(spectrum, report_frequencies_hz_[i]);
      stream << frequency_khz << "kHz: [" << magnitude_db << " dB]";
    }

    stream << " | detected: " << std::setprecision(1) << result.detected_frequency_hz << " Hz";
    stream << ", error: " << result.frequency_error_hz << " Hz";
    stream << ", level: " << result.detected_level_db << " dB";
    stream << ", noise: " << result.noise_floor_db << " dB";
    stream << ", snr: " << result.snr_db << " dB";
    stream << ", target_detected: " << (result.target_detected ? "true" : "false");

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      report_period_ms_,
      "%s",
      stream.str().c_str());
  }

  void warnIfNoAudioInput()
  {
    if (received_audio_messages_ == 0U && !warned_no_audio_input_) {
      RCLCPP_WARN(
        this->get_logger(),
        "No audio messages have been received on %s yet. Check that audio_capture is running and publishing /audio.",
        audio_topic_.c_str());
      warned_no_audio_input_ = true;
    }
  }

  void publishDetection(const DetectionResult & result)
  {
    std_msgs::msg::Float32 frequency_msg;
    frequency_msg.data = static_cast<float>(result.detected_frequency_hz);
    detected_frequency_pub_->publish(frequency_msg);

    std_msgs::msg::Float32 amplitude_msg;
    amplitude_msg.data = static_cast<float>(result.detected_amplitude);
    detected_amplitude_pub_->publish(amplitude_msg);

    std_msgs::msg::Float32 error_msg;
    error_msg.data = static_cast<float>(result.frequency_error_hz);
    frequency_error_pub_->publish(error_msg);

    std_msgs::msg::Bool detection_msg;
    detection_msg.data = result.target_detected;
    target_detected_pub_->publish(detection_msg);
  }

  std::string audio_topic_;
  std::string audio_info_topic_;
  int sample_rate_;
  int channels_;
  int channel_index_;
  std::string sample_format_;
  double target_frequency_hz_;
  double target_tolerance_hz_;
  double search_half_width_hz_;
  std::vector<double> report_frequencies_hz_;
  int report_neighbor_bins_;
  int report_period_ms_;
  int noise_exclusion_bins_;
  std::size_t window_size_;
  std::size_t hop_size_;
  double min_detected_level_db_;
  double min_snr_db_;
  bool waterfall_enabled_;
  std::string waterfall_topic_;
  int waterfall_height_;
  double waterfall_min_frequency_hz_;
  double waterfall_max_frequency_hz_;
  double waterfall_min_db_;
  double waterfall_max_db_;
  bool warned_bad_channel_;
  bool warned_non_wave_format_;
  bool warned_unsupported_format_;
  bool warned_nyquist_;
  bool warned_no_audio_input_;
  std::string coding_format_;
  std::deque<float> sample_buffer_;
  std::vector<double> window_;
  double window_sum_;
  Eigen::FFT<float> fft_;
  std::size_t received_audio_messages_;
  int waterfall_width_bins_;
  int waterfall_min_bin_;
  int waterfall_max_bin_;
  std::vector<std::uint8_t> waterfall_image_;

  rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr audio_sub_;
  rclcpp::Subscription<audio_common_msgs::msg::AudioInfo>::SharedPtr audio_info_sub_;
  rclcpp::TimerBase::SharedPtr input_watchdog_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr detected_frequency_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr detected_amplitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr frequency_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_detected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr waterfall_pub_;
};
}  // namespace audio_capture

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<audio_capture::AudioFrequencyDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
