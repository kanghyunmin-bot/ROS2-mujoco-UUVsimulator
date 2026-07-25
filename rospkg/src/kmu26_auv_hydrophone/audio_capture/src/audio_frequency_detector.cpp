//타겟 주파수 탐지 알고리즘

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <complex>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <audio_common_msgs/msg/audio_data.hpp>
#include <audio_common_msgs/msg/audio_data_stamped.hpp>
#include <audio_common_msgs/msg/float64_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>
#include <unsupported/Eigen/FFT>


namespace audio_capture
{
class AudioFrequencyDetectorNode : public rclcpp::Node
{
    public:
    explicit AudioFrequencyDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("audio_frequency_detector", options) {
        // V2의 SNR-odometry 동기화를 위해 캡처 시작 시각이 포함된 오디오를 받는다.
        audio_stamped_sub_ =
            this->create_subscription<audio_common_msgs::msg::AudioDataStamped>(
        "/audio_stamped",
        rclcpp::QoS(10),
        std::bind(&AudioFrequencyDetectorNode::audio_stamped_callback, this, std::placeholders::_1));
        //std::placeholders::_1: 콜백에 들어갈 첫 번째 인자를 의미 
        locked_frequency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/audio/locked_frequency_hz", 10);
        snr_db_pub_ =
            this->create_publisher<audio_common_msgs::msg::Float64Stamped>(
            "/audio_frequency_detector/snr_db_stamped", 20);

        // 분석 thread는 생성자 마지막에서 한 번만 시작한다.
        worker_thread_ = std::thread(&AudioFrequencyDetectorNode::analysis_loop, this);
    }

    ~AudioFrequencyDetectorNode()//소멸자
    {
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);    //뮤텍스락을 잡고
            stop_worker_ = true;
        }
        buffer_cv_.notify_one();   //waiting 큐 맨 앞에 있는 쓰레드를 깨움.

        if (worker_thread_.joinable()) {    //분석 thread 존재하면
            worker_thread_.join();   // 끝날 때까지 기다림.
        }
    }

    private:
    void audio_stamped_callback(
        const audio_common_msgs::msg::AudioDataStamped::ConstSharedPtr msg)
    {
        rclcpp::Time buffer_start_stamp(msg->header.stamp);
        if (buffer_start_stamp.nanoseconds() <= 0) {
            buffer_start_stamp = now() - rclcpp::Duration::from_seconds(
                static_cast<double>(msg->audio.data.size() / frame_size_) /
                static_cast<double>(sampling_rate_));
        }
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);    //뮤텍스락을 잡고
            if (!sample_buffer_.empty()) {
                const rclcpp::Time expected_stamp =
                    sample_buffer_start_stamp_ + rclcpp::Duration::from_seconds(
                    static_cast<double>(sample_buffer_.size()) /
                    static_cast<double>(sampling_rate_));
                if (std::abs((buffer_start_stamp - expected_stamp).seconds()) > 0.02) {
                    sample_buffer_.clear();
                    candidate_frequencies_hz_.clear();
                    locked_on_ = false;
                }
            }
            if (sample_buffer_.empty()) {
                sample_buffer_start_stamp_ = buffer_start_stamp;
            }
            append_samples_from_pcm(msg->audio.data);    //오디오 데이터를 버퍼에 추가
        }

        // 분석 thread가 기다리고 있을 수 있으니 새 샘플이 들어왔다고 알려준다.
        buffer_cv_.notify_one();   //waiting 큐 맨 앞에 있는 쓰레드를 깨움.
    }

    void analysis_loop()
    {
        while (rclcpp::ok()) {
            std::vector<double> window;
            rclcpp::Time window_center_stamp(0, 0, RCL_ROS_TIME);

            {
                // Entry section
                std::unique_lock<std::mutex> lock(buffer_mutex_); 

                // window_size_만큼 샘플이 쌓일 때까지 thread는 잠들어 있음.
                // wait 중에는 lock을 잠시 놓고, 조건이 만족되면 다시 lock을 잡은 상태로 돌아옴.
                buffer_cv_.wait(lock, [this]() {
                    return stop_worker_ || sample_buffer_.size() >= window_size_;
                }); //stop_worker_가 true이거나 sample_buffer_.size()가 window_size_보다 크면 waiting 큐에서 나옴.

                if (stop_worker_) {
                    break;
                }

                // Critical section
                // mutex를 오래 잡지 않기 위해, 분석할 구간만 복사하고 바로 버퍼를 정리.
                window.assign(sample_buffer_.begin(), sample_buffer_.begin() + window_size_);
                window_center_stamp =
                    sample_buffer_start_stamp_ + rclcpp::Duration::from_seconds(
                    0.5 * static_cast<double>(window_size_) /
                    static_cast<double>(sampling_rate_));
                sample_buffer_.erase(sample_buffer_.begin(), sample_buffer_.begin() + hop_size_);
                sample_buffer_start_stamp_ =
                    sample_buffer_start_stamp_ + rclcpp::Duration::from_seconds(
                    static_cast<double>(hop_size_) /
                    static_cast<double>(sampling_rate_));
            }  // Exit section: 이 블록을 벗어나면 unique_lock 소멸자가 mutex를 자동으로 unlock함.

            // Remainder section: FFT/RMS 같은 실제 분석은 공유 버퍼를 안 쓰므로 lock 밖에서 수행함.
            analyze_window(window, window_center_stamp);
        }
    }

    //FFT 분석 함수
    void analyze_window(
        const std::vector<double> & window,
        const rclcpp::Time & window_center_stamp)
    {
        std::vector<double> fft_input(fft_size_, 0.0); // window 뒤를 0으로 채워 FFT bin 간격을 촘촘하게 만든다.
        const double hann_denominator = static_cast<double>(window.size() - 1); // N-1

        // FFT 전에 Hann window를 곱해 window 경계에서 생기는 주파수 누설을 줄인다.
        for (std::size_t n = 0; n < window.size(); ++n) {
            const double hann = 0.5 * (1.0 - std::cos((2.0 * M_PI * static_cast<double>(n)) / hann_denominator));
            fft_input[n] = window[n] * hann;
        }

        std::vector<std::complex<double>> frequency_bins;   //FFT 결과를 저장할 벡터
        fft_.fwd(frequency_bins, fft_input);   //zero-padded FFT 계산

        const double frequency_resolution_hz =  //zero-padding 후 bin 간격: 96000 / 32768 = 2.9296875 Hz
            static_cast<double>(sampling_rate_) / static_cast<double>(fft_input.size()); 

        // 입력이 실수 신호라서 Nyquist까지만.
        const std::size_t nyquist_bin = frequency_bins.size() / 2;
        const std::size_t min_bin = std::max<std::size_t>(
            1,
            static_cast<std::size_t>(std::ceil(min_detection_frequency_hz_ / frequency_resolution_hz)));
        const std::size_t max_bin = std::min<std::size_t>(
            nyquist_bin,
            static_cast<std::size_t>(std::floor(max_detection_frequency_hz_ / frequency_resolution_hz)));

        std::size_t peak_bin = min_bin;
        double peak_target_frequency_hz = 0.0;
        double peak_magnitude = 0.0;


        // min_bin부터 max_bin까지 반복하면서 핑거 후보 주파수 주변의 가장 큰 진폭을 가진 bin을 찾는다.
        for (std::size_t bin = min_bin; bin <= max_bin; ++bin) {
            const double frequency_hz = static_cast<double>(bin) * frequency_resolution_hz; //bin을 주파수로 변환
            const double magnitude = std::abs(frequency_bins[bin]); //해당 bin의 진폭

            if (std::abs(frequency_hz - blacklist_frequency_hz_) <= blacklist_half_width_hz_) { //blacklist 주파수 주변에 있으면 건너뜀
                continue;
            }

            double matched_target_frequency_hz = 0.0;
            bool is_candidate_frequency = false;
            for (const double target_frequency_hz : target_frequencies_hz_) {   //target_frequencies_hz_ 벡터에 있는 주파수 하나씩 반복
                if (std::abs(frequency_hz - target_frequency_hz) <= target_search_half_width_hz_) { //주파수 오차가 1kHz 이하이면 탐지 가능한 주파수로 판정
                    matched_target_frequency_hz = target_frequency_hz;
                    is_candidate_frequency = true;
                    break;
                }
            }

            if (!is_candidate_frequency) {  //탐지 가능한 주파수가 아니면 건너뜀
                continue;
            }

            if (magnitude > peak_magnitude) {   //현재 bin의 진폭이 이전 bin의 진폭보다 크면 피크 진폭과 피크 주파수 업데이트
                peak_magnitude = magnitude;
                peak_bin = bin;
                peak_target_frequency_hz = matched_target_frequency_hz;
            }
        }  
        
        // peak_bin을 주파수로 변환
        const double peak_frequency_hz = static_cast<double>(peak_bin) * frequency_resolution_hz;
        const double noise_floor = calculate_local_noise_floor(
            frequency_bins,
            peak_bin,
            min_bin,
            max_bin,
            frequency_resolution_hz);
        const double snr_db = 20.0 * std::log10((peak_magnitude + epsilon_) / (noise_floor + epsilon_));   //SNR 계산
        const bool snr_detected = snr_db >= min_snr_db_; //SNR이 최소 SNR 이상이면 true, 아니면 false

        audio_common_msgs::msg::Float64Stamped snr_msg;
        snr_msg.header.stamp = window_center_stamp;
        snr_msg.data = snr_db;
        snr_db_pub_->publish(snr_msg);

        update_lock_state(snr_detected, peak_frequency_hz); 
        if (locked_on_) {
            std_msgs::msg::Float64 frequency_msg;
            frequency_msg.data = locked_frequency_hz_;
            locked_frequency_pub_->publish(frequency_msg);
        }

        const std::string lock_detail =
            locked_on_ ? ", locked_freq: " + std::to_string(locked_frequency_hz_) + " Hz" : "";

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "target %.0f Hz peak: %.1f Hz, mag: %.6f, noise: %.6f, snr: %.1f dB, lock: %s%s",
            peak_target_frequency_hz,
            peak_frequency_hz,
            peak_magnitude,
            noise_floor,
            snr_db,
            locked_on_ ? "true" : "false",
            lock_detail.c_str());
    }

    double calculate_median(std::vector<double> values) const
    {
        if (values.empty()) {
            return 0.0;
        }
        const std::size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        return values[middle];
    }

    double calculate_local_noise_floor(
        const std::vector<std::complex<double>> & frequency_bins,
        const std::size_t peak_bin,
        const std::size_t min_bin,
        const std::size_t max_bin,
        const double frequency_resolution_hz) const
    {
        std::vector<double> noise_magnitudes;
        const std::size_t offset_min =
            static_cast<std::size_t>(std::ceil(noise_guard_half_width_hz_ / frequency_resolution_hz));
        const std::size_t offset_max =
            static_cast<std::size_t>(std::floor(noise_floor_half_width_hz_ / frequency_resolution_hz));
        noise_magnitudes.reserve((offset_max - offset_min + 1) * 2);

        for (std::size_t offset = offset_min; offset <= offset_max; ++offset) {
            if (peak_bin >= min_bin + offset) {
                const std::size_t left_bin = peak_bin - offset;
                const double left_frequency_hz = static_cast<double>(left_bin) * frequency_resolution_hz;
                if (std::abs(left_frequency_hz - blacklist_frequency_hz_) > blacklist_half_width_hz_) {
                    noise_magnitudes.push_back(std::abs(frequency_bins[left_bin]));
                }
            }

            if (peak_bin + offset <= max_bin) {
                const std::size_t right_bin = peak_bin + offset;
                const double right_frequency_hz = static_cast<double>(right_bin) * frequency_resolution_hz;
                if (std::abs(right_frequency_hz - blacklist_frequency_hz_) > blacklist_half_width_hz_) {
                    noise_magnitudes.push_back(std::abs(frequency_bins[right_bin]));
                }
            }
        }

        return calculate_median(noise_magnitudes);
    }

    void update_lock_state(const bool snr_detected, const double peak_frequency_hz)
    {
        // SNR이 최소 SNR 이상이 아니면 주파수 판정 결과를 false로 설정
        if (!snr_detected) {
            candidate_frequencies_hz_.clear();   // 판정할 주파수 후보들 비우기
            locked_on_ = false;   // 주파수 판정 결과를 false로 설정
            return;
        }
        // SNR이 최소 SNR 이상이면 판정할 주파수 후보들에 추가
        candidate_frequencies_hz_.push_back(peak_frequency_hz);
        if (candidate_frequencies_hz_.size() > lock_window_count_) {   // 판정할 주파수 후보들의 개수가 최대 개수를 초과하면 맨 앞의 주파수 후보를 제거
            candidate_frequencies_hz_.pop_front();
        }

        if (candidate_frequencies_hz_.size() < lock_window_count_) {   // 판정할 주파수 후보들의 개수가 최소 개수를 미만이면 주파수 판정 결과를 false로 설정
            locked_on_ = false;
            return;
        }

        // 7개 후보 중 5개 이상이 lock_tolerance_hz_ 안에 몰리면 같은 핑거 주파수로 판단한다.
        double mode_frequency_hz = candidate_frequencies_hz_.front();   // 가장 많이 몰린 후보 그룹의 대표 피크 주파수
        std::size_t mode_count = 0;
        for (const double candidate_frequency_hz : candidate_frequencies_hz_) {
            std::size_t candidate_count = 0;
            for (const double other_frequency_hz : candidate_frequencies_hz_) {
                if (std::abs(other_frequency_hz - candidate_frequency_hz) <= lock_tolerance_hz_) {
                    ++candidate_count;
                }
            }
            if (candidate_count > mode_count) {
                mode_count = candidate_count;
                mode_frequency_hz = candidate_frequency_hz;
            }
        }
        if (mode_count < min_lock_count_) {
            locked_on_ = false;
            return;
        }
        locked_frequency_hz_ = mode_frequency_hz;   // 판정할 주파수 후보들의 최빈값을 lock-on된 주파수로 설정
        locked_on_ = true;   // 주파수 판정 결과를 true로 설정
    }

    
    // msg->data는 uint8 바이트 배열이므로, 실제 오디오 샘플 값으로 해석해서 버퍼에 쌓는다.
    void append_samples_from_pcm(const std::vector<uint8_t> & data)
    {
        // channel_index_가 0이면 각 frame의 첫 번째 채널, 1이면 두 번째 채널을 읽는다.
        const std::size_t channel_offset = channel_index_ * bytes_per_sample_;// 0 * 4 = 0
        // frame 단위로 이동하면서 원하는 채널의 샘플만 꺼냄.(현재는 왼쪽 채널만 사용)
        for (std::size_t frame_start = 0; frame_start + frame_size_ <= data.size();
            frame_start += frame_size_)
        {
            const std::size_t sample_offset = frame_start + channel_offset;
            // ***토픽으로 들어오는 채널의 4바이트를 little-endian 순서로 합쳐 S32LE 샘플로 재해석***
            const int32_t sample = read_int32_little_endian(data, sample_offset);
            // int32 샘플을 -1.0 ~ 1.0 근처의 실수값으로 정규화하여 RMS/FFT 계산 용이성 높임.
            const double normalized_sample = static_cast<double>(sample) / 2147483648.0;
            sample_buffer_.push_back(normalized_sample);
        }
    }

    // little-endian은 낮은 자리 바이트가 먼저 오기에 shift 연산: b0 | b1<<8 | b2<<16 | b3<<24.
    int32_t read_int32_little_endian(const std::vector<uint8_t> & data, const std::size_t offset) const
    {
        const uint32_t raw =
            static_cast<uint32_t>(data[offset]) |
            (static_cast<uint32_t>(data[offset + 1]) << 8) |
            (static_cast<uint32_t>(data[offset + 2]) << 16) |
            (static_cast<uint32_t>(data[offset + 3]) << 24);

        // 같은 32비트 패턴을 signed int32로 해석해서 음수 샘플까지 표현
        return static_cast<int32_t>(raw);
    }

    // 타임스탬프가 포함된 오디오 구독자와 FFT SNR 발행자
    rclcpp::Subscription<audio_common_msgs::msg::AudioDataStamped>::SharedPtr audio_stamped_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr locked_frequency_pub_;
    rclcpp::Publisher<audio_common_msgs::msg::Float64Stamped>::SharedPtr snr_db_pub_;

    // 변환된 단일 채널 오디오 샘플을 계속 누적하는 버퍼
    std::vector<double> sample_buffer_;
    rclcpp::Time sample_buffer_start_stamp_{0, 0, RCL_ROS_TIME};

    // S32LE, 2채널, 샘플당 4바이트.
    std::size_t channels_ = 2;
    std::size_t channel_index_ = 0;
    std::size_t bytes_per_sample_ = 4;
    std::size_t frame_size_ = channels_ * bytes_per_sample_; // 8바이트


    std::mutex buffer_mutex_;//뮤텍스락
    std::condition_variable buffer_cv_;//조건 변수
    std::thread worker_thread_;//분석 thread
    bool stop_worker_ = false;//분석 thread 종료 플래그
    //FFT할 때 사용할 윈도우 크기와 홉 크기
    std::size_t window_size_ = 4096;
    std::size_t hop_size_ = 2048;
    std::size_t fft_size_ = 32768; // 4096 샘플 뒤를 zero-padding해서 FFT peak 주파수를 더 촘촘하게 고른다.
    

    std::size_t sampling_rate_ = 96000;//샘플링 주파수
    double min_detection_frequency_hz_ = 10000.0;
    double max_detection_frequency_hz_ = 30000.0;
    std::vector<double> target_frequencies_hz_ = {21164.0, 27211.0};
    double target_search_half_width_hz_ = 100.0;
    double blacklist_frequency_hz_ = 23900.0;
    double blacklist_half_width_hz_ = 500.0;

    //SNR 계산 관련 파라미터
    double min_snr_db_ = 10.0;  // threshold SNR: 10dB
    double noise_floor_half_width_hz_ = 1000.0;  // peak 주변 local noise를 볼 범위
    double noise_guard_half_width_hz_ = 200.0;  // peak와 누설 성분은 noise 계산에서 제외
    double lock_tolerance_hz_ = 50.0;  // 판정할 주파수 후보들 간의 오차 허용 범위: 20Hz
    std::size_t lock_window_count_ = 7;  // 판정할 주파수 후보들의 개수: 7개
    std::size_t min_lock_count_ = 5;  // 7개 후보 중 5개 이상이 같은 대역에 몰리면 lock-on
    bool locked_on_ = false;  // 주파수 판정 결과: true/false    
    double locked_frequency_hz_ = 0.0;  // lock-on된 주파수: 판정할 주파수 후보들의 최빈값
    std::deque<double> candidate_frequencies_hz_;   // 판정할 주파수 후보들을 저장할 큐
    double epsilon_ = 1.0e-12;  // SNR 계산할 때 0으로 나누는 걸 막기 위한 아주 작은 값


    //FFT 계산 관련 파라미터
    Eigen::FFT<double> fft_;
};
} 



// namespace audio_capture
RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::AudioFrequencyDetectorNode)
