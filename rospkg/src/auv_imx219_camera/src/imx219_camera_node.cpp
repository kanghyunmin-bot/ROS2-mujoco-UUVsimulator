// Copyright 2026 AUV Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "capture_stamp_selector.hpp"

namespace auv_imx219_camera
{

using namespace std::chrono_literals;

class Imx219CameraNode : public rclcpp::Node
{
public:
  explicit Imx219CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("imx219_camera", options)
  {
    initialize_gstreamer();

    sensor_id_ = declare_parameter<int>("sensor_id", 0);
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 720);
    framerate_ = declare_parameter<int>("framerate", 30);
    flip_method_ = declare_parameter<int>("flip_method", 0);
    frame_id_ = declare_parameter<std::string>("frame_id", "imx219_camera_optical_frame");
    image_topic_ = declare_parameter<std::string>("image_topic", "image_raw");
    pipeline_override_ = declare_parameter<std::string>("pipeline", "");
    capture_timeout_ms_ = declare_parameter<int>("capture_timeout_ms", 1000);
    timestamp_source_ =
      declare_parameter<std::string>("timestamp_source", "gstreamer_pts");
    max_capture_age_ms_ = declare_parameter<int>("max_capture_age_ms", 2000);

    distortion_model_ = declare_parameter<std::string>("distortion_model", "plumb_bob");
    distortion_coefficients_ = declare_parameter<std::vector<double>>(
      "distortion_coefficients", std::vector<double>(5, 0.0));
    camera_matrix_ = declare_parameter<std::vector<double>>(
      "camera_matrix", std::vector<double>(9, 0.0));
    rectification_matrix_ = declare_parameter<std::vector<double>>(
      "rectification_matrix",
      std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    projection_matrix_ = declare_parameter<std::vector<double>>(
      "projection_matrix", std::vector<double>(12, 0.0));

    validate_parameters();
    capture_stamp_selector_.emplace(
      std::max<std::int64_t>(1, 1'000'000'000LL / static_cast<std::int64_t>(framerate_)));
    initialize_camera_info();

    camera_publisher_ = image_transport::create_camera_publisher(
      this, image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());

    pipeline_description_ = pipeline_override_.empty() ? build_pipeline() : pipeline_override_;
    open_pipeline();

    running_.store(true);
    capture_thread_ = std::thread(&Imx219CameraNode::capture_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "IMX219 sensor-id=%d started: %dx%d@%d, image=%s, camera_info=%s, frame_id=%s",
      sensor_id_, width_, height_, framerate_, camera_publisher_.getTopic().c_str(),
      camera_publisher_.getInfoTopic().c_str(), frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "GStreamer pipeline: %s", pipeline_description_.c_str());
  }

  ~Imx219CameraNode() override
  {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    close_pipeline();
  }

private:
  static void initialize_gstreamer()
  {
    static std::once_flag init_flag;
    std::call_once(init_flag, []() {gst_init(nullptr, nullptr);});
  }

  void validate_parameters() const
  {
    if (sensor_id_ < 0) {
      throw std::invalid_argument("sensor_id must be zero or greater");
    }
    if (width_ <= 0 || height_ <= 0 || framerate_ <= 0) {
      throw std::invalid_argument("width, height and framerate must be greater than zero");
    }
    if (flip_method_ < 0 || flip_method_ > 7) {
      throw std::invalid_argument("flip_method must be between 0 and 7");
    }
    if (capture_timeout_ms_ < 100) {
      throw std::invalid_argument("capture_timeout_ms must be at least 100");
    }
    if (timestamp_source_ != "gstreamer_pts" && timestamp_source_ != "ros_now") {
      throw std::invalid_argument("timestamp_source must be gstreamer_pts or ros_now");
    }
    if (max_capture_age_ms_ <= 0) {
      throw std::invalid_argument("max_capture_age_ms must be greater than zero");
    }
    if (frame_id_.empty() || image_topic_.empty()) {
      throw std::invalid_argument("frame_id and image_topic must not be empty");
    }
    if (camera_matrix_.size() != 9) {
      throw std::invalid_argument("camera_matrix must contain 9 values");
    }
    if (rectification_matrix_.size() != 9) {
      throw std::invalid_argument("rectification_matrix must contain 9 values");
    }
    if (projection_matrix_.size() != 12) {
      throw std::invalid_argument("projection_matrix must contain 12 values");
    }
  }

  void initialize_camera_info()
  {
    camera_info_.width = static_cast<uint32_t>(width_);
    camera_info_.height = static_cast<uint32_t>(height_);
    camera_info_.distortion_model = distortion_model_;
    camera_info_.d = distortion_coefficients_;
    std::copy(camera_matrix_.begin(), camera_matrix_.end(), camera_info_.k.begin());
    std::copy(
      rectification_matrix_.begin(), rectification_matrix_.end(), camera_info_.r.begin());
    std::copy(projection_matrix_.begin(), projection_matrix_.end(), camera_info_.p.begin());
  }

  std::string build_pipeline() const
  {
    std::ostringstream pipeline;
    pipeline
      << "nvarguscamerasrc sensor-id=" << sensor_id_
      << " ! video/x-raw(memory:NVMM),width=(int)" << width_
      << ",height=(int)" << height_
      << ",format=(string)NV12,framerate=(fraction)" << framerate_ << "/1"
      << " ! nvvidconv flip-method=" << flip_method_
      << " ! video/x-raw,width=(int)" << width_
      << ",height=(int)" << height_ << ",format=(string)BGRx"
      << " ! videoconvert"
      << " ! video/x-raw,format=(string)BGR"
      << " ! appsink name=imx219_sink emit-signals=false sync=false max-buffers=1 drop=true";
    return pipeline.str();
  }

  void open_pipeline()
  {
    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description_.c_str(), &error);
    if (pipeline_ == nullptr) {
      const std::string message = error != nullptr ? error->message : "unknown parse error";
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("Failed to create GStreamer pipeline: " + message);
    }
    if (error != nullptr) {
      const std::string message = error->message;
      g_error_free(error);
      close_pipeline();
      throw std::runtime_error("GStreamer pipeline parse warning: " + message);
    }

    GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline_), "imx219_sink");
    if (sink == nullptr || !GST_IS_APP_SINK(sink)) {
      if (sink != nullptr) {
        gst_object_unref(sink);
      }
      close_pipeline();
      throw std::runtime_error("Pipeline must contain an appsink named 'imx219_sink'");
    }
    app_sink_ = GST_APP_SINK(sink);
    bus_ = gst_element_get_bus(pipeline_);

    const GstStateChangeReturn state_result = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
      const std::string error_message = consume_bus_error();
      close_pipeline();
      throw std::runtime_error(
              "Failed to start GStreamer pipeline" +
              (error_message.empty() ? std::string() : ": " + error_message));
    }
  }

  void close_pipeline()
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (bus_ != nullptr) {
      gst_object_unref(bus_);
      bus_ = nullptr;
    }
    if (app_sink_ != nullptr) {
      gst_object_unref(app_sink_);
      app_sink_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  std::string consume_bus_error()
  {
    if (bus_ == nullptr) {
      return {};
    }

    GstMessage * message = gst_bus_pop_filtered(
      bus_, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (message == nullptr) {
      return {};
    }

    std::string result;
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError * error = nullptr;
      gchar * debug_info = nullptr;
      gst_message_parse_error(message, &error, &debug_info);
      result = error != nullptr ? error->message : "unknown GStreamer error";
      if (debug_info != nullptr) {
        result += " (" + std::string(debug_info) + ")";
        g_free(debug_info);
      }
      if (error != nullptr) {
        g_error_free(error);
      }
    } else {
      result = "GStreamer pipeline reached end of stream";
    }
    gst_message_unref(message);
    return result;
  }

  void capture_loop()
  {
    const GstClockTime timeout = static_cast<GstClockTime>(capture_timeout_ms_) * GST_MSECOND;
    int consecutive_timeouts = 0;

    while (running_.load() && rclcpp::ok()) {
      GstSample * sample = gst_app_sink_try_pull_sample(app_sink_, timeout);
      if (sample == nullptr) {
        const std::string error_message = consume_bus_error();
        if (!error_message.empty()) {
          RCLCPP_ERROR(get_logger(), "IMX219 capture stopped: %s", error_message.c_str());
          running_.store(false);
          rclcpp::shutdown();
          return;
        }

        ++consecutive_timeouts;
        if (consecutive_timeouts == 3 || consecutive_timeouts % 10 == 0) {
          RCLCPP_WARN(
            get_logger(), "No frame from IMX219 sensor-id=%d for %d consecutive timeouts",
            sensor_id_, consecutive_timeouts);
        }
        continue;
      }

      consecutive_timeouts = 0;
      publish_sample(sample);
      gst_sample_unref(sample);
    }
  }

  void publish_sample(GstSample * sample)
  {
    GstCaps * caps = gst_sample_get_caps(sample);
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstVideoInfo video_info;
    if (caps == nullptr || buffer == nullptr || !gst_video_info_from_caps(&video_info, caps)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Received an invalid GStreamer video sample");
      return;
    }

    const int sample_width = static_cast<int>(GST_VIDEO_INFO_WIDTH(&video_info));
    const int sample_height = static_cast<int>(GST_VIDEO_INFO_HEIGHT(&video_info));
    const int source_stride = GST_VIDEO_INFO_PLANE_STRIDE(&video_info, 0);
    const size_t destination_stride = static_cast<size_t>(sample_width) * 3U;
    if (sample_width <= 0 || sample_height <= 0 || source_stride < 0 ||
      static_cast<size_t>(source_stride) < destination_stride)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unexpected BGR frame layout from GStreamer");
      return;
    }

    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to map camera frame");
      return;
    }

    const size_t required_source_size =
      static_cast<size_t>(source_stride) * static_cast<size_t>(sample_height);
    if (map_info.size < required_source_size) {
      gst_buffer_unmap(buffer, &map_info);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Camera frame buffer is smaller than its caps describe");
      return;
    }

    sensor_msgs::msg::Image image;
    image.header.stamp = capture_stamp(sample, buffer);
    image.header.frame_id = frame_id_;
    image.height = static_cast<uint32_t>(sample_height);
    image.width = static_cast<uint32_t>(sample_width);
    image.encoding = sensor_msgs::image_encodings::BGR8;
    image.is_bigendian = false;
    image.step = static_cast<sensor_msgs::msg::Image::_step_type>(destination_stride);
    image.data.resize(destination_stride * static_cast<size_t>(sample_height));

    if (static_cast<size_t>(source_stride) == destination_stride) {
      std::memcpy(image.data.data(), map_info.data, image.data.size());
    } else {
      for (int row = 0; row < sample_height; ++row) {
        std::memcpy(
          image.data.data() + static_cast<size_t>(row) * destination_stride,
          map_info.data + static_cast<size_t>(row) * static_cast<size_t>(source_stride),
          destination_stride);
      }
    }
    gst_buffer_unmap(buffer, &map_info);

    camera_info_.header = image.header;
    camera_info_.width = image.width;
    camera_info_.height = image.height;
    camera_publisher_.publish(image, camera_info_);
  }

  rclcpp::Time capture_stamp(GstSample * sample, GstBuffer * buffer)
  {
    const rclcpp::Time ros_now = now();
    if (timestamp_source_ == "ros_now") {
      return select_valid_stamp(ros_now.nanoseconds(), ros_now, "ROS clock");
    }

    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    const GstSegment * segment = gst_sample_get_segment(sample);
    if (!GST_CLOCK_TIME_IS_VALID(pts) || segment == nullptr || pipeline_ == nullptr) {
      return select_fallback_stamp(ros_now, "missing GStreamer PTS, segment, or pipeline");
    }

    guint64 capture_running_time = GST_CLOCK_TIME_NONE;
    const gint running_time_sign = gst_segment_to_running_time_full(
      segment, GST_FORMAT_TIME, pts, &capture_running_time);
    const GstClockTime current_running_time =
      gst_element_get_current_running_time(pipeline_);
    if (running_time_sign <= 0 || !GST_CLOCK_TIME_IS_VALID(capture_running_time) ||
      !GST_CLOCK_TIME_IS_VALID(current_running_time) ||
      capture_running_time > current_running_time)
    {
      return select_fallback_stamp(ros_now, "invalid or future GStreamer running time");
    }

    const guint64 capture_age_ns = current_running_time - capture_running_time;
    const guint64 maximum_age_ns =
      static_cast<guint64>(max_capture_age_ms_) * GST_MSECOND;
    const int64_t ros_now_ns = ros_now.nanoseconds();
    if (capture_age_ns > maximum_age_ns ||
      capture_age_ns > static_cast<guint64>(std::max<int64_t>(ros_now_ns, 0)))
    {
      return select_fallback_stamp(
        ros_now, "GStreamer capture age is outside the configured bound");
    }

    return select_valid_stamp(
      ros_now_ns - static_cast<int64_t>(capture_age_ns), ros_now, "GStreamer PTS");
  }

  rclcpp::Time select_valid_stamp(
    int64_t candidate_ns, const rclcpp::Time & ros_now, const char * source)
  {
    const CaptureStampSelector::Selection selection =
      capture_stamp_selector_->select_valid(candidate_ns, ros_now.nanoseconds());
    warn_timestamp_correction(selection, source);
    return rclcpp::Time(selection.stamp_ns, ros_now.get_clock_type());
  }

  rclcpp::Time select_fallback_stamp(const rclcpp::Time & ros_now, const char * reason)
  {
    warn_timestamp_fallback(reason);
    const CaptureStampSelector::Selection selection =
      capture_stamp_selector_->select_fallback(ros_now.nanoseconds());
    warn_timestamp_correction(selection, "fallback");
    return rclcpp::Time(selection.stamp_ns, ros_now.get_clock_type());
  }

  void warn_timestamp_correction(
    const CaptureStampSelector::Selection & selection, const char * source)
  {
    if (selection.correction == CaptureStampSelector::Correction::kNone) {
      return;
    }

    const char * policy =
      selection.correction == CaptureStampSelector::Correction::kFramePeriodCapped ?
      "last stamp plus one frame period, capped by ROS time" :
      "last stamp plus 1 ns because ROS time did not advance";
    const std::string correction_count = std::to_string(selection.correction_count);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Corrected non-increasing %s camera stamp using %s (correction_count=%s)",
      source, policy, correction_count.c_str());
  }

  void warn_timestamp_fallback(const char * reason)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Using monotonic fallback for camera stamp: %s", reason);
  }

  int sensor_id_{0};
  int width_{1280};
  int height_{720};
  int framerate_{30};
  int flip_method_{0};
  int capture_timeout_ms_{1000};
  int max_capture_age_ms_{2000};
  std::string frame_id_;
  std::string image_topic_;
  std::string pipeline_override_;
  std::string pipeline_description_;
  std::string timestamp_source_;
  std::string distortion_model_;
  std::vector<double> distortion_coefficients_;
  std::vector<double> camera_matrix_;
  std::vector<double> rectification_matrix_;
  std::vector<double> projection_matrix_;

  image_transport::CameraPublisher camera_publisher_;
  sensor_msgs::msg::CameraInfo camera_info_;
  std::optional<CaptureStampSelector> capture_stamp_selector_;

  GstElement * pipeline_{nullptr};
  GstAppSink * app_sink_{nullptr};
  GstBus * bus_{nullptr};
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
};

}  // namespace auv_imx219_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<auv_imx219_camera::Imx219CameraNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("imx219_camera"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
