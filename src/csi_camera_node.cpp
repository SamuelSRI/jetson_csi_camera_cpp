#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

class JetsonCsiCameraNode : public rclcpp::Node
{
public:
  JetsonCsiCameraNode()
  : Node("jetson_csi_camera"), fps_window_start_(std::chrono::steady_clock::now())
  {
    declare_parameters();
    read_parameters();

    const auto qos = make_qos();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);

    gst_init(nullptr, nullptr);
    pipeline_description_ = build_pipeline();
    RCLCPP_INFO(get_logger(), "Opening pipeline: %s", pipeline_description_.c_str());

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description_.c_str(), &error);
    if (pipeline_ == nullptr) {
      const std::string message = error != nullptr ? error->message : "unknown GStreamer error";
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("Unable to create GStreamer pipeline: " + message);
    }

    GstElement * sink_element = gst_bin_get_by_name(GST_BIN(pipeline_), "camera_sink");
    if (sink_element == nullptr || !GST_IS_APP_SINK(sink_element)) {
      if (sink_element != nullptr) {
        gst_object_unref(sink_element);
      }
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      throw std::runtime_error("Unable to find GStreamer appsink named 'camera_sink'.");
    }

    appsink_ = GST_APP_SINK(sink_element);
    gst_app_sink_set_emit_signals(appsink_, true);
    gst_app_sink_set_drop(appsink_, true);
    gst_app_sink_set_max_buffers(appsink_, 1);

    new_sample_handler_id_ = g_signal_connect(
      appsink_, "new-sample", G_CALLBACK(&JetsonCsiCameraNode::on_new_sample_static), this);

    const GstStateChangeReturn state_result = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
      shutdown_pipeline();
      throw std::runtime_error(
              "Unable to start CSI camera pipeline. Check nvargus-daemon, sensor_id and camera mode.");
    }

    bus_ = gst_element_get_bus(pipeline_);
    bus_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&JetsonCsiCameraNode::check_bus, this));

    RCLCPP_INFO(
      get_logger(),
      "Jetson CSI camera started: sensor_id=%d, capture=%dx%d@%d FPS, output=%dx%d, QoS=%s",
      sensor_id_, capture_width_, capture_height_, framerate_, output_width_, output_height_,
      use_sensor_data_qos_ ? "BEST_EFFORT" : "RELIABLE");
  }

  ~JetsonCsiCameraNode() override
  {
    shutdown_pipeline();
  }

private:
  void declare_parameters()
  {
    declare_parameter<int>("sensor_id", 0);
    declare_parameter<int>("capture_width", 1640);
    declare_parameter<int>("capture_height", 1232);
    declare_parameter<int>("output_width", 640);
    declare_parameter<int>("output_height", 480);
    declare_parameter<int>("framerate", 30);
    declare_parameter<int>("flip_method", 0);
    declare_parameter<std::string>("frame_id", "camera_front_link");
    declare_parameter<std::string>("camera_name", "camera_front");
    declare_parameter<std::string>("image_topic", "image_raw");
    declare_parameter<std::string>("camera_info_topic", "camera_info");
    declare_parameter<bool>("use_sensor_data_qos", true);
    declare_parameter<int>("qos_depth", 1);
    declare_parameter<double>("fps_log_period", 2.0);
  }

  void read_parameters()
  {
    sensor_id_ = static_cast<int>(get_parameter("sensor_id").as_int());
    capture_width_ = static_cast<int>(get_parameter("capture_width").as_int());
    capture_height_ = static_cast<int>(get_parameter("capture_height").as_int());
    output_width_ = static_cast<int>(get_parameter("output_width").as_int());
    output_height_ = static_cast<int>(get_parameter("output_height").as_int());
    framerate_ = static_cast<int>(get_parameter("framerate").as_int());
    flip_method_ = static_cast<int>(get_parameter("flip_method").as_int());
    frame_id_ = get_parameter("frame_id").as_string();
    camera_name_ = get_parameter("camera_name").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    use_sensor_data_qos_ = get_parameter("use_sensor_data_qos").as_bool();
    qos_depth_ = static_cast<int>(get_parameter("qos_depth").as_int());
    fps_log_period_ = get_parameter("fps_log_period").as_double();

    if (capture_width_ <= 0 || capture_height_ <= 0 || output_width_ <= 0 ||
      output_height_ <= 0 || framerate_ <= 0 || qos_depth_ <= 0 || fps_log_period_ <= 0.0)
    {
      throw std::invalid_argument("Camera dimensions, framerate, QoS depth and FPS period must be positive.");
    }

    if (flip_method_ < 0 || flip_method_ > 7) {
      throw std::invalid_argument("flip_method must be between 0 and 7.");
    }
  }

  rclcpp::QoS make_qos() const
  {
    if (use_sensor_data_qos_) {
      return rclcpp::SensorDataQoS().keep_last(static_cast<std::size_t>(qos_depth_));
    }

    return rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(qos_depth_)))
      .reliable()
      .durability_volatile();
  }

  std::string build_pipeline() const
  {
    return
      "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id_) + " ! "
      "video/x-raw(memory:NVMM),"
      "width=(int)" + std::to_string(capture_width_) + ","
      "height=(int)" + std::to_string(capture_height_) + ","
      "format=(string)NV12,"
      "framerate=(fraction)" + std::to_string(framerate_) + "/1 ! "
      "nvvidconv flip-method=" + std::to_string(flip_method_) + " ! "
      "video/x-raw,"
      "width=(int)" + std::to_string(output_width_) + ","
      "height=(int)" + std::to_string(output_height_) + ","
      "format=(string)BGRx ! "
      "videoconvert n-threads=2 ! "
      "video/x-raw,format=(string)BGR ! "
      "appsink name=camera_sink emit-signals=true drop=true max-buffers=1 sync=false";
  }

  static GstFlowReturn on_new_sample_static(GstAppSink * sink, gpointer user_data)
  {
    auto * node = static_cast<JetsonCsiCameraNode *>(user_data);
    return node->on_new_sample(sink);
  }

  GstFlowReturn on_new_sample(GstAppSink * sink)
  {
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (sample == nullptr) {
      return GST_FLOW_ERROR;
    }

    GstCaps * caps = gst_sample_get_caps(sample);
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (caps == nullptr || buffer == nullptr) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    GstVideoInfo video_info;
    if (!gst_video_info_from_caps(&video_info, caps)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unable to parse GStreamer video caps.");
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    GstVideoFrame video_frame;
    if (!gst_video_frame_map(&video_frame, &video_info, buffer, GST_MAP_READ)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unable to map GStreamer video frame.");
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    const auto width = static_cast<uint32_t>(GST_VIDEO_FRAME_WIDTH(&video_frame));
    const auto height = static_cast<uint32_t>(GST_VIDEO_FRAME_HEIGHT(&video_frame));
    constexpr uint32_t bytes_per_pixel = 3;
    const uint32_t row_bytes = width * bytes_per_pixel;
    const int source_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&video_frame, 0);
    const auto * source = static_cast<const uint8_t *>(
      GST_VIDEO_FRAME_PLANE_DATA(&video_frame, 0));

    auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
    image_msg->header.stamp = now();
    image_msg->header.frame_id = frame_id_;
    image_msg->height = height;
    image_msg->width = width;
    image_msg->encoding = "bgr8";
    image_msg->is_bigendian = false;
    image_msg->step = row_bytes;
    image_msg->data.resize(static_cast<std::size_t>(row_bytes) * height);

    if (source_stride == static_cast<int>(row_bytes)) {
      std::memcpy(image_msg->data.data(), source, image_msg->data.size());
    } else {
      for (uint32_t row = 0; row < height; ++row) {
        const auto * source_row = source + static_cast<std::ptrdiff_t>(row) * source_stride;
        auto * destination_row = image_msg->data.data() + static_cast<std::size_t>(row) * row_bytes;
        std::memcpy(destination_row, source_row, row_bytes);
      }
    }

    const auto stamp = image_msg->header.stamp;

    auto camera_info = sensor_msgs::msg::CameraInfo();
    camera_info.header.stamp = stamp;
    camera_info.header.frame_id = frame_id_;
    camera_info.width = width;
    camera_info.height = height;

    gst_video_frame_unmap(&video_frame);
    gst_sample_unref(sample);

    image_pub_->publish(std::move(image_msg));
    camera_info_pub_->publish(camera_info);
    update_fps_counter();

    return GST_FLOW_OK;
  }

  void update_fps_counter()
  {
    std::lock_guard<std::mutex> lock(fps_mutex_);
    ++frame_count_;

    const auto current_time = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(current_time - fps_window_start_).count();
    if (elapsed >= fps_log_period_) {
      const double fps = static_cast<double>(frame_count_) / elapsed;
      RCLCPP_INFO(get_logger(), "Published camera rate: %.2f FPS", fps);
      frame_count_ = 0;
      fps_window_start_ = current_time;
    }
  }

  void check_bus()
  {
    if (bus_ == nullptr) {
      return;
    }

    while (GstMessage * message = gst_bus_pop_filtered(
        bus_, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_EOS)))
    {
      switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR:
        {
          GError * error = nullptr;
          gchar * debug = nullptr;
          gst_message_parse_error(message, &error, &debug);
          RCLCPP_ERROR(
            get_logger(), "GStreamer error from %s: %s%s%s",
            GST_OBJECT_NAME(message->src),
            error != nullptr ? error->message : "unknown error",
            debug != nullptr ? " | " : "",
            debug != nullptr ? debug : "");
          if (error != nullptr) {
            g_error_free(error);
          }
          if (debug != nullptr) {
            g_free(debug);
          }
          break;
        }
        case GST_MESSAGE_WARNING:
        {
          GError * error = nullptr;
          gchar * debug = nullptr;
          gst_message_parse_warning(message, &error, &debug);
          RCLCPP_WARN(
            get_logger(), "GStreamer warning from %s: %s%s%s",
            GST_OBJECT_NAME(message->src),
            error != nullptr ? error->message : "unknown warning",
            debug != nullptr ? " | " : "",
            debug != nullptr ? debug : "");
          if (error != nullptr) {
            g_error_free(error);
          }
          if (debug != nullptr) {
            g_free(debug);
          }
          break;
        }
        case GST_MESSAGE_EOS:
          RCLCPP_WARN(get_logger(), "GStreamer pipeline reached end of stream.");
          break;
        default:
          break;
      }
      gst_message_unref(message);
    }
  }

  void shutdown_pipeline()
  {
    if (bus_timer_) {
      bus_timer_->cancel();
      bus_timer_.reset();
    }

    if (appsink_ != nullptr && new_sample_handler_id_ != 0U) {
      g_signal_handler_disconnect(appsink_, new_sample_handler_id_);
      new_sample_handler_id_ = 0U;
    }

    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }

    if (bus_ != nullptr) {
      gst_object_unref(bus_);
      bus_ = nullptr;
    }

    if (appsink_ != nullptr) {
      gst_object_unref(appsink_);
      appsink_ = nullptr;
    }

    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  int sensor_id_{};
  int capture_width_{};
  int capture_height_{};
  int output_width_{};
  int output_height_{};
  int framerate_{};
  int flip_method_{};
  int qos_depth_{};
  bool use_sensor_data_qos_{};
  double fps_log_period_{};

  std::string frame_id_;
  std::string camera_name_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string pipeline_description_;

  GstElement * pipeline_{nullptr};
  GstAppSink * appsink_{nullptr};
  GstBus * bus_{nullptr};
  gulong new_sample_handler_id_{0U};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr bus_timer_;

  std::mutex fps_mutex_;
  std::size_t frame_count_{0};
  std::chrono::steady_clock::time_point fps_window_start_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<JetsonCsiCameraNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("jetson_csi_camera"), "%s", error.what());
  }

  rclcpp::shutdown();
  return 0;
}
