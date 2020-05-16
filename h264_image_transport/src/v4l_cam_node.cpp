#include "camera_calibration_parsers/parse.h"
#include "h264_msgs/msg/packet.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

extern "C" {
#include <libavutil/imgutils.h>
#include <libavdevice/avdevice.h>
}

namespace h264_image_transport
{

#undef RUN_PERF
#ifdef RUN_PERF
  #define START_PERF()\
auto __start__ = std::chrono::high_resolution_clock::now();

#define STOP_PERF(msg)\
auto __stop__ = std::chrono::high_resolution_clock::now();\
std::cout << msg << " " << std::chrono::duration_cast<std::chrono::microseconds>(__stop__ - __start__).count()\
  << " microseconds" << std::endl;
#else
#define START_PERF()
#define STOP_PERF(msg)
#endif

  constexpr int QUEUE_SIZE = 10;

  class V4LCamNode : public rclcpp::Node
  {
    struct Parameters
    {
      std::string input_fn_;
      int fps_;
      std::string size_;
      std::string frame_id_;
      std::string camera_info_path_;
    };

    Parameters parameters_;
    int seq_;
    AVInputFormat *input_format_{nullptr};
    AVFormatContext *format_context_{nullptr};
    std::thread cam_thread_;
    std::atomic<bool> stop_signal_{false};

    sensor_msgs::msg::CameraInfo camera_info_msg_;
    rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr h264_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  public:

    V4LCamNode() :
      Node{"v4l_cam_node"}
    {
      av_register_all();
      avdevice_register_all();
      av_log_set_level(AV_LOG_WARNING);

      // Device drivers appear as formats in ffmpeg
      // Find the v4l driver
      AVInputFormat *input_format = av_find_input_format("video4linux2");
      if (!input_format) {
        RCLCPP_ERROR(get_logger(), "Could not find the v4l driver");
        throw std::runtime_error("Could not find the v4l driver");
      }

      // Allocate a format context
      format_context_ = avformat_alloc_context();
      if (!format_context_) {
        RCLCPP_ERROR(get_logger(), "Could not allocate a format context");
        throw std::runtime_error("Could not allocate a format context");
      }

      // Get parameters
      parameters_.input_fn_ = declare_parameter("input_fn", "/dev/video2");
      parameters_.fps_ = declare_parameter("fps", 30);
      parameters_.size_ = declare_parameter("size", "800x600");
      parameters_.frame_id_ = declare_parameter("frame_id", "camera_frame");
      parameters_.camera_info_path_ = declare_parameter("camera_info_path", "info.ini");

      // Start pipeline
      restart();

      // Register parameters
      set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
        auto result = rcl_interfaces::msg::SetParametersResult();
        for (const auto &parameter : parameters) {
          if (parameter.get_name() == "input_fn") {
            parameters_.input_fn_ = parameter.get_value<std::string>();
            result.successful = true;
          }
          if (parameter.get_name() == "fps") {
            parameters_.fps_ = parameter.get_value<int>();
            result.successful = true;
          }
          if (parameter.get_name() == "size") {
            parameters_.size_ = parameter.get_value<std::string>();
            result.successful = true;
          }
          if (parameter.get_name() == "frame_id") {
            parameters_.frame_id_ = parameter.get_value<std::string>();
            result.successful = true;
          }
        }
        restart();
        return result;
      });
    }

    void restart()
    {
      RCLCPP_INFO_STREAM(get_logger(), "Parameter input_fn: " << parameters_.input_fn_);
      RCLCPP_INFO_STREAM(get_logger(), "Parameter fps: " << parameters_.fps_);
      RCLCPP_INFO_STREAM(get_logger(), "Parameter size: " << parameters_.size_);
      RCLCPP_INFO_STREAM(get_logger(), "Parameter frame_id: " << parameters_.frame_id_);
      RCLCPP_INFO_STREAM(get_logger(), "Parameter camera_info_path: " << parameters_.camera_info_path_);

      if (cam_thread_.joinable()) {
        stop_signal_ = true;
        cam_thread_.join();
        stop_signal_ = false;
      }

      // Set format options, this will allocate an AVDictionary
      AVDictionary *format_options = nullptr;
      av_dict_set(&format_options, "input_format", "h264", 0);
      av_dict_set(&format_options, "framerate", std::to_string(parameters_.fps_).c_str(), 0);
      av_dict_set(&format_options, "video_size", parameters_.size_.c_str(), 0);

      // Open 4vl device, pass ownership of format_options
      if (avformat_open_input(&format_context_, parameters_.input_fn_.c_str(), input_format_, &format_options) < 0) {
        RCLCPP_ERROR_STREAM(get_logger(), "Could not open the v4l device: " << parameters_.input_fn_);
        throw std::runtime_error("Could not open the v4l device");
      }

      h264_pub_ = create_publisher<h264_msgs::msg::Packet>("image_raw/h264", QUEUE_SIZE);

      // Parse camera info
      assert(!parameters_.camera_info_path_.empty()); // readCalibration will crash if file_name is ""
      std::string camera_name;
      if (camera_calibration_parsers::readCalibration(parameters_.camera_info_path_, camera_name, camera_info_msg_)) {
        RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", QUEUE_SIZE);
        camera_info_msg_.header.frame_id = parameters_.frame_id_;
      } else {
        RCLCPP_ERROR(get_logger(), "Could not get camera info, will not publish");
        camera_info_pub_ = nullptr;
      }

      cam_thread_ = std::thread(
        [this]()
        {
          RCLCPP_INFO(get_logger(), "Camera thread started");

          while (!stop_signal_ && rclcpp::ok()) {
            h264_msgs::msg::Packet h264_msg;
            h264_msg.header.frame_id = parameters_.frame_id_;
            h264_msg.seq = seq_++;

            // Block until a frame is ready
            AVPacket packet;
            if (av_read_frame(format_context_, &packet) < 0) {
              RCLCPP_INFO(get_logger(), "EOS");
              break;
            }

            START_PERF()

            auto stamp = now();

            // Copy to the ROS message and free the packet
            if (h264_pub_->get_subscription_count() > 0) {
              h264_msg.data.insert(h264_msg.data.end(), &packet.data[0], &packet.data[packet.size]);
              h264_msg.header.stamp = stamp;
              h264_pub_->publish(std::move(h264_msg));
            }

            av_packet_unref(&packet);

            if (camera_info_pub_ && camera_info_pub_->get_subscription_count() > 0) {
              camera_info_msg_.header.stamp = stamp;
              camera_info_pub_->publish(camera_info_msg_);
            }

            STOP_PERF("Copy and publish")
          }

          // Close v4l device
          avformat_close_input(&format_context_);

          RCLCPP_INFO(get_logger(), "Camera thread stopped");
        });
    }

    ~V4LCamNode() override
    {
      if (cam_thread_.joinable()) {
        stop_signal_ = true;
        cam_thread_.join();
      }
      avformat_free_context(format_context_);
    }
  };

} // namespace h264_image_transport

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<h264_image_transport::V4LCamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
