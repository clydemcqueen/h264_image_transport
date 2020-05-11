#include "h264_msgs/msg/packet.hpp"
#include "rclcpp/rclcpp.hpp"

extern "C" {
#include <libavutil/imgutils.h>
#include <libavdevice/avdevice.h>
}

namespace h264_image_transport
{

  constexpr int QUEUE_SIZE = 10;

  class V4LCamNode : public rclcpp::Node
  {
    struct Parameters
    {
      std::string input_fn_;
      std::string fps_;
      std::string size_;
      std::string frame_id_;
    };

    Parameters parameters_;
    AVInputFormat *input_format_{nullptr};
    AVFormatContext *format_context_{nullptr};
    std::thread cam_thread_;
    std::atomic<bool> stop_signal_{false};
    rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr h264_pub_;

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

      h264_pub_ = create_publisher<h264_msgs::msg::Packet>("image_raw/h264", QUEUE_SIZE);

      // Get parameters
      parameters_.input_fn_ = declare_parameter("input_fn", "/dev/video2");
      parameters_.fps_ = declare_parameter("fps", "30");
      parameters_.size_ = declare_parameter("size", "800x600");
      parameters_.frame_id_ = declare_parameter("frame_id", "camera_frame");

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
            parameters_.fps_ = parameter.get_value<std::string>();
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

      if (cam_thread_.joinable()) {
        stop_signal_ = true;
        cam_thread_.join();
        stop_signal_ = false;
      }

      // Set format options, this will allocate an AVDictionary
      AVDictionary *format_options = nullptr;
      av_dict_set(&format_options, "input_format", "h264", 0);
      av_dict_set(&format_options, "framerate", parameters_.fps_.c_str(), 0);
      av_dict_set(&format_options, "video_size", parameters_.size_.c_str(), 0);

      // Open 4vl device, pass ownership of format_options
      if (avformat_open_input(&format_context_, parameters_.input_fn_.c_str(), input_format_, &format_options) < 0) {
        RCLCPP_ERROR_STREAM(get_logger(), "Could not open the v4l device: " << parameters_.input_fn_);
        throw std::runtime_error("Could not open the v4l device");
      }

      cam_thread_ = std::thread(
        [this]()
        {
          RCLCPP_INFO(get_logger(), "Camera thread started");

          while (!stop_signal_ && rclcpp::ok()) {
            // Block until a frame is ready
            AVPacket packet;
            if (av_read_frame(format_context_, &packet) < 0) {
              RCLCPP_INFO(get_logger(), "EOS");
              break;
            }

            // Publish h264 message
            h264_msgs::msg::Packet h264_msg;
            h264_msg.data.insert(h264_msg.data.end(), &packet.data[0], &packet.data[packet.size]);
            h264_msg.header.stamp = now();
            h264_msg.header.frame_id = parameters_.frame_id_;
            h264_pub_->publish(h264_msg);

            // Free packet
            av_packet_unref(&packet);
          }

          // Close v4l device
          avformat_close_input(&format_context_);

          RCLCPP_INFO(get_logger(), "Camera thread stopped");
        });
    }

    ~V4LCamNode() override
    {
      avformat_free_context(format_context_);
      if (cam_thread_.joinable()) {
        stop_signal_ = true;
        cam_thread_.join();
      }
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
