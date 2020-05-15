#include "h264_image_transport/h264_subscriber.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace h264_image_transport
{

  H264Subscriber::H264Subscriber() :
    logger_(rclcpp::get_logger("H264Subscriber")),
    seq_(-1),
    p_codec_(),
    p_codec_context_(),
    p_frame_(),
    packet_(),
    p_sws_context_()
  {}

  H264Subscriber::~H264Subscriber()
  {
    avcodec_close(p_codec_context_);
    av_free(p_codec_context_);
    av_frame_free(&p_frame_);
  }

  void H264Subscriber::subscribeImpl(rclcpp::Node *node, const std::string &base_topic, const Callback &callback,
                                     rmw_qos_profile_t custom_qos)
  {
    SimpleSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos);

    logger_ = node->get_logger();
    av_init_packet(&packet_);
    avcodec_register_all();
    av_log_set_level(AV_LOG_WARNING);

    p_codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!p_codec_) {
      RCLCPP_ERROR(logger_, "Could not find ffmpeg h264 codec");
      throw std::runtime_error("Could not find ffmpeg h264 codec");
    }

    p_codec_context_ = avcodec_alloc_context3(p_codec_);

    if (avcodec_open2(p_codec_context_, p_codec_, nullptr) < 0) {
      RCLCPP_ERROR(logger_, "Could not open ffmpeg h264 codec");
      throw std::runtime_error("Could not open ffmpeg h264 codec");
    }

    p_frame_ = av_frame_alloc();
  }

  void H264Subscriber::internalCallback(const h264_msgs::msg::Packet::ConstSharedPtr &message,
                                        const Callback &user_cb)
  {
    if (seq_ < 0) {
      RCLCPP_INFO(logger_, "First message: %d", message->seq);
      std::cout << std::flush;
    } else {
      if (message->seq < seq_) {
        RCLCPP_INFO(logger_, "Drop old message: %d", seq_);
        std::cout << std::flush;
        return;
      }
      if (message->seq == seq_) {
        RCLCPP_INFO(logger_, "Drop repeat message: %d", seq_);
        std::cout << std::flush;
        return;
      }
      if (message->seq > ++seq_) {
        RCLCPP_INFO(logger_, "Missing message(s): %d-%d", seq_, message->seq - 1);
        std::cout << std::flush;
      }
    }
    seq_ = message->seq;

    packet_.size = message->data.size();
    packet_.data = (uint8_t *) &message->data[0];

    // Send packet to decoder
    if (avcodec_send_packet(p_codec_context_, &packet_) < 0) {
      RCLCPP_INFO(logger_, "Could not send packet");
      std::cout << std::flush;
      return;
    }

    // Get decoded frame
    if (avcodec_receive_frame(p_codec_context_, p_frame_) < 0) {
      RCLCPP_INFO(logger_, "Could not receive frame");
      std::cout << std::flush;
      return;
    }

    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->width = p_frame_->width;
    image->height = p_frame_->height;
    image->step = 3 * p_frame_->width;
    image->encoding = sensor_msgs::image_encodings::BGR8;
    image->header = message->header;

    // Set / update sws context
    p_sws_context_ = sws_getCachedContext(p_sws_context_, p_frame_->width, p_frame_->height, AV_PIX_FMT_YUV420P,
                                          p_frame_->width, p_frame_->height, AV_PIX_FMT_BGR24,
                                          SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

    // Copy and convert from YUYV420P to BGR24
    image->data.resize(p_frame_->width * p_frame_->height * 3);
    int stride = 3 * p_frame_->width;
    uint8_t *destination = &image->data[0];
    sws_scale(p_sws_context_, (const uint8_t *const *) p_frame_->data, p_frame_->linesize, 0,
              p_frame_->height, &destination, &stride);

    user_cb(image);
  }

}
