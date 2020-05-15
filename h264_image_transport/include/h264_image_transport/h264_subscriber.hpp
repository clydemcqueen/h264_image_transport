#pragma once

#include "image_transport/simple_subscriber_plugin.h"
#include "h264_msgs/msg/packet.hpp"

extern "C"
{
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"
}

namespace h264_image_transport
{

  class H264Subscriber : public image_transport::SimpleSubscriberPlugin<h264_msgs::msg::Packet>
  {
  private:

    rclcpp::Logger logger_;
    int seq_;
    AVCodec *p_codec_;
    AVCodecContext *p_codec_context_;
    AVFrame *p_frame_;
    AVPacket packet_;
    SwsContext *p_sws_context_;

  protected:

    void internalCallback(const h264_msgs::msg::Packet::ConstSharedPtr &message,
                          const Callback &user_cb) override;

    void subscribeImpl(rclcpp::Node *node, const std::string &base_topic, const Callback &callback,
                       rmw_qos_profile_t custom_qos) override;

  public:

    H264Subscriber();

    ~H264Subscriber() override;

    std::string getTransportName() const override
    {
      return "h264";
    }
  };

}
