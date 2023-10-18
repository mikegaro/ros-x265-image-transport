#pragma once

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "compression/ImagePacket.h"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avio.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
}

using ImagePacketConstPtr = compression::ImagePacketConstPtr;
using ImagePacket = compression::ImagePacket;
class VideoDecompressionNode
{

public:
    VideoDecompressionNode();

    ~VideoDecompressionNode();

    void compressedImageCallback(const ImagePacketConstPtr &msg);

    bool initializeDecoder(const std::string &codec, const int &width, const int &height);

    bool decodePacket(const ImagePacketConstPtr &msg);

    void publishUncompressedImage();

private:
    // ROS
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher      image_pub_;
    ros::Subscriber                 compressed_image_sub_;

    // FFMPEG PARAMETERS FOR CONTEXT
    std::string     pixFormat_;
    const AVCodec   *codec_{nullptr};
    AVRational      timeBase_{1, 10};

    // FFMPEG STRUCTURES
    AVCodecContext  *codecContext_{nullptr};
    AVFrame         *decodedFrame_{nullptr};
    AVFrame         *colorFrame_{nullptr};
    SwsContext      *swsContext_{nullptr};
    uint8_t         pts_{1};
    using PTSMap = std::unordered_map<int64_t, ros::Time>;
    PTSMap ptsToStamp_;
};