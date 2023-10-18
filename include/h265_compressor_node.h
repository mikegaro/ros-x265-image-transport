// This node aims to receive a live feed from a camera as input and publish compressed frames.

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Header.h>

#include "compression/ImagePacket.h"


extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>
}

using ImagePacketConstPtr = compression::ImagePacketConstPtr;
using ImagePacket = compression::ImagePacket;

class H265VideoCompressor
{

typedef std::unique_lock<std::recursive_mutex> Lock;

private:
    // ROS
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber     image_sub_;
    ros::Publisher                  compressed_pub_;
    
    // FFMPEG CONFIGURATIONS
    mutable std::recursive_mutex mutex_;
    std::string       codecName_;
    std::string       preset_;
    std::string       profile_;
    AVPixelFormat     pixFormat_{AV_PIX_FMT_GBRP};
    int               rate_;
    int               GOPSize_;
    std::string       tune_;
    int          bitRate_;
    int               qmax_;
    
    // FFMPEG CLASSES
    AVCodecContext    * codecContext_   {nullptr};
    AVFrame           * frame_          {nullptr};
    AVFrame           * wrapperFrame_   {nullptr};
    SwsContext        * swsContext_     {nullptr};
    AVPacket          * packet_         {nullptr};
    
    // Other
    using PTSMap = std::unordered_map<int64_t, ros::Time>;
    int64_t             pts_{0};
    PTSMap              ptsToStamp_;
public:
    H265VideoCompressor(ros::NodeHandle nh);
    ~H265VideoCompressor();

    bool initialize(const int &width, const int &height);

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    void publishCompressedFrame(const sensor_msgs::ImageConstPtr & img);

    bool openCodec(const int &width, const int &height);

    void closeCodec();

    bool encodeFrameWithFFmpeg(const cv::Mat &img);

    std::vector<enum AVPixelFormat> get_encoder_formats(const AVCodec * c);

    enum AVPixelFormat get_preferred_pixel_format(const std::string & encoder, const std::vector<AVPixelFormat> & fmts);

};