#include "h265_decoder_node.h"

VideoDecompressionNode::VideoDecompressionNode() : it_(nh_)
{
    int img_width, img_height;
    nh_.getParam("img_width", img_width);
    nh_.getParam("img_height", img_height);
    initializeDecoder("hevc", img_width, img_height);

    // Subscribe to the compressed video topic
    compressed_image_sub_ = nh_.subscribe("compressed_video", 1, &VideoDecompressionNode::compressedImageCallback, this);

    image_pub_ = it_.advertise("decompressed_video", 1);
}

VideoDecompressionNode::~VideoDecompressionNode()
{
}

bool VideoDecompressionNode::initializeDecoder(const std::string &codec, const int &width, const int &height)
{
    try
    {
        // Set codec
        codec_ = avcodec_find_decoder_by_name(codec.c_str());
        if (!codec_)
        {
            ROS_ERROR("Cannot find codec %s", codec.c_str());
            throw(std::runtime_error("cannot find codec"));
        }

        // Set context for codec_
        codecContext_ = avcodec_alloc_context3(codec_);
        if (!codecContext_)
        {
            ROS_ERROR("alloc context failed for %s", codec.c_str());
            codec_ = nullptr;
            throw(std::runtime_error("alloc context failed!"));
        }

        // Set refcounted frames
        av_opt_set_int(codecContext_, "refcounted_frames", 1, 0);

        // Fill the context
        codecContext_->width = width;
        codecContext_->height = height;
        codecContext_->pkt_timebase = timeBase_;

        // Open codec with the specified context
        if (avcodec_open2(codecContext_, codec_, nullptr) < 0)
        {
            ROS_DEBUG("open context failed for %s", codec.c_str());
            av_free(codecContext_);
            codecContext_ = nullptr;
            codec_ = nullptr;
            throw(std::runtime_error("open context failed!"));
        }

        // Here we will receive the uncompressed image
        decodedFrame_ = av_frame_alloc();

        // Allocate enough memory for the colorFrame
        colorFrame_ = av_frame_alloc();
        colorFrame_->width = width;
        colorFrame_->height = height;
        colorFrame_->format = AV_PIX_FMT_BGR24;
    }
    catch (const std::runtime_error &e)
    {
        ROS_ERROR("%s", e.what());
        return (false);
    }
    ROS_DEBUG("CODEC INITIALIZED SUCCESSFULLY");
    return (true);
}

void VideoDecompressionNode::compressedImageCallback(const ImagePacketConstPtr &msg)
{
    try
    {
        if (decodePacket(msg))
        {
            publishUncompressedImage();
        }
    }
    catch (cv::Exception &e)
    {
        ROS_ERROR("Error decoding compressed image: %s", e.what());
    }
}

bool VideoDecompressionNode::decodePacket(const ImagePacketConstPtr &msg)
{
    // This is only to use a shorter name down below
    AVCodecContext *ctx = codecContext_;

    // Allocate memory to save temporarily the packet
    AVPacket *packet = av_packet_alloc();
    av_new_packet(packet, msg->data.size()); // This will add some padding
    memcpy(packet->data, &msg->data[0], msg->data.size());
    packet->pts = msg->pts;
    packet->dts = packet->pts;
    pts_ = (pts_ % timeBase_.den) + 1;
    ptsToStamp_[packet->pts] = msg->header.stamp;

    ROS_DEBUG("PACKET SIZE %d", packet->size);

    // Send packet to the codec
    int ret = avcodec_send_packet(ctx, packet);
    switch (ret)
    {
    case 0:
    {
        ROS_DEBUG("---- FRAME SENT SUCCESSFULLY -----");
        ret = avcodec_receive_frame(ctx, decodedFrame_);
        ROS_DEBUG("---- avcodec_receive_frame returned %d  -----", ret);
        const AVFrame *frame = decodedFrame_;

        if (ret == 0 && frame->width != 0)
        {
            // convert frame into something
            if (!swsContext_)
            {
                ROS_DEBUG("---- converting frame with sws_getContext()  -----");
                swsContext_ = sws_getContext(
                    ctx->width, ctx->height, (AVPixelFormat)frame->format,       // src
                    ctx->width, ctx->height, (AVPixelFormat)colorFrame_->format, // dest
                    SWS_FAST_BILINEAR | SWS_ACCURATE_RND, nullptr, nullptr, nullptr);
                if (!swsContext_)
                {
                    ROS_ERROR("cannot allocate sws context!!!!");
                    return (false);
                }
                ROS_DEBUG("---- SUCCESS  -----");
            }
        }
        if(ret < 0)
        {
            pts_--;
            av_packet_free(&packet);
            return (false);
        }
        break;
    }
    case AVERROR(EAGAIN):
        ROS_ERROR("avcodec_send_packet() returned AVERROR(EAGAIN)");
        av_packet_free(&packet);
        return (false);
    case AVERROR_EOF:
        ROS_ERROR("avcodec_send_packet() returned AVERROR_EOF");
        av_packet_free(&packet);
        return false;
    case AVERROR(EINVAL):
        ROS_ERROR("avcodec_send_packet() returned AVERROR(EINVAL)");
        av_packet_free(&packet);
        return false;
    case AVERROR(ENOMEM):
        ROS_ERROR("avcodec_send_packet() returned (ENOMEM)");
        av_packet_free(&packet);
        return false;
    default:
        ROS_ERROR("avcodec_send_packet() returned %d", ret);
        av_packet_free(&packet);
        return false;
    }
    av_packet_free(&packet);
    return (true);
}

void VideoDecompressionNode::publishUncompressedImage()
{
    const AVCodecContext *ctx = codecContext_;
    const AVFrame *frame = decodedFrame_;
    ROS_DEBUG("---- Preparing frame to publish  -----");
    sensor_msgs::ImagePtr image(new sensor_msgs::Image());
    image->height = frame->height;
    image->width = frame->width;
    image->step = image->width * 3; // 3 bytes per pixel
    image->encoding = sensor_msgs::image_encodings::BGR8;
    image->data.resize(image->step * image->height);
    ROS_DEBUG("---- Preparing frame to publish  -----");

    // bend the memory pointers in colorFrame to the right locations
    ROS_DEBUG("---- av_image_fill_arrays()  -----");
    av_image_fill_arrays(colorFrame_->data, colorFrame_->linesize, &(image->data[0]),
                         (AVPixelFormat)colorFrame_->format,
                         colorFrame_->width, colorFrame_->height, 1);
    ROS_DEBUG("---- SUCCESS  -----");

    // Rescale the frame to get the final uncompressed image
    ROS_DEBUG("---- Preparing sws_scale -----");
    sws_scale(swsContext_, frame->data, frame->linesize, 0,           // src
              ctx->height, colorFrame_->data, colorFrame_->linesize); // dest

    ROS_DEBUG("---- SUCCESS  -----");
    image_pub_.publish(image);
}

int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "image_decompression_node");
    VideoDecompressionNode node;
    ros::spin();
    return 0;
}