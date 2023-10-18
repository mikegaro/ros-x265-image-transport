#include "h265_compressor_node.h"

H265VideoCompressor::H265VideoCompressor(ros::NodeHandle nh) : nh_(nh), it_(nh)
{
  int img_width, img_height;
  nh_.getParam("img_width", img_width);
  nh_.getParam("img_height", img_height);

  if(initialize(img_width, img_height))
  {
    ROS_DEBUG("Decoder initialized succesfully");
  }
  else
  {
    throw std::runtime_error("Could not initiate ffmpeg decoder");
  } // Initialize ffmpeg encoder
  ROS_INFO("On constructor of H265VideoCompressor");

  // Stablish subscriber and the
  image_sub_ = it_.subscribe("/pylon_camera_node/image_raw", 1, &H265VideoCompressor::imageCallback, this);
  compressed_pub_ = nh_.advertise<ImagePacket>("compressed_video", 1);

  ROS_DEBUG("Settin av_init_packet(&packet_);");
  ROS_DEBUG("Constructor finished succesfully");
}

void H265VideoCompressor::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

  ROS_DEBUG("On H265VideoCompressor::imageCallback");
  try
  {
    ROS_DEBUG("On cv_bridge::toCvCopy()");
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    ROS_DEBUG("Raw image resolution = %d x %d", img.rows, img.cols);

    if (encodeFrameWithFFmpeg(img))
    {
      publishCompressedFrame(msg);
    }
    else
    {
      ROS_ERROR("Could not publish frame;");
    }
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("CV_Bridge exception: %s", e.what());
  }
}

void H265VideoCompressor::publishCompressedFrame(const sensor_msgs::ImageConstPtr &img)
{
  ImagePacket compressed_msg;
  const AVPacket &h265_img = *packet_;

  // Fill the message parameters
  compressed_msg.header    = img->header;
  compressed_msg.pixFormat = img->encoding;
  compressed_msg.width     = img->width;
  compressed_msg.height    = img->height;
  compressed_msg.pts       = h265_img.pts;
  compressed_msg.flags     = h265_img.flags;
  
  ptsToStamp_.insert(PTSMap::value_type(frame_->pts, img->header.stamp));
  auto it = ptsToStamp_.find(h265_img.pts);
  if (it != ptsToStamp_.end())
  {
    compressed_msg.header.stamp = it->second;
    ptsToStamp_.erase(it);
  }

  //Fill the message with the packet
  compressed_msg.data.resize(h265_img.size);
  memcpy(&(compressed_msg.data[0]), h265_img.data, h265_img.size);

  //Publish
  compressed_pub_.publish(compressed_msg);
}

bool H265VideoCompressor::initialize(const int & width,const int & height)
{
  Lock lock(mutex_);
  return (openCodec(width, height));
}

bool H265VideoCompressor::openCodec(const int &width,const int &height)
{
  ROS_DEBUG("On H265VideoCompressor::openCodec(int width, int height)");
  codecContext_ = nullptr;
  try
  {
    if ((width % 32) != 0)
    {
      throw(std::runtime_error("horizontal resolution must be"
                               "multiple of 32 but is: " +
                               std::to_string(width)));
    }

    // Find the H265 Codec
    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H265);
    if (!codec)
    {
      throw(std::runtime_error("cannot find video codec: " + codecName_));
    }

    ROS_DEBUG("CODEC FOUND!!! %s", codec->name);
    codecName_ = codec->name;

    // allocate codec context
    codecContext_ = avcodec_alloc_context3(codec);
    if (!codecContext_)
    {
      throw(std::runtime_error("cannot allocate video codec context!"));
    }
    
    if(!nh_.getParam("bitrate", bitRate_))
    {
      throw(std::runtime_error("bitrate not set"));
    }
    codecContext_->bit_rate = bitRate_;

    if(!nh_.getParam("qmax", qmax_))
    {
      throw(std::runtime_error("qmax not set"));
    }
    codecContext_->qmax = qmax_;
    
    if(!nh_.getParam("framerate", rate_))
    {
      throw(std::runtime_error("framerate not set"));
    }
    codecContext_->time_base = {1, rate_};
    codecContext_->framerate = {rate_, 1};

    if(!nh_.getParam("gop_size", GOPSize_))
    {
      throw(std::runtime_error("gop_size not set"));
    }
    codecContext_->gop_size = GOPSize_;

    codecContext_->max_b_frames = 0;

    codecContext_->width = width;
    codecContext_->height = height;

    // Set the pixel format supported by the encoder
    auto pixFmts = get_encoder_formats(codec);
    codecContext_->pix_fmt = get_preferred_pixel_format(codecName_, pixFmts);
    codecContext_->sw_pix_fmt = codecContext_->pix_fmt;

    // Set the profile
    nh_.getParam("profile", profile_);
    if (av_opt_set(codecContext_->priv_data,
                   "profile",
                   profile_.c_str(),
                   AV_OPT_SEARCH_CHILDREN) != 0)
    {
      ROS_ERROR_STREAM("cannot set profile: " << profile_);
      throw(std::runtime_error("Profile not set. Aborting."));
    }

    // Set the preset
    nh_.getParam("preset", preset_);
    if (av_opt_set(codecContext_->priv_data,
                   "preset",
                   preset_.c_str(),
                   AV_OPT_SEARCH_CHILDREN) != 0)
    {
      ROS_ERROR_STREAM("cannot set preset: " << preset_);
    }

    // Set the tune
    nh_.getParam("tune", tune_);
    if (av_opt_set(codecContext_->priv_data,
                   "tune",
                   tune_.c_str(),
                   AV_OPT_SEARCH_CHILDREN) != 0)
    {
      ROS_ERROR("cannot set option %s", tune_.c_str());
    }

    ROS_INFO("codec: %10s, profile: %10s, preset: %10s, bit_rate: %10d qmax: %2d",
              codecName_.c_str(),
              profile_.c_str(),
              preset_.c_str(),
              bitRate_,
              qmax_);

    // Open the codec
    if (avcodec_open2(codecContext_, codec, NULL) < 0)
    {
      throw(std::runtime_error("cannot open codec!"));
    }

    ROS_DEBUG_STREAM("opened codec: " << codecName_);

    // Allocate the frame that will be sent to ffmpegs
    frame_ = av_frame_alloc();
    if (!frame_)
    {
      throw(std::runtime_error("cannot alloc frame!"));
    }
    frame_->width = width;
    frame_->height = height;
    frame_->format = codecContext_->sw_pix_fmt;

    // allocate image for frame
    if (av_image_alloc(frame_->data, frame_->linesize,
                       width, height,
                       static_cast<AVPixelFormat>(frame_->format), 32) < 0)
    {
      throw(std::runtime_error("Cannot allocate image"));
    }

    // Initialize packet for the first time
    packet_ = av_packet_alloc();
    packet_->data = NULL; // Encoder will put the compressed image here
    packet_->size = 0;

    // create (src) frame that wraps the received uncompressed image
    wrapperFrame_ = av_frame_alloc();
    wrapperFrame_->width = width;
    wrapperFrame_->height = height;
    wrapperFrame_->format = AV_PIX_FMT_BGR24;

    // initialize format conversion library
    if (!swsContext_)
    {
      swsContext_ = sws_getContext(width, height, AV_PIX_FMT_BGR24,                           // src
                                   width, height, static_cast<AVPixelFormat>(frame_->format), // dest
                                   SWS_FAST_BILINEAR | SWS_ACCURATE_RND, NULL, NULL, NULL);
      if (!swsContext_)
      {
        throw(std::runtime_error("cannot allocate sws context"));
      }
    }
  }
  catch (const std::runtime_error &e)
  {
    ROS_ERROR_STREAM(e.what());
    if (codecContext_)
    {
      avcodec_close(codecContext_);
      codecContext_ = NULL;
    }
    if (frame_)
    {
      av_free(frame_);
      frame_ = 0;
    }
    return (false);
  }
  ROS_DEBUG_STREAM("intialized codec " << codecName_ << " for image: "
                                       << width << "x" << height);
  ROS_DEBUG("CODEC INITIALIZED SUCCESS");

  return (true);
}

void H265VideoCompressor::closeCodec()
{
  if (codecContext_)
  {
    avcodec_close(codecContext_);
    codecContext_ = nullptr;
  }

  if (frame_)
  {
    av_free(frame_);
    frame_ = 0;
  }

  if (wrapperFrame_)
  {
    av_free(frame_);
    frame_ = 0;
  }

  if (packet_)
  {
    av_packet_free(&packet_); // also unreferences the packet
    packet_ = nullptr;
  }

  if (swsContext_)
  {
    sws_freeContext(swsContext_);
    swsContext_ = NULL;
  }
}

bool H265VideoCompressor::encodeFrameWithFFmpeg(const cv::Mat &img)
{
  Lock lock(mutex_);
  ROS_DEBUG(" ---- encodeFrameWithFFmpeg(const cv::Mat &img) -------");
  ROS_DEBUG("Transforming payload from raw image");
  const int width = img.cols;
  const int height = img.rows;

  ROS_DEBUG("img size: %d x %d", width, height);

  // Prepare the wrapper tto set the memory pointers to the right locations
  av_image_fill_arrays(wrapperFrame_->data, wrapperFrame_->linesize,
                       &(img.data[0]), static_cast<AVPixelFormat>(wrapperFrame_->format),
                       wrapperFrame_->width, wrapperFrame_->height, 1);

  // Using the wrapper to fill the frame that will be sent to the encoder
  sws_scale(swsContext_,
            wrapperFrame_->data, wrapperFrame_->linesize, 0, codecContext_->height,
            frame_->data, frame_->linesize);

  pts_ = (pts_ % codecContext_->framerate.num) + 1;
  frame_->pts=pts_;

  ROS_DEBUG("Frame dimensions: %d x %d", frame_->height, frame_->width);
  ROS_DEBUG("size_t = %ld", sizeof(size_t));
  ROS_DEBUG("Sending frame to x265 API");

  int ret = avcodec_send_frame(codecContext_, frame_);
  switch (ret)
  {
  case 0:
    ROS_DEBUG("---- FRAME SENT SUCCESSFULLY -----");
    while (ret == 0)
    {
      ret = avcodec_receive_packet(codecContext_, packet_);
      switch (ret)
      {
      case 0:
      {
        const AVPacket &pk = *packet_;
        if (pk.size > 0)
        {
          ROS_DEBUG("---- PACKET RECEIVED SUCCESFULLY -----");
          ROS_DEBUG("---- PACKET SIZE: %d -----", pk.size);
        }
        return true;
      }
      case AVERROR(EAGAIN):
        ROS_INFO("New input data is required to return new output");
        return false;
      case AVERROR_EOF:
        ROS_ERROR("avcodec_receive_packet returned AVERROR_EOF");
        return false;
      case AVERROR(EINVAL):
        ROS_ERROR("avcodec_receive_packet returned AVERROR(EINVAL)");
        return false;
      default:
        ROS_ERROR("avcodec_receive_packet returned something else");
        return false;
      }
    }
  case AVERROR(EAGAIN):
    ROS_ERROR("avcodec_send_frame() returned AVERROR(EAGAIN)");
    return false;
  case AVERROR_EOF:
    ROS_ERROR("avcodec_send_frame() returned AVERROR_EOF");
    return false;
  case AVERROR(EINVAL):
    ROS_ERROR("avcodec_send_frame() returned AVERROR(EINVAL)");
    return false;
  case AVERROR(ENOMEM):
    ROS_ERROR("avcodec_send_frame() returned (ENOMEM)");
    return false;
  default:
    ROS_ERROR("avcodec_send_frame() returned something else");
    return false;
  }

  ROS_DEBUG("Finished encodeFrameWithFFmpeg(const cv::Mat &img) --Success--");
  return false;
}

std::vector<enum AVPixelFormat> H265VideoCompressor::get_encoder_formats(const AVCodec *c)
{
  std::vector<enum AVPixelFormat> formats;
  if (c && c->pix_fmts)
  {
    for (const auto *p = c->pix_fmts; *p != AV_PIX_FMT_NONE; ++p)
    {
      formats.push_back(*p);
    }
  }
  return (formats);
}

static bool has_format(const std::vector<AVPixelFormat> &fmts, const AVPixelFormat &f)
{
  return (std::find(fmts.begin(), fmts.end(), f) != fmts.end());
}

enum AVPixelFormat H265VideoCompressor::get_preferred_pixel_format(
    const std::string &encoder, const std::vector<AVPixelFormat> &fmts)
{
  if (has_format(fmts, AV_PIX_FMT_BGR24))
  {
    ROS_DEBUG("Pixel format is BGR24");
    return (AV_PIX_FMT_BGR24); // fastest, needs no copy
  }
  if (has_format(fmts, AV_PIX_FMT_YUV420P))
  {
    ROS_DEBUG("Pixel format is YUV420P");
    return (AV_PIX_FMT_YUV420P); // needs no transformation
  }
  if (has_format(fmts, AV_PIX_FMT_NV12))
  {
    ROS_DEBUG("Pixel format is NV12");
    return (AV_PIX_FMT_NV12); // needs transformation
  }
  return (AV_PIX_FMT_NONE);
}

H265VideoCompressor::~H265VideoCompressor()
{
  Lock lock(mutex_);
  closeCodec();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "video_compressor");
  ros::NodeHandle nh;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  ROS_DEBUG("Debbugging");
  ROS_INFO("Info");
  H265VideoCompressor compressor(nh);

  ros::spin();

  return 0;
}