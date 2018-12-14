#ifndef RGBD_READER
#define RGBD_READER
#include "common_headers.h"
#include "parameter_reader.h"
#include "rgbdframe.h"

#include <thread>
#include <mutex>
//#include <functional>
//#include <map>
#include <condition_variable>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace rgbd_tutor
{
using namespace rgbd_tutor;

class RGBDFrame;

class RgbdReader
{
public:
  RgbdReader(const ParameterReader& para);
  RGBDFrame::Ptr next();

protected:
  ros::NodeHandle nh_;
  ros::Subscriber rgb_subscriber_;
  ros::Subscriber depth_subscriber_;
  /*
  cv::Mat depth;
  double depth_timestamp;
  std::vector<cv::Mat> rgbs;
  std::vector<double> rgb_timestamps;
  int rgb_count;
  */
  cv::Mat rgb;
  double rgb_timestamp;
  std::vector<cv::Mat> depths;
  std::vector<double> depth_timestamps;
  int depth_count;

  int depth_recv_count;
  int rgb_recv_count;

  std::condition_variable rgb_updated;
  std::mutex              rgb_updated_mutex;
  std::mutex              rgb_mutex;
  std::condition_variable depth_updated;
  std::mutex              depth_updated_mutex;
  std::mutex              depth_mutex;

  const   ParameterReader&    parameterReader;
  int currentIndex;
  CAMERA_INTRINSIC_PARAMETERS     camera;

  bool rgb_update;
  bool depth_update;

protected:
  void rgbCallback(const sensor_msgs::ImageConstPtr& message);
  void depthCallback(const sensor_msgs::ImageConstPtr& message);
  void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);
};

}

#endif