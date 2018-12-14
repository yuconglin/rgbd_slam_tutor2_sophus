#ifndef ROS_WRAPPER
#define ROS_WRAPPER
#include "common_headers.h"
#include "parameter_reader.h"
#include "pose_graph.h"
#include "rgbdframe.h"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <rgbd_slam_tutor2_sophus/rgbd_receive.h>

namespace rgbd_tutor
{
using namespace rgbd_tutor;

class PoseGraph;

class RosWrapper
{
public:
  //RosWrapper(const ParameterReader& para, PoseGraph& graph);
  RosWrapper(const ParameterReader& para, shared_ptr<Tracker>& t, PoseGraph& graph);
  void shutdown();
  void SubscriberRun();
  void ConsumerRun();
  void ConsumerRunSimple();
  void ConsumerRunQueue();
  void SignalHandling();

protected:
  ros::NodeHandle nh_;
  ros::Subscriber rgb_subscriber_;
  ros::Subscriber depth_subscriber_;
  ros::ServiceServer serv_;
  
  std::queue<sensor_msgs::Image> rgb_queue;
  std::queue<sensor_msgs::Image> depth_queue;

  std::queue<sensor_msgs::Image> rgb_msgs;
  std::vector<sensor_msgs::Image> depth_msgs;
  sensor_msgs::Image rgb_msg;
  sensor_msgs::Image depth_msg;
  bool is_rgb_updated;
  bool is_depth_updated;
  //std::queue<sensor_msgs::Image> depth_msgs;

  std::condition_variable rgb_updated;
  std::mutex              rgb_updated_mutex;
  std::mutex              rgb_mutex;
  std::condition_variable depth_updated;
  std::mutex              depth_updated_mutex;
  std::mutex              depth_mutex;

  int currentIndex;
  bool shutdownFlag;
  int depth_count;
  int i_start;

  CAMERA_INTRINSIC_PARAMETERS     camera;
  shared_ptr<thread> subscriberThread;
  shared_ptr<thread> consumerThread;
  shared_ptr<thread> signalThread;
  const ParameterReader& parameterReader;
  PoseGraph&  poseGraph;
  shared_ptr<Tracker> tracker;

protected:
  void rgbCallback(const sensor_msgs::ImageConstPtr& message);
  void depthCallback(const sensor_msgs::ImageConstPtr& message);
  void depthToCV8UC1(cv::Mat& float_img, cv::Mat& mono8_img);
  bool updatePoseGraph(rgbd_slam_tutor2_sophus::rgbd_receive::Request &req, rgbd_slam_tutor2_sophus::rgbd_receive::Response &resp);
};

}

#endif