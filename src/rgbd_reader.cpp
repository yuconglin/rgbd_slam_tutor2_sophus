#include "rgbd_reader.h"

using namespace rgbd_tutor;

RgbdReader::RgbdReader(const ParameterReader& para) : parameterReader( para )
{
    std::string rgb_topic = parameterReader.getData<string>("rgb_topic");
    std::string depth_topic = parameterReader.getData<string>("depth_topic");

    depth_subscriber_ = nh_.subscribe(depth_topic, 10, &RgbdReader::depthCallback, this);

    rgb_subscriber_ = nh_.subscribe(rgb_topic, 10, &RgbdReader::rgbCallback, this);
    
    
    camera = parameterReader.getCamera();
    //rgb_count = 5;
    depth_count = 5;
    depth_recv_count = 0;
    rgb_recv_count = 0;

    currentIndex = 0;
    rgb_update = false;
    depth_update = false;
    std::cout << "rgbd_reader constructed\n";
}

RGBDFrame::Ptr RgbdReader::next()
{
    /*
    std::cout << " ...... waiting ....... \n";
    unique_lock<mutex> lck_update_rgb(rgb_updated_mutex);
    rgb_updated.wait(lck_update_rgb);
    unique_lock<mutex> lck_update_depth(depth_updated_mutex);
    depth_updated.wait(lck_update_depth);
    std::cout << " ....... the wait is over ..... \n";
    */
    if (!rgb_update || !depth_update) {
        return nullptr;
    }
    
    /*
    //make a copy first
    cv::Mat depth_copy = depth;
    double depth_time = depth_timestamp;
    std::vector<cv::Mat> rgbs_copy = rgbs;
    std::vector<double> rgb_times = rgb_timestamps;
    //to find the matching pair
    int idx = 0;
    double temp = std::abs(depth_time-rgb_times[0]);
    for (size_t i = 1; i < rgb_times.size(); ++i) {
        if (std::abs(depth_time-rgb_times[i]) < temp) {
            idx = i;
            temp = std::abs(depth_time-rgb_times[i]);
        }
    }
    */
    //make a copy first
    //to find the matching pair
    int idx = 0;
    double temp = std::abs(rgb_timestamp-depth_timestamps[0]);
    for (size_t i = 1; i < depth_timestamps.size(); ++i) {
        if (std::abs(rgb_timestamp - depth_timestamps[i]) < temp) {
            idx = i;
            temp = std::abs(rgb_timestamp - depth_timestamps[i]);
        }
    }

    RGBDFrame::Ptr frame (new RGBDFrame);
    frame->id = currentIndex;
    /*
    frame->rgb = rgbs_copy[idx];
    frame->depth = depth_copy;
    frame->timestamp = rgb_times[idx];
    */
    frame->rgb = rgb;
    frame->depth = depths[idx];
    frame->timestamp = rgb_timestamp;
    std::cout << std::setprecision(15) << "depth time: " << depth_timestamps[idx] << " rgb time: " << rgb_timestamp << "\n";

    if (frame->rgb.data == nullptr || frame->depth.data == nullptr)
    {
        return nullptr;
    }

    frame->camera = camera;
    currentIndex ++;
    rgb_update = false;
    depth_update = false;
    std::cout << "frame number :" << currentIndex << "\n";
    return frame;
}

void RgbdReader::rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    //std::cout << "rgb callback\n";
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //std::unique_lock<mutex> lck(rgb_mutex);
    /*
    rgbs.push_back(cv_ptr->image);
    if (rgbs.size() > rgb_count) 
    {
        rgbs.erase(rgbs.begin(), rgbs.end()-rgb_count);
    }
    double timestamp = (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * 1e-9;
    rgb_timestamps.push_back(timestamp);
    if (rgb_timestamps.size() > rgb_count)
    {
        rgb_timestamps.erase(rgb_timestamps.begin(), rgb_timestamps.end()-rgb_count);
    }*/
    rgb_timestamp = (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * 1e-9;
    rgb = cv_ptr->image;
    rgb_update = true;
    rgb_recv_count ++;
    std::cout << "rgb_recv: " << rgb_recv_count << "\n";
    //rgb_updated.notify_one();
    
}

void RgbdReader::depthToCV8UC1(const cv::Mat &float_img, cv::Mat &mono8_img)
{
    //Process images
    if (mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols)
    {
        mono8_img = cv::Mat(float_img.size(), CV_8UC1);
    }
    cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
}

void RgbdReader::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //std::cout << "depth callback\n";
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Copy the image.data to imageBuf.
    cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat depth_mono8_img;
    depthToCV8UC1(depth_float_img, depth_mono8_img);
    depths.push_back(depth_mono8_img);
    if (depths.size() > depth_count) 
    {
        depths.erase(depths.begin(), depths.end()-depth_count);
    }
    double timestamp = (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * 1e-9;
    depth_timestamps.push_back(timestamp);
    if (depth_timestamps.size() > depth_count)
    {
        depth_timestamps.erase(depth_timestamps.begin(), depth_timestamps.end()-depth_count);
    }

    //std::unique_lock<mutex> lck(depth_mutex);
    /*
    depth = depth_mono8_img;
    double timestamp = (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * 1e-9;
    depth_timestamp = timestamp;
    */
    depth_update = true;
    //depth_updated.notify_one();
    depth_recv_count ++;
    std::cout << "depth_recv: " << depth_recv_count << "\n";
}