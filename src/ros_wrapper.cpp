#include "ros_wrapper.h"
#include<signal.h>

using namespace rgbd_tutor;
RosWrapper::RosWrapper(const ParameterReader& para, shared_ptr<Tracker>& t, PoseGraph& pose_graph)
          : parameterReader(para), tracker(t), poseGraph(pose_graph), nh_("~")
{
    std::string rgb_topic = parameterReader.getData<string>("rgb_topic");
    std::string depth_topic = parameterReader.getData<string>("depth_topic");

    depth_subscriber_ = nh_.subscribe(depth_topic, 10, &RosWrapper::depthCallback, this);
    rgb_subscriber_ = nh_.subscribe(rgb_topic, 10, &RosWrapper::rgbCallback, this);
    //serv_ = nh_.advertiseService ("update_posegraph", &RosWrapper::updatePoseGraph, this);
    
    camera = parameterReader.getCamera();

    //signalThread = make_shared<thread>(bind(&RosWrapper::SignalHandling, this));
    //subscriberThread = make_shared<thread>(bind(&RosWrapper::SubscriberRun, this));
    consumerThread = make_shared<thread>(bind(&RosWrapper::ConsumerRunSimple, this));
    signalThread = nullptr;
    subscriberThread = nullptr;

    currentIndex = 0;
    depth_count = 10;
    shutdownFlag = false;
    i_start = 0;
    is_rgb_updated = false;
    is_depth_updated = false;
    std::cout << "RosWrapper got constructed\n";
}

void RosWrapper::shutdown()
{
    std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxx shutdown xxxxxxxxxxxxxxxxxxxxx \n";
    shutdownFlag = true;
    rgb_updated.notify_all();
    depth_updated.notify_all();

    std::cout << "please wait signal thread to stop ...\n";
    if (signalThread != nullptr) {
        signalThread->join();
    }

    std::cout << "please wait subscriber thread to stop ...\n";
    if (subscriberThread != nullptr) {
        subscriberThread->join();
    }
    
    std::cout << "please wait consumer thread to stop ...\n";
    if (consumerThread != nullptr) {
        consumerThread->join();
    }

    std::cout << "all shutdown\n";
}

void RosWrapper::SignalHandling()
{
    int sig_num;
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);

    while (1) {
        sigwait(&set, &sig_num);
        if (sig_num == SIGINT) {
            printf("signal_hander_thread caught SIGINT\n");
            shutdownFlag = true;
            break;
        }
    }
}

void RosWrapper::SubscriberRun()
{
    std::cout << "SubscriberRun() ready\n";
    ros::Rate r(30);
    while( ros::ok() && !shutdownFlag )
    {   
        ros::spinOnce(); 
        r.sleep();
    }
    std::cout << "xxxxxxxxxxxxx subscriber run ended xxxxxxxxxxxxxx\n";
    shutdownFlag = true;
}

void RosWrapper::rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{ 
    std::unique_lock<mutex> lck(rgb_mutex);
    //std::cout << "rgb updated \n";
    //rgb_queue.push(*msg);
    rgb_msg = *msg;
    is_rgb_updated = true;
    rgb_updated.notify_all();
}

void RosWrapper::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::unique_lock<mutex> lck(depth_mutex);
    //std::cout << "depth updated \n";
    //depth_queue.push(*msg);
    depth_msg = *msg;
    is_depth_updated = true;
    depth_updated.notify_all();
}

void RosWrapper::depthToCV8UC1(cv::Mat &depth_img, cv::Mat &mono8_img)
{
    //Process images
    if (depth_img.type() == CV_32FC1)
    {
        //std::cout << "depth CV_32FC1 \n";
        depth_img.convertTo(mono8_img, CV_8UC1, 1, 0); //milimeter (scale of mono8_img does not matter)
    }
    else if (depth_img.type() == CV_16UC1)
    {
        //sstd::cout << "depth CV_16UC1 \n";
        depth_img.convertTo(mono8_img, CV_8UC1, 50, 0);
        //mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
        //cv::Mat float_img;
        //depth_img.convertTo(mono8_img, CV_8UC1, 0.05, -25); //scale to 2cm
    }
    else {
        std::cout << "no depth conversion type\n";
    }
}

bool RosWrapper::updatePoseGraph(rgbd_slam_tutor2_sophus::rgbd_receive::Request &req, rgbd_slam_tutor2_sophus::rgbd_receive::Response &resp)
{
    std::cout << "updatePoseGraph \n";
    {
        unique_lock<mutex> lck(depth_mutex);
        if (!depth_queue.empty())
        {
            depth_msgs.push_back(depth_queue.front());
            depth_queue.pop();
        }
    }
    {
        unique_lock<mutex> lck(rgb_mutex);
        if (!rgb_queue.empty())
        {
            rgb_msgs.push(rgb_queue.front());
            rgb_queue.pop();
        }
    }

    if (rgb_msgs.empty() || depth_msgs.empty())
    {
        std::cout << "both empty \n";
        return false;
    }
    sensor_msgs::Image rgb_front = rgb_msgs.front();
    sensor_msgs::Image depth_front;

    double t = (double)rgb_front.header.stamp.sec + (double)rgb_front.header.stamp.nsec * 1e-9;
    double t_start = (double)depth_msgs[i_start].header.stamp.sec + (double)depth_msgs[i_start].header.stamp.nsec * 1e-9;
    double t_end = (double)depth_msgs.back().header.stamp.sec + (double)depth_msgs.back().header.stamp.nsec * 1e-9;
    double t_depth;

    double dt_thres = 0.01;

    if (t <= t_start)
    {
        std::cout << "xxx match less than t_start, t: " << std::setprecision(15) << t << " t_start: " << t_start << "\r";

        if (t_start - t > dt_thres)
        {
            std::cout << "rgb delay too much \r";
            rgb_msgs.pop();
            return false;
        }
        depth_front = depth_msgs[i_start++];
        t_depth = t_start;
        rgb_msgs.pop();
    }
    else if (t >= t_end)
    {
        std::cout << "rgb needs to wait for depths, skip ############\r";
        return false;
    }
    else
    {
        double t_prev = t_start;
        size_t i = i_start + 1;
        for (; i < depth_msgs.size(); ++i)
        {
            double t_curr = (double)depth_msgs[i].header.stamp.sec + (double)depth_msgs[i].header.stamp.nsec * 1e-9;
            if (t >= t_prev && t <= t_curr && t_curr - t < dt_thres)
            {
                depth_front = depth_msgs[i];
                t_depth = t_curr;
                i_start = i;
                break;
            }
            t_prev = t_curr;
        }
        rgb_msgs.pop();
        if (i == depth_msgs.size())
        {
            std::cout << "dt larger than threshold, skip ############\r";
            return false;
        }
    }

    //convert to cv::Mat
    //rgb
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rgb_front, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    cv::Mat rgb = cv_ptr->image;
    //depth
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_front);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    cv::Mat depth_float_img = cv_ptr->image;

    RGBDFrame::Ptr frame(new RGBDFrame);
    frame->id = currentIndex;
    frame->rgb = rgb;
    frame->depth = depth_float_img;
    frame->timestamp = t;
    std::cout << std::setprecision(15) << " rgb time: " << t << " depth time: " << t_depth << "\n";

    if (frame->rgb.data == nullptr || frame->depth.data == nullptr)
    {
        return false;
    }
    frame->camera = camera;
    currentIndex++;

    //process the frame
    cout << "*******************************************" << endl;
    boost::timer timer;
    cout << "loading frame " << frame->id << endl;
    SE3d T = tracker->updateFrame(frame);
    cout << std::setprecision(6) << "current frame T = " << endl
         << T.matrix() << endl;
    //real_traj.push_back(T);
    //times.push_back(t);
    //cv::imshow( "image", frame->rgb );
    if (poseGraph.tryInsertKeyFrame(frame) == true)
    {
        cout << "Insert key-frame succeed" << endl;
        //cv::waitKey(1);
    }
    else
    {
        cout << "Insert key-frame failed" << endl;
        //cv::waitKey(1);
    }
    cout << GREEN << "time cost=" << timer.elapsed() << RESET << endl;

    return true;
}

void RosWrapper::ConsumerRunQueue()
{
    std::cout << "ConsumerRunQueue() started \n";
    while (!shutdownFlag)
    {
        sensor_msgs::Image depth_msg_copy;
        {
            unique_lock<mutex> lck(depth_mutex);
            depth_updated.wait(lck, [this]{return !depth_queue.empty() || shutdownFlag;});
            depth_msg_copy = depth_queue.front();
            depth_queue.pop();
        }
        sensor_msgs::Image rgb_msg_copy;
        {
            unique_lock<mutex> lck(rgb_mutex);
            rgb_updated.wait(lck, [this]{return !rgb_queue.empty() || shutdownFlag;});
            rgb_msg_copy = rgb_queue.front();
            rgb_queue.pop();
        }
        if (shutdownFlag) {
            break;
        }
        if (depth_msg_copy.header.stamp - rgb_msg_copy.header.stamp > ros::Duration(1./30)) {
            std::cout << "xxxxxxxxxxxxxx time difference too large xxxxxxxxxxxxxxxxx \r";
            continue;
        }
        //convert to cv::Mat
        //rgb
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(rgb_msg_copy, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat rgb = cv_ptr->image;
        //depth
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_msg_copy);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat depth_float_img = cv_ptr->image;

        RGBDFrame::Ptr frame(new RGBDFrame);
        frame->id = currentIndex;
        frame->rgb = rgb;
        frame->depth = depth_float_img;
        frame->timestamp = rgb_msg_copy.header.stamp.toSec();
        std::cout << std::setprecision(15) 
                  << " rgb time: " << rgb_msg_copy.header.stamp.toSec() 
                  << " depth time: " << depth_msg_copy.header.stamp.toSec() <<  "\n";

        if (frame->rgb.data == nullptr || frame->depth.data == nullptr)
        {
            continue;
        }
        frame->camera = camera;
        currentIndex++;

        //process the frame
        cout<<"*******************************************"<<endl;
        boost::timer timer;
        cout << "loading frame " << frame->id <<endl;
        SE3d T = tracker -> updateFrame( frame );
        cout << std::setprecision(6) << "current frame T = " << endl << T.matrix() << endl;
        //cv::imshow( "image", frame->rgb );
        if ( poseGraph.tryInsertKeyFrame( frame ) == true )
        {
            cout<<"Insert key-frame succeed"<<endl;
            //cv::waitKey(1);
        }
        else
        {
            cout<<"Insert key-frame failed"<<endl;
            //cv::waitKey(1);
        }
        cout<< GREEN << "time cost=" << timer.elapsed() << RESET << endl;
    }
    cout << "ConsumerRunQueue shutdown\n";
    poseGraph.shutdown();
}

void RosWrapper::ConsumerRunSimple()
{
    std::cout << "ConsumerRunSimple() started \n";

    while (!shutdownFlag)
    {
        sensor_msgs::Image depth_msg_copy;
        {
            unique_lock<mutex> lck(depth_mutex);
            //std::cout << "############# waiting for depth############\r";
            depth_updated.wait(lck, [this]{return is_depth_updated || shutdownFlag;});
            depth_msg_copy = depth_msg;
            is_depth_updated = false;
        }
        sensor_msgs::Image rgb_msg_copy;
        {
            unique_lock<mutex> lck(rgb_mutex);
            //std::cout << "############# waiting for rgb############\r";
            rgb_updated.wait(lck, [this]{return is_rgb_updated || shutdownFlag;});
            rgb_msg_copy = rgb_msg;
            is_rgb_updated = false;
        }
        if (shutdownFlag) {
            break;
        }
        if (depth_msg_copy.header.stamp - rgb_msg_copy.header.stamp > ros::Duration(1./30)) {
            continue;
        }

        //convert to cv::Mat
        //rgb
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(rgb_msg_copy, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat rgb = cv_ptr->image;
        //depth
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_msg_copy);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat depth_float_img = cv_ptr->image;

        RGBDFrame::Ptr frame(new RGBDFrame);
        frame->id = currentIndex;
        frame->rgb = rgb;
        frame->depth = depth_float_img;
        frame->timestamp = rgb_msg_copy.header.stamp.toSec();
        std::cout << std::setprecision(15) 
                  << " rgb time: " << rgb_msg_copy.header.stamp.toSec() 
                  << " depth time: " << depth_msg_copy.header.stamp.toSec() <<  "\n";

        if (frame->rgb.data == nullptr || frame->depth.data == nullptr)
        {
            continue;
        }
        frame->camera = camera;
        currentIndex++;

        //process the frame
        cout<<"*******************************************"<<endl;
        boost::timer timer;
        cout << "loading frame " << frame->id <<endl;
        SE3d T = tracker -> updateFrame( frame );
        cout << std::setprecision(6) << "current frame T = " << endl << T.matrix() << endl;
        //cv::imshow( "image", frame->rgb );
        if ( poseGraph.tryInsertKeyFrame( frame ) == true )
        {
            cout<<"Insert key-frame succeed"<<endl;
            //cv::waitKey(1);
        }
        else
        {
            cout<<"Insert key-frame failed"<<endl;
            //cv::waitKey(1);
        }
        cout<< GREEN << "time cost=" << timer.elapsed() << RESET << endl;
    }
    cout << "ConsumerRunSimple shutdown\n";
    poseGraph.shutdown();
}

void RosWrapper::ConsumerRun()
{
    int i_start = 0;
    std::cout << "ConsumerRun() started \n";
   
    vector<SE3d> real_traj;
    vector<double> times;

    while (!shutdownFlag)
    {
        {
            unique_lock<mutex> lck(depth_mutex);
            if (!depth_queue.empty()) 
            {
               depth_msgs.push_back(depth_queue.front());
               depth_queue.pop();
            } 
            /*
            unique_lock<mutex> lck(depth_mutex);
            depth_updated.wait(lck, [this]{return !depth_queue.empty() || shutdownFlag;});
            depth_msgs.push_back(depth_queue.front());
            depth_queue.pop();
            */
        }
        {
            unique_lock<mutex> lck(rgb_mutex);
            if (!rgb_queue.empty())
            {
               rgb_msgs.push(rgb_queue.front());
               rgb_queue.pop();
            }
            /*
            unique_lock<mutex> lck(rgb_mutex);
            rgb_updated.wait(lck, [this]{return !rgb_queue.empty() || shutdownFlag;});
            rgb_msgs.push(rgb_queue.front());
            rgb_queue.pop();
            */
        }
        /*
        if (shutdownFlag) {
            break;
        }*/
        
        if (rgb_msgs.empty() || depth_msgs.empty()) {
            continue;
        }
        sensor_msgs::Image rgb_front = rgb_msgs.front();
        sensor_msgs::Image depth_front;

        double t = (double)rgb_front.header.stamp.sec + (double)rgb_front.header.stamp.nsec * 1e-9;
        double t_start = (double)depth_msgs[i_start].header.stamp.sec + (double)depth_msgs[i_start].header.stamp.nsec * 1e-9;
        double t_end = (double)depth_msgs.back().header.stamp.sec + (double)depth_msgs.back().header.stamp.nsec * 1e-9;
        double t_depth;

        double dt_thres = 0.01;
        
        if (t <= t_start) 
        {
           std::cout << "xxx match less than t_start, t: " << std::setprecision(15) << t << " t_start: " << t_start << "\r";

           if (t_start - t > dt_thres) {
               std::cout << "rgb delay too much \r";
               rgb_msgs.pop();
               continue;
           }
           depth_front = depth_msgs[i_start++];
           t_depth = t_start;
           rgb_msgs.pop();
        }
        else if (t >= t_end)
        {
           std::cout << "rgb needs to wait for depths, skip ############\r";
           continue;
        }
        else 
        {
           double t_prev = t_start;
           size_t i = i_start + 1;
           for (; i < depth_msgs.size(); ++i)
           {
               double t_curr = (double)depth_msgs[i].header.stamp.sec + (double)depth_msgs[i].header.stamp.nsec * 1e-9;
               if (t >= t_prev && t <= t_curr && t_curr - t < dt_thres)
               {
                   depth_front = depth_msgs[i];
                   t_depth = t_curr;
                   i_start = i;      
                   break;
               }
               t_prev = t_curr;
           }
           rgb_msgs.pop();
           if (i == depth_msgs.size()) {
              std::cout << "dt larger than threshold, skip ############\r";
              continue;
           }
           
        }
        
        //convert to cv::Mat
        //rgb
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(rgb_front, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat rgb = cv_ptr->image;
        //depth
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_front);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat depth_float_img = cv_ptr->image;

        RGBDFrame::Ptr frame(new RGBDFrame);
        frame->id = currentIndex;
        frame->rgb = rgb;
        frame->depth = depth_float_img;
        frame->timestamp = t;
        std::cout << std::setprecision(15) << " rgb time: " << t << " depth time: " << t_depth <<  "\n";

        if (frame->rgb.data == nullptr || frame->depth.data == nullptr)
        {
            continue;
        }
        frame->camera = camera;
        currentIndex++;

        //process the frame
        cout<<"*******************************************"<<endl;
        boost::timer timer;
        cout << "loading frame " << frame->id <<endl;
        SE3d T = tracker -> updateFrame( frame );
        cout << std::setprecision(6) << "current frame T = " << endl << T.matrix() << endl;
        real_traj.push_back(T);
        times.push_back(t);
        //cv::imshow( "image", frame->rgb );
        if ( poseGraph.tryInsertKeyFrame( frame ) == true )
        {
            cout<<"Insert key-frame succeed"<<endl;
            //cv::waitKey(1);
        }
        else
        {
            cout<<"Insert key-frame failed"<<endl;
            //cv::waitKey(1);
        }
        cout<< GREEN << "time cost=" << timer.elapsed() << RESET << endl;

        //if (currentIndex > 2) break;
    } //while ends
    ofstream f("/home/ylin67/rgbdslam_catkin_ws/src/rgbd_slam_tutor2_sophus/data/realtime_traj.txt");
    for (size_t i = 0; i < real_traj.size(); ++i)
    {
        double* data = real_traj[i].data();
        double qx = data[0];
        double qy = data[1];
        double qz = data[2];
        double qw = data[3];
        double tx = data[4];
        double ty = data[5];
        double tz = data[6];

        double t = times[i];

        f << std::setprecision(15) << t << " " 
          <<  std::setprecision(5) << tx << " " << ty << " " << tz
          << " " << qx << " " << qy << " " << qz
          << " " << qw << "\n";
    }
    f.close();
    poseGraph.shutdown();
}