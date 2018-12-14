#include "track.h"
#include "pose_graph.h"
#include "ros_wrapper.h"
#include <csignal>
using namespace rgbd_tutor;

int main(int argc, char** argv)
{
    ParameterReader	parameterReader;
    Tracker::Ptr	tracker( new Tracker(parameterReader)); 
    PoseGraph		poseGraph( parameterReader, tracker );

    ros::init(argc, argv, "exp_pose_graph_ros");
    RosWrapper ros_wrapper(parameterReader, tracker, poseGraph);
    /*
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    */
    ros::spin();
    //ros_wrapper.ConsumerRun();
    ros_wrapper.shutdown();
    return 0;
}