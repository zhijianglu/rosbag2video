
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <beginner_tutorials/myNum.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
bool shouldquit = false;
// global variable
int key = 0;
typedef message_filters::sync_policies::ApproximateTime<Image, Image> sync_policy_classification;

VideoWriter writer;
VideoWriter outputVideo;

void callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right)
{
    cv::Mat left_img =  cv_bridge::toCvShare(left, "mono8")->image;
    cv::Mat right_img =  cv_bridge::toCvShare(right, "mono8")->image;
    cv::Mat bind_img(left_img.cols*2,left_img.rows,CV_16UC1);
    cv::hconcat(left_img, right_img, bind_img);
    imshow("img", bind_img);

    if(key==27){
        shouldquit = true;
    }

    if(!shouldquit){
        writer << (bind_img);
    }else{
        cout<<"quit record"<<endl;
    }

     key =   cv::waitKey(0);

//    cout << "I should record the pose: " << endl;
}



int main(int argc, char** argv)
{

    if(argc != 2 ){
        cout<<"usage:\n       rosrun rosbag_tools rosbag2video OUTPUT_PATH/output.flv"<<endl;
        return -1;
    }

    string video_name = argv[1];
//    video_name = video_name+"/output.flv";
    ros::init(argc, argv, "msg_filter");

    ros::NodeHandle nh;

//    string video_name = "out.avi";

//    writer.open(video_name, CV_FOURCC('M', 'J', 'P', 'G'), 20.0, Size(752 * 2, 480));
    writer.open(video_name,CV_FOURCC('F', 'L', 'V', '1') ,20 ,Size(752 * 2, 480),false);

//    VideoWriter writer = VideoWriter();

    message_filters::Subscriber<Image> left(nh, "/mynteye/left/image_raw", 1);
    message_filters::Subscriber<Image> right(nh, "/mynteye/right/image_raw", 1);
    message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(100), left, right);
    //TimeSynchronizer<CameraInfo, PoseStamped> sync(info_sub, pose_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate loop_rate(30);
    cout<<"press esc to quit record"<<endl;
    while(ros::ok() && !shouldquit){
        loop_rate.sleep();
//        ROS_INFO("ROS is ok!");
        ros::spinOnce();
    }

    writer.release();
    return 0;
}
