#include <ros/ros.h>
#include <Camera/camera.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>


// static const char WINDOW[] = "IMGAE VIEWER";
// KCF初始化参数
const bool HOG(true), FIXWINDOW(true), MULTISCALE(false), SILENT(false), LAB(false);
const int MINIMUM_POINTS = 2, EPSILON = 50;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "CADC_Camera");
    ros::NodeHandle nh;
   
    // *************simulation*************
    AAMED aam(240, 320);
    // *************simulation*************

    // *************realflight*************
    // AAMED aam(480, 640);
    // *************realflight*************
    KCFTracker tra(HOG, FIXWINDOW, MULTISCALE, LAB);
    DBSCAN::DBSCAN d(MINIMUM_POINTS, EPSILON);

    cv::namedWindow("IMGAE VIEWER");
    // cv::namedWindow("IMGAE GRAY VIEWER");
    cv::startWindowThread();
    ROS_INFO("main before");
    // CADC::camera cam(&nh, aam, tra, d, 0, 0, 0.7);
    CADC::camera cam(&nh, aam, tra, d, 20, 0, 0, 0);
    ROS_INFO("main after");
    ros::spin();
    return 0;
}
