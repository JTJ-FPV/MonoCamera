#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ROS_KCF/kcftracker.hpp"

#include "AAMED/FLED.h"

#include "DBSCAN/dbscan.h"

namespace CADC{

class camera
{
private:
    ros::Publisher position;
    ros::Publisher aamed_pub;

    ros::Subscriber local_pos_sub;
    ros::Subscriber state;
    ros::Subscriber img_compress_sub;
    ros::Subscriber cam_info_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber odom_sub;


    ros::NodeHandle nh;

    int ROWS,COLS;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;  // Camera intrinsics
    double body_offset_x_, body_offset_y_, height_;
    double k1_,k2_,p1_,p2_,k3_;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();  // 内参矩阵
    Eigen::Matrix3d K_1_ = Eigen::Matrix3d::Identity();  // 内参矩阵
    Eigen::Matrix3d R_b_bc_ = Eigen::Matrix3d::Identity();  // 相机坐标转换成机体坐标系
    Eigen::Matrix3d R_Lc_bc_ = Eigen::Matrix3d::Identity();   // 相机坐标系转换成相机的水平坐标系
    KCFTracker tracker;     double centerDistance;
    AAMED aamed;
    DBSCAN::DBSCAN ds;


    void init_publisher();
    void init_subscriber();
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_cb(const std_msgs::Bool::ConstPtr& msg);
    void compressImg_cb(const sensor_msgs::CompressedImageConstPtr& img_compress);
    void cam_info_cb(const sensor_msgs::CameraInfo::ConstPtr& camInfo);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& imu);
    void odomCallback(const nav_msgs::OdometryConstPtr &odom);
public: 
    Eigen::Matrix3d R_Lb_ = Eigen::Matrix3d::Identity();  // 机体坐标系旋转到水平坐标系

public:
    camera(ros::NodeHandle* nodehandle, 
            AAMED aam, KCFTracker tra, DBSCAN::DBSCAN d, double centerDistance,
            double body_offset_x, double body_offset_y, double height);
    ~camera();

    ros::Timer calc_timer;

    virtual void calc_cb(const ros::TimerEvent&);


    void Body2LevelRotationMatrix(const Eigen::Vector3d &q);

    // void computeDbscanPointSet(const std::vector<cv::RotatedRect> &detEllipses);
    
    void classifiterPointSet(const std::vector<cv::RotatedRect> &detEllipses);

    void choosePoint(const std::vector<DBSCAN::Point> &p);

    cv::Rect computeEllipseRoI(const cv::RotatedRect &rotaterect);

    bool CenterDistance(const cv::Rect &r1, const cv::Rect &r2);

    double distance2d(const cv::Point2d p1, const cv::Point2d p2);

    Eigen::Vector3d computeLevelCoordinateWayPoint(DBSCAN::Point p);

    Eigen::Vector3d computeBodyCoordinateWayPoint(DBSCAN::Point p);

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped point_aim;
    sensor_msgs::Imu imu_data;
    std_msgs::Bool detect_sign;
    std_msgs::Bool point_aim_;
    std::vector<DBSCAN::Point> point;
    cv::Mat img_r,img_G;
    sensor_msgs::CameraInfo cam_info;
    Eigen::Vector3d eular;

    cv::Rect tracker_result;
    cv::Rect initRoi;
    cv::Rect nowRoi;
    uint count = 1;
    uint frame = 0;
};


}