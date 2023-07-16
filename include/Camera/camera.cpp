#include "camera.h"
#include <cmath>

namespace CADC
{
camera::~camera()
{

}

camera::camera(ros::NodeHandle* nodehandle, 
                AAMED aam, KCFTracker tra, DBSCAN::DBSCAN d, double centerDistance,
                double body_offset_x = 0, double body_offset_y = 0, double height = 0)
                :nh(*nodehandle), ds(d), aamed(aam), centerDistance(centerDistance)
{   
    body_offset_x_ = body_offset_x, body_offset_y_ = body_offset_y, height_ = height; 
    aamed.SetParameters(CV_PI / 3, 3.4, 0.77);
    ROS_INFO("init_publisher before");
    init_publisher();
    ROS_INFO("init_publisher after");
    ROS_INFO("init_subscriber before");
    init_subscriber();
    ROS_INFO("init_subscriber after");
    calc_timer = nh.createTimer(ros::Duration(0.05), &camera::calc_cb, this);
    // ds.setPoint(point);
    COLS = aamed.getdCOLS();
    ROWS = aamed.getdROWS();
    point_aim_.data = false;
    // *************simulation*************
    R_b_bc_ << 0, -1, 0,
               -1, 0, 0,
               0, 0, -1;
    // *************simulation*************

    // *************realflight*************
    // R_b_bc_ << 1, 0, 0,
    //            0, -1, 0,
    //            0, 0, -1;
    // *************realflight*************
    ROS_INFO("Construct");
}


void camera::init_publisher()
{
    position = nh.advertise<geometry_msgs::PoseStamped>
        ("/camera/ellipse/center", 10, this);
    aamed_pub = nh.advertise<std_msgs::Bool>
        ("/camera/ellipse/Bool", 10, this);
}

void camera::init_subscriber()
{
    // *************simulation*************
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, &camera::local_pos_cb,this);
    img_compress_sub = nh.subscribe<sensor_msgs::CompressedImage>
        ("/iris_0/usb_cam/image_raw/compressed", 10, &camera::compressImg_cb, this);
    cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>
        ("/iris_0/usb_cam/camera_info", 1, &camera::cam_info_cb, this);
    // *************simulation*************
    
    // *************realflight*************
    // local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //     ("mavros/vision_pose/pose", 10, &camera::local_pos_cb, this);
    // img_compress_sub = nh.subscribe<sensor_msgs::CompressedImage>
    //     ("/usb_cam/image_raw/compressed", 10, &camera::compressImg_cb, this);
    // cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>
    //     ("/usb_cam/camera_info", 1, &camera::cam_info_cb, this);
    // *************realflight*************

    state = nh.subscribe<std_msgs::Bool>
        ("/cadc_controll/detect", 10, &camera::state_cb, this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>
        ("/mavros/imu/data", 1, &camera::imu_cb, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("/mavros/local_position/odom", 1, &camera::odomCallback, this);
}

void camera::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;
}

void camera::state_cb(const std_msgs::Bool::ConstPtr& msg)
{
    detect_sign = *msg;
}


void camera::compressImg_cb(const sensor_msgs::CompressedImageConstPtr& img_compress)
{
    try{
        // Mat img;
        // ROS_INFO("compressImg_cb");
        Eigen::Vector3d aim;
        cv_bridge::CvImagePtr cv_compress_ptr = cv_bridge::toCvCopy(img_compress, sensor_msgs::image_encodings::BGR8);
        img_r = cv_compress_ptr->image;
        cv::cvtColor(img_r, img_G, cv::COLOR_RGB2GRAY);
        aamed.run_FLED(img_G);
        if(aamed.detEllipses.size() > 0)
        {
            // todo
            // 聚类算法   对DBSACN 下的Point进行修改增加一个与椭圆的索引
            classifiterPointSet(aamed.detEllipses);
            // 从分类后的点中获取目标点（决策）
            choosePoint(ds.m_points);

            if(ds.m_points.size() > 0)
            {   
                DBSCAN::Point p;
                p.index_ellipse = 0;
                if(0 == frame)
                {
                    // 第一帧图像对KCF初始化ROI区域并跟踪
                    ROS_INFO("KCF INIT");
                    initRoi = computeEllipseRoI(aamed.detEllipses[p.index_ellipse]);
                    tracker.init(initRoi, img_r);
                    p.x = aamed.detEllipses[p.index_ellipse].center.x;
                    p.y = aamed.detEllipses[p.index_ellipse].center.y;

                    point_aim_.data = true;
                    aamed_pub.publish(point_aim_);
                    ROS_INFO("*************************************");
                    ROS_INFO_STREAM("tracker center :" << p.x << ", " << p.y);
                    aim = computeBodyCoordinateWayPoint(p);
                    point_aim.pose.position.x = aim(0);
                    point_aim.pose.position.y = aim(1);
                    point_aim.pose.position.z = aim(2);
                    ROS_INFO("*************************************");
                    cv::rectangle( img_r, Point( tracker_result.x, tracker_result.y ), Point( tracker_result.x+tracker_result.width, 
                        tracker_result.y+tracker_result.height), cv::Scalar( 0, 255, 255 ), 1, 8 );
                    frame++;
                }
                else
                {
                    ROS_INFO("Track");
                    // 计算ROI区域
                    nowRoi = computeEllipseRoI(aamed.detEllipses[p.index_ellipse]);
                    tracker_result = tracker.update(img_r);
                    p.x = tracker_result.x + tracker_result.width / 2;
                    p.y = tracker_result.y + tracker_result.height / 2;
                    cv::Point2d center(p.x, p.y);
                    cv::circle(img_r, center, 3, cv::Scalar(255, 0, 0), -1);
                    // cv::imshow("IMGAE VIEWER", img_r);
                    if(CenterDistance(nowRoi, tracker_result))
                    {
                        point_aim_.data = true;
                        aamed_pub.publish(point_aim_);
                        ROS_INFO("*************************************");
                        ROS_INFO_STREAM("tracker center :" << p.x << ", " << p.y);
                        aim = computeBodyCoordinateWayPoint(p);
                        // ROS_INFO("Position begin");
                        point_aim.pose.position.x = aim(0);
                        point_aim.pose.position.y = aim(1);
                        point_aim.pose.position.z = aim(2);
                        // ROS_INFO("%lf, %lf, %lf", aim(0), aim(1), aim(2));
                        ROS_INFO("*************************************");
                        cv::rectangle( img_r, Point( tracker_result.x, tracker_result.y ), Point( tracker_result.x+tracker_result.width, 
                        tracker_result.y+tracker_result.height), cv::Scalar( 0, 255, 255 ), 1, 8 );
                        frame++;
                    }
                    else
                    {
                        initRoi = nowRoi;
                        frame = 0;
                        ROS_INFO("INIT KCF ROI AGAIN");
                    }
                }
            }
            else
            {
                point_aim_.data = false;
                aamed_pub.publish(point_aim_);
            }
        }
        else 
        {
            point_aim_.data = false;
            aamed_pub.publish(point_aim_);
        }
        aamed.drawFLED(img_r, "", "IMGAE VIEWER");
    }
    catch(cv_bridge::Exception &e){
        ROS_ERROR("convert fail!");
    }
}

void camera::cam_info_cb(const sensor_msgs::CameraInfo::ConstPtr &camInfo)
{
    cam_info = *camInfo;
    if(count == 1)
    {
        fx_ = cam_info.K.at(0);
        fy_ = cam_info.K.at(4);
        cx_ = cam_info.K.at(2);
        cy_ = cam_info.K.at(5);
        K_ << fx_, 0, cx_,
            0, fy_, cy_,
            0, 0, 1;
        K_1_ = K_.inverse();
        k1_ = cam_info.D.at(0);
        k2_ = cam_info.D.at(1);
        p1_ = cam_info.D.at(2);
        p2_ = cam_info.D.at(3);
        k3_ = cam_info.D.at(4);
        count = 2;
    }
}

void camera::imu_cb(const sensor_msgs::Imu::ConstPtr& imu)
{
    imu_data = *imu;
}


void camera::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);

    
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    eular << roll, pitch, yaw;
    // ROS_INFO("*************************************");
    // ROS_INFO_STREAM("ROLL : " << roll * 180 / M_PI);
    // ROS_INFO_STREAM("PITCH : " << pitch * 180 / M_PI);
    // ROS_INFO_STREAM("YAW : " << yaw * 180 / M_PI);
    // ROS_INFO("*************************************");
    Body2LevelRotationMatrix(eular);
}

void camera::Body2LevelRotationMatrix(const Eigen::Vector3d &q)
{  
    R_Lb_ = Eigen::AngleAxisd(-q(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(-q(0), Eigen::Vector3d::UnitX());
    R_Lc_bc_ = Eigen::AngleAxisd(q(0), Eigen::Vector3d::UnitY()) * 
               Eigen::AngleAxisd(q(1), Eigen::Vector3d::UnitX());
    // ROS_INFO("*************************************");
    // Eigen::AngleAxisd a(R_Lb_);
    // Eigen::Vector3d v = a.matrix().eulerAngles(2, 1, 0);          // ZYX顺序,roll, pitch, yaw
    // ROS_INFO("Level Coordinate X AIXED : %lf", v(2) * 180 / M_PI);
	// ROS_INFO("Level Coordinate Y AIXED : %lf", v(1) * 180 / M_PI);
	// ROS_INFO("Level Coordinate Z AIXED : %lf", v(0) * 180 / M_PI);
    // ROS_INFO("*************************************");
}

void camera::classifiterPointSet(const std::vector<cv::RotatedRect> &detEllipses)
{
    point.clear();
    DBSCAN::Point p;
    ds.m_points.clear();
    for(int i = 0; i < detEllipses.size(); ++i)
    {
        p.x = detEllipses[i].center.y;
        p.y = detEllipses[i].center.x;
        p.z = 1 / detEllipses[i].size.width + 1 / detEllipses[i].size.height + detEllipses[i].angle;
        p.index_ellipse = i;
        p.clusterID = UNCLASSIFIED;
        ds.m_points.push_back(p);
    }
    ds.run();
    cv::Point2f center;
    for(int i = 0; i < ds.m_points.size(); ++i)
    {
        center.x = ds.m_points[i].y;
        center.y = ds.m_points[i].x;
        ROS_INFO_STREAM("clusterID = " << ds.m_points[i].clusterID);
        if(ds.m_points[i].clusterID > 0)
            point.push_back(ds.m_points[i]);
        switch (ds.m_points[i].clusterID)
        {
        case 0:
            cv::circle(img_r, center, 3, cv::Scalar(255, 255, 255), -1);
            break;
        case 1:
            cv::circle(img_r, center, 3, cv::Scalar(0, 255, 255), -1);
            break;
        case 2:
            cv::circle(img_r, center, 3, cv::Scalar(255, 0, 255), -1);
            break;
        case 3:
            cv::circle(img_r, center, 3, cv::Scalar(255, 255, 0), -1);
            break;
        case 4:
            cv::circle(img_r, center, 3, cv::Scalar(255, 0, 0), -1);
            break;
        case 5:
            cv::circle(img_r, center, 3, cv::Scalar(0, 255, 0), -1);
            break;
        case 6:
            cv::circle(img_r, center, 3, cv::Scalar(0, 0, 255), -1);
        default:
            break;
        }
    }
}

void camera::choosePoint(const std::vector<DBSCAN::Point> &p)
{
    // 椭圆聚类分类出椭圆圆心
    if(p.size())
    {
        // 从多个椭圆中找到一个最大的圆
        
    }
    else // 如果聚类算法未分类出椭圆
    {
        // 判断是否为噪点

        
    }
}

cv::Rect camera::computeEllipseRoI(const cv::RotatedRect &rotaterect)
{
    cv::RotatedRect temp;
    Point2f ver[4];
    temp.size.height = rotaterect.size.width;
    temp.size.width = rotaterect.size.height;
    temp.center.x = rotaterect.center.y;
    temp.center.y = rotaterect.center.x;
    temp.angle = -rotaterect.angle;
    temp.points(ver);
    int radius =  (int)(sqrt(pow((ver[0].x - ver[2].x), 2) + pow(ver[0].y - ver[2].y, 2)) * 0.5);
    int right_up_x = temp.center.x - radius;
    int right_up_y = temp.center.y - radius;
    int r_x = radius * 2, r_y = radius * 2;
    if(right_up_x < 0 && right_up_y < 0 && right_up_x > -2 * radius && right_up_y > -2 * radius){
    r_x = 2 * radius + right_up_x;
    r_y = 2 * radius + right_up_y;
    right_up_x = 0, right_up_y = 0;
                    // cout << 1 << endl;
    }
    else if(right_up_x < 0 && right_up_y < 0 && (right_up_x <= -2 * radius || right_up_y <= -2 * radius)){
        r_x = 2 * radius + right_up_x;
        r_y = 2 * radius + right_up_y;
        right_up_x = 0, right_up_y = 0;
        // cout << 2 << endl;
    }
    else if(right_up_x < 0 && right_up_y >= 0 && right_up_x > -2 * radius && right_up_y < ROWS - 2 * radius){
        r_x = 2 * radius + right_up_x;
        right_up_x = 0;
        // cout << 3 << endl;
    }
    else if(right_up_x < 0 && right_up_y >= 0 && right_up_x > -2 * radius && right_up_y >= ROWS - 2 * radius && right_up_y < ROWS){
        r_x = 2 * radius + right_up_x;
        r_y= ROWS - right_up_y - 1;
        right_up_x = 0;
        // cout << 4 << endl;
    }
    else if(right_up_x < 0 && right_up_y >= 0 && (right_up_x <= -2 * radius || right_up_y >= ROWS)){
        r_x = 2 * radius + right_up_x;
        r_y= ROWS - right_up_y - 1;
        right_up_x = 0;
        // cout << 5 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_x >= COLS - 2 * radius && right_up_x < COLS && right_up_y >= ROWS - 2 * radius && right_up_y < ROWS){
        r_x = COLS - right_up_x - 1;
        r_y = ROWS - right_up_y - 1;
        // cout << 6 << endl;
    }
    else if(right_up_x >= COLS || right_up_y >= ROWS){
        r_x = COLS - right_up_x - 1;
        r_y = ROWS - right_up_y - 1;
        // cout << 7 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_x < COLS - 2 * radius && right_up_y >= ROWS - 2 * radius && right_up_y < ROWS){
        r_y = ROWS - right_up_y - 1;
        // cout << 8 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_x < COLS - 2 * radius && right_up_y >= ROWS){
        r_y = ROWS - right_up_y - 1;
        // cout << 9 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_y > ROWS - 2 * radius && right_up_x >= COLS - 2 * radius && right_up_x < COLS){
        r_x = COLS - right_up_x - 1;
        // cout << 10 << endl;
    }
    else if(right_up_x >= 0 && right_up_y >= 0 && right_up_y < ROWS - 2 * radius && right_up_x >= COLS){
        r_x = COLS - right_up_y - 1;
        // cout << 11 << endl;
    }
    else if(right_up_x >= 0 && right_up_y < 0 && right_up_x < COLS - 2 * radius && right_up_y >= -2 * radius){
        r_y = 2 * radius + right_up_y;
        right_up_y = 0;
        // cout << 12 << endl;
    }
    else if(right_up_x >= 0 && right_up_y < 0 && right_up_x >= COLS - 2 * radius && right_up_x < COLS && right_up_y >= -2 * radius){
        r_x = COLS - right_up_x;
        r_y = 2 * radius + right_up_y;
        right_up_y = 0;
        // cout << 13 << endl;
    }
    else if(right_up_x >= 0 && right_up_y < 0 && (right_up_x > COLS || right_up_y < -2 * radius)){
        r_x = COLS- right_up_x;
        r_y =2 * radius + right_up_y;
        // cout << 14 << endl;
    }
    // cout << right_up_x << "   " << right_up_y << "   " << r_x << "   " << r_y << endl;
    if(r_x <= 0 || r_y <= 0)
        r_x = 0, r_y = 0;
    else
    {
        if(right_up_x + r_x >= COLS)
        r_x = COLS - right_up_x - 1;
        if(right_up_y + r_y > ROWS)
        r_y = ROWS - right_up_y - 1;
    }
    return cv::Rect(right_up_x, right_up_y, r_x, r_y);
}

bool camera::CenterDistance(const cv::Rect &r1, const cv::Rect &r2)
{
    cv::Point2d p1(r1.x, r1.y);
    cv::Point2d p2(r2.x, r2.y);
    return (10 > distance2d(p1, p2)) ? true : false;
}

double camera::distance2d(const cv::Point2d p1, const cv::Point2d p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

Eigen::Vector3d camera::computeLevelCoordinateWayPoint(DBSCAN::Point p)
{
    Eigen::Vector3d pixel(p.x, p.y, 1);
    ROS_INFO_STREAM("center.x , center.y : " << p.x << " " << p.y);
    Eigen::Vector3d param_pixel_distorted = K_1_ * pixel;

    // *************realflight*************
    //去畸变
    double r = sqrt(pow(param_pixel_distorted(0), 2) + pow(param_pixel_distorted(1), 2));
    param_pixel_distorted(0) = param_pixel_distorted(0) * (1 + k1_ * r * r + k2_ * r * r * r * r) + 
                             2 * p1_ * param_pixel_distorted(0) * param_pixel_distorted(1) + 
                             p2_ * (r * r + 2 * param_pixel_distorted(0) * param_pixel_distorted(0));
    param_pixel_distorted(1) = param_pixel_distorted(1) * (1 + k1_ * r * r + k2_ * r * r * r * r) + 
                             p1_ * (r * r + 2 * param_pixel_distorted(1) * param_pixel_distorted(1)) +
                             2 * p2_ * param_pixel_distorted(0) * param_pixel_distorted(1);
    // *************realflight*************

    Eigen::Vector3d param_pixel_ = R_Lc_bc_ * param_pixel_distorted;
    Eigen::Vector3d point_l = ((pose.pose.position.z + height_) / param_pixel_(2)) * param_pixel_;

    // *************simulation*************
    point_l(0) += 5 / 2;
    point_l(1) += 3.5 / 2;
    // *************simulation*************

    ROS_INFO_STREAM("Level pixel :" << point_l(0) << " " << point_l(1) << " " << point_l(2));
    ROS_INFO_STREAM("Level distance :" << sqrt(pow(point_l(0), 2) + pow(point_l(1), 2)));
    return point_l;
}

Eigen::Vector3d camera::computeBodyCoordinateWayPoint(DBSCAN::Point p)
{
    Eigen::Vector3d point_l = computeLevelCoordinateWayPoint(p);
    // ROS_INFO("computeBodyCoordinateWayPoint begin");
    // Eigen::Vector3d point_b = R_b_bc_ * R_Lc_bc_.transpose() * point_l;
    Eigen::Vector3d point_b = R_b_bc_ * R_Lc_bc_.transpose() * point_l;
    ROS_INFO_STREAM("Body pixel :" << point_b(0) << " " << point_b(1) << " " << point_b(2));
    ROS_INFO_STREAM("Body distance :" << sqrt(pow(point_b(0), 2) + pow(point_b(1), 2)));
    // ROS_INFO("computeBodyCoordinateWayPoint end");
    return point_b;
}


// 处理函数
void camera::calc_cb(const ros::TimerEvent&)
{
    if(point_aim_.data)
        position.publish(point_aim);
}   

} // namespace CADC