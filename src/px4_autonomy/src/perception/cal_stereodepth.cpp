#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <string>
#include <math.h>
#define t265
//#define iris_stil
using namespace cv;
using namespace std;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> RGBPointCloud;

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> mStatisticalOutlierRemoval;
pcl::RadiusOutlierRemoval<pcl::PointXYZ> mRadiusOutlierRemoval;

string config_path;
int saveImgIndex = 0;
// for PCLRadiusOutlierRemoval
double radius = 0.05;
int MinNeighbor = 5;

// for PCLStatisticalOutlierFilter
int mean_k = 20;
double stdThres = 0.1;
int  NumDisparities;
int displayImage, disparityMax, disparityMin;
int method = 0, p1, p2, disp12MaxDiff, speckleRange, fullDP, SADWindowSize;
float distanceMax, distanceMin;

int PreFilterSize = 255, PreFilterCap = 1, SmallerBlockSize = 7 ;
int RangeOfDisparity = 16, MinDisparity = 0, SizeOfBlockWindow = 15;
int numberOfDisparities = 112;
int TextureThreshold = 255;
int UniquenessRatio = 10;
int SpeckleWindowSize = 100;

double Lambda = 100., SigmaColor = 1.5;
int bUseWLSfilter = 1, UsePCLfiltering = 1;
int isShowDisparity = 1;
int pclOutlierMethod = 0;
ros::Publisher pubdepth,pubCloud,pubFilterCloud;

enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
//enum outlierMethod { StatisticalOutlier = 0, RadiusOutlier = 1} ;
//outlierMethod pclOutlierMethod = StatisticalOutlier;

#ifdef iris_stil
std::string leftImgtopicname = "/iris_0/stereo_camera/left/image_raw";
std::string rightImgtopicname = "/iris_0/stereo_camera/right/image_raw";
Mat cameraMatrixL = (Mat_<double>(3, 3) << 376.0, 0, 376.0,
                                            0, 376.0, 240.0,
                                            0, 0, 1.0);
Mat distCoeffL = (Mat_<double>(5, 1) << -0.1, 0.01, 0.0, 5e-5, -1e-4);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 376.0, 0, 376.0,
                                            0, 376.0, 240.0,
                                            0, 0, 1.0);
Mat distCoeffR = (Mat_<double>(5, 1) << -0.1, 0.01, 0.0, 5e-5, -1e-4);
Mat T = (Mat_<double>(3, 1) << 0, -0.12, 0); //T平移向量
Mat rec = (Mat_<double>(3, 1) <<0.04345, -0.05236, -0.01810);//rec旋转向量
Mat R = (Mat_<double>(3, 3) << 1.0, 0, 0,
         0, 1.0, 0,
         0, 0, 1.0);//R 旋转矩阵
#endif

#ifdef t265
/***
frame_id: "camera_fisheye1_optical_frame"
height: 800
width: 848
distortion_model: "plumb_bob"
D: [-0.0063258809968829155, 0.04120932146906853, -0.0386807806789875, 0.006657741963863373, 0.0]
K: [285.1953125, 0.0, 417.7026062011719, 0.0, 285.11090087890625, 401.0010070800781, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [285.1953125, 0.0, 417.7026062011719, 0.0, 0.0, 285.11090087890625, 401.0010070800781, 0.0, 0.0, 0.0, 1.0, 0.0]

frame_id: "camera_fisheye2_optical_frame"
height: 800
width: 848
distortion_model: "plumb_bob"
D: [-0.00628004502505064, 0.04418269172310829, -0.04190414026379585, 0.0076002501882612705, 0.0]
K: [285.2970886230469, 0.0, 426.4549865722656, 0.0, 285.3150939941406, 400.7929992675781, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [285.2970886230469, 0.0, 426.4549865722656, 0.0, 0.0, 285.3150939941406, 400.7929992675781, 0.0, 0.0, 0.0, 1.0, 0.0]
***/
    std::string leftImgtopicname = "/camera/fisheye1/image_raw";
    std::string rightImgtopicname = "/camera/fisheye2/image_raw";
    Mat cameraMatrixL = (Mat_<double>(3, 3) << 285.1953125, 0.0, 417.7026062011719,
                                                0.0, 285.11090087890625, 401.0010070800781,
                                                0.0, 0.0, 1.0);
    Mat distCoeffL = (Mat_<float>(1, 4) << -0.0063258809968829155, 0.04120932146906853, -0.0386807806789875, 0.006657741963863373);

    Mat cameraMatrixR = (Mat_<double>(3, 3) << 285.2970886230469, 0.0, 426.4549865722656,
                                                             0.0, 285.3150939941406, 400.7929992675781,
                                                             0.0, 0.0, 1.0);
    Mat distCoeffR = (Mat_<float>(1, 4) << -0.00628004502505064, 0.04418269172310829, -0.04190414026379585, 0.0076002501882612705);
    //R 旋转矩阵
    Mat R = (Mat_<double>(3, 3) << 0.99998426, 0.00194362, -0.00526411,
                                  -0.00196591,  0.99998903, -0.00423323,
                                   0.00525582,  0.00424351,  0.99997723);
    //T平移向量
    Mat T = (Mat_<double>(3, 1) << -0.06376617, -0.00015167, -0.00011729);

#endif
Mat Rl, Rr, Pl, Pr, Q;
Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
Rect validROIR;
Mat mapLx, mapLy, mapRx, mapRy;     //映射表
Mat rectifyImageL, rectifyImageR;
Mat xyz;

void getParameters()
{
    const string configFile = "/home/zhuchen/project/px4_ws/src/src/px4_command/config/stereoConfig.yaml";
    cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        ROS_INFO("%s don't exist!!",configFile.c_str());
        return;
    }
    int ignoreConfigFile = fsSettings["notUseConfigFile"];
    if(ignoreConfigFile)
        return;
    //camera parameter
    int useCameraParam;
    if(useCameraParam)
    {
        fsSettings["useCameraParam"] >> useCameraParam;
        fsSettings["K1"] >> cameraMatrixL;
        fsSettings["D1"] >> distCoeffL;
        fsSettings["K2"] >> cameraMatrixR;
        fsSettings["D2"] >> distCoeffR;
        fsSettings["R"] >> R;
        fsSettings["T"] >> T;
    }
    method  = fsSettings["method"];
//    conductStereoRectify = fsSettings["conductStereoRectify"];

    UsePCLfiltering = fsSettings["UsePCLfiltering"];
    disparityMax = fsSettings["disparityMax"];
    disparityMin = fsSettings["disparityMin"];
    distanceMax = fsSettings["distanceMax"];
    distanceMin = fsSettings["distanceMin"];

    // for StereoBM matcher
    RangeOfDisparity  = fsSettings["RangeOfDisparity"];
    SizeOfBlockWindow  = fsSettings["SizeOfBlockWindow"];
    PreFilterSize  = fsSettings["PreFilterSize"];
    PreFilterCap  = fsSettings["PreFilterCap"];
    SmallerBlockSize  = fsSettings["SmallerBlockSize"];
    MinDisparity  = fsSettings["MinDisparity"];
    numberOfDisparities  = fsSettings["NumDisparities"];
    TextureThreshold  = fsSettings["TextureThreshold"];
    UniquenessRatio  = fsSettings["UniquenessRatio"];
    SpeckleWindowSize  = fsSettings["SpeckleWindowSize"];

    // for StereoSGBM matcher
    p1 = fsSettings["p1"];
    p2 = fsSettings["p2"];
    disp12MaxDiff = fsSettings["disp12MaxDiff"];
    speckleRange = fsSettings["speckleRange"];
    fullDP = fsSettings["fullDP"];
    SADWindowSize = fsSettings["SADWindowSize"];

    // for WLS filter
    bUseWLSfilter = fsSettings["UseWLSfilter"];
    Lambda  = fsSettings["Lambda"];
    SigmaColor  = fsSettings["SigmaColor"];
//    displayImage = fsSettings["displayImage"];

    pclOutlierMethod = fsSettings["pclOutlierMethod"];

}
void onMouse(int event, int x, int y,int,void*)
{
    cv::Point origin;
    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
            origin = cv::Point(x, y);
            xyz.at<cv::Vec3f>(origin)[2] +=2;
            std::cout << origin << "in world coordinate is: " << xyz.at<cv::Vec3f>(origin)<< std::endl;
            break;
    }
}
void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)
{
    // color map
    float max_val = 255.0f;
    float map[8][4] = { { 0,0,0,114 },{ 0,0,1,185 },{ 1,0,0,114 },{ 1,0,1,174 },
    { 0,1,0,114 },{ 0,1,1,185 },{ 1,1,0,114 },{ 1,1,1,0 } };
    float sum = 0;
    for (int i = 0; i<8; i++)
        sum += map[i][3];

    float weights[8]; // relative   weights
    float cumsum[8];  // cumulative weights
    cumsum[0] = 0;
    for (int i = 0; i<7; i++) {
        weights[i] = sum / map[i][3];
        cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
    }

    int height_ = src.rows;
    int width_ = src.cols;
    // for all pixels do
    for (int v = 0; v<height_; v++) {
        for (int u = 0; u<width_; u++) {
            // get normalized value
            float val = std::min(std::max(src.data[v*width_ + u] / max_val, 0.0f), 1.0f);
            // find bin
            int i;
            for (i = 0; i<7; i++)
                if (val<cumsum[i + 1])
                    break;
            // compute red/green/blue values
            float   w = 1.0 - (val - cumsum[i])*weights[i];
            uchar r = (uchar)((w*map[i][0] + (1.0 - w)*map[i + 1][0]) * 255.0);
            uchar g = (uchar)((w*map[i][1] + (1.0 - w)*map[i + 1][1]) * 255.0);
            uchar b = (uchar)((w*map[i][2] + (1.0 - w)*map[i + 1][2]) * 255.0);
            //rgb内存连续存放
            disp.data[v*width_ * 3 + 3 * u + 0] = b;
            disp.data[v*width_ * 3 + 3 * u + 1] = g;
            disp.data[v*width_ * 3 + 3 * u + 2] = r;
        }
    }
}
pcl::PointCloud<pcl::PointXYZ>::Ptr PCLStatisticalOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_p)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    mStatisticalOutlierRemoval.setInputCloud (raw_cloud_p);

//    std::cerr << "Cloud before filtering: " << std::endl;
//    std::cerr << *raw_cloud_p << std::endl;

    mStatisticalOutlierRemoval.setMeanK (mean_k);
    mStatisticalOutlierRemoval.setStddevMulThresh (stdThres);
    mStatisticalOutlierRemoval.filter (*cloud_filtered);

    mStatisticalOutlierRemoval.setNegative (true);
    mStatisticalOutlierRemoval.filter (*cloud_filtered);

//    std::cerr << "Cloud after filtering: " << std::endl;
//    std::cerr << *cloud_filtered << std::endl;

    return cloud_filtered;

}
pcl::PointCloud<pcl::PointXYZ>::Ptr PCLRadiusOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_p)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    mRadiusOutlierRemoval.setInputCloud (raw_cloud_p);
    mRadiusOutlierRemoval.setRadiusSearch(radius);          // 设置搜索半径
    mRadiusOutlierRemoval.setMinNeighborsInRadius (MinNeighbor);   // 设置最少的邻点数量
    mRadiusOutlierRemoval.filter (*cloud_filtered);
    return cloud_filtered;

}
void pubPointCloud(cv::Mat disparity, cv::Mat imgL)
{
    reprojectImageTo3D(disparity, xyz, Q);
    xyz = xyz * 16;
    RGBPointCloud::Ptr cloud ( new RGBPointCloud );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);
    Mat depthimage(xyz.rows,xyz.cols,CV_16UC1);
    for (int m = 0; m < xyz.rows; m++){
        for (int n=0; n < xyz.cols; n++)
        {
            float d = xyz.ptr<float>(m)[n*3+2];
            // d 可能没有值，若如此，跳过此点
            if (d <= 0 || d > 10)
                continue;
            // d 存在值，则向点云增加一个点
             PointT p;
             pcl::PointXYZ p_;
             p.z = d;
             p.x = xyz.ptr<float>(m)[n*3];
             p.y = xyz.ptr<float>(m)[n*3+1];
             p_.z = d;
             p_.x = xyz.ptr<float>(m)[n*3];
             p_.y = xyz.ptr<float>(m)[n*3+1];
             if(imgL.channels() == 3)
             {
                 p.b = imgL.ptr<uchar>(m)[n*3];
                 p.g = imgL.ptr<uchar>(m)[n*3+1];
                 p.r = imgL.ptr<uchar>(m)[n*3+2];
             }
             else
             {
                 p.b = imgL.ptr<uchar>(m)[n];
                 p.g = imgL.ptr<uchar>(m)[n];
                 p.r = imgL.ptr<uchar>(m)[n];
             }
             // 把p加入到点云中
             cloud->points.push_back( p );
             cloudxyz->points.push_back(p_);
            depthimage.at<short>(m,n) = d;
        }
    }
    // 设置并保存点云
    cloud->header.frame_id = "map";
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    //cloud->header.stamp= ros::Time::now().toNSec();
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    sensor_msgs::PointCloud2 pc2;
//    bool UsePCLfiltering = true;
    if (UsePCLfiltering)
    {
        //conduct pointcloud filering provided by PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
        cloudPTR = cloudxyz;
        pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud;
        if(pclOutlierMethod == 0)
            result_cloud = PCLStatisticalOutlierFilter(cloudPTR);
        else
            result_cloud = PCLRadiusOutlierFilter(cloudPTR);
        //step 1. convert pcl_T to pcl_cloud2
        pcl::PCLPointCloud2 pcl_cloud;
        pcl::toPCLPointCloud2(*result_cloud, pcl_cloud);

        //step 2. convert pcl_cloud2 pc to ROS pc
        pcl_conversions::fromPCL(pcl_cloud, pc2);
    }
    pc2.header.frame_id = "map";
    pc2.header.stamp = ros::Time::now();
    pubCloud.publish(cloud);
    pubFilterCloud.publish(pc2);
    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthimage).toImageMsg();
    depth_msg->header.frame_id = "map";
    pubdepth.publish(depth_msg);

//    Mat color(dispf.size(), CV_8UC3);
    //GenerateFalseMap(imgDisparity8U, color);//转成彩图
    //imshow("disparity", color);
    //cv::setMouseCallback("disparity", onMouse, 0);
    cv::waitKey(10);
}
void calDispWithSGBM(Mat imgL, Mat imgR, Mat &imgDisparity8U)
{
    cv::Size imgSize = imgL.size();
    //int numberOfDisparities = ((imgSize.width / 8) + 15) & -16;
    int numberOfDisparities = 112;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
    sgbm->setPreFilterCap(10);
    int SADWindowSize = 15;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(16);
    int cn = imgL.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);

    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);

    int alg = STEREO_SGBM;
    if (alg == STEREO_HH)
        sgbm->setMode(cv::StereoSGBM::MODE_HH);
    else if (alg == STEREO_SGBM)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    else if (alg == STEREO_3WAY)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    Mat imgDisparity16S = Mat(imgL.rows, imgL.cols, CV_16S);
    sgbm->compute(imgL, imgR, imgDisparity16S);
    //去黑边
    Mat img1p, img2p;
    Mat disp, dispf;
    disp = imgDisparity16S.clone();
    copyMakeBorder(imgL, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    copyMakeBorder(imgR, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    dispf = disp.colRange(numberOfDisparities, img2p.cols- numberOfDisparities);
    dispf.convertTo(imgDisparity8U, CV_8U, 255 / (numberOfDisparities *16.));
    cv::imshow("dispf",imgDisparity8U);

    //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    reprojectImageTo3D(imgDisparity16S, xyz, Q, true);
    xyz = xyz * 16;
    RGBPointCloud::Ptr cloud ( new RGBPointCloud );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);
    Mat depthimage(xyz.rows,xyz.cols,CV_16UC1);
    for (int m = 0; m < xyz.rows; m++){
        for (int n=0; n < xyz.cols; n++)
        {
            float d = xyz.ptr<float>(m)[n*3+2];
            // d 可能没有值，若如此，跳过此点
            if (d <= 0 || d > 10)
                continue;
            // d 存在值，则向点云增加一个点
             PointT p;
             pcl::PointXYZ p_;
             p.z = d;
             p.x = xyz.ptr<float>(m)[n*3];
             p.y = xyz.ptr<float>(m)[n*3+1];
             p_.z = d;
             p_.x = xyz.ptr<float>(m)[n*3];
             p_.y = xyz.ptr<float>(m)[n*3+1];
             if(imgL.channels() == 3)
             {
                 p.b = imgL.ptr<uchar>(m)[n*3];
                 p.g = imgL.ptr<uchar>(m)[n*3+1];
                 p.r = imgL.ptr<uchar>(m)[n*3+2];
             }
             else
             {
                 p.b = imgL.ptr<uchar>(m)[n];
                 p.g = imgL.ptr<uchar>(m)[n];
                 p.r = imgL.ptr<uchar>(m)[n];
             }
             // 把p加入到点云中
             cloud->points.push_back( p );
             cloudxyz->points.push_back(p_);
            depthimage.at<short>(m,n) = d;
        }
    }
    // 设置并保存点云
    cloud->header.frame_id = "map";
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    //cloud->header.stamp= ros::Time::now().toNSec();
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    sensor_msgs::PointCloud2 pc2;
    bool UsePCLfiltering = true;
    if (UsePCLfiltering)
    {
        //conduct pointcloud filering provided by PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
        cloudPTR = cloudxyz;
        pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud;
        if(pclOutlierMethod == 0)
            result_cloud = PCLStatisticalOutlierFilter(cloudPTR);
        else
            result_cloud = PCLRadiusOutlierFilter(cloudPTR);
        //step 1. convert pcl_T to pcl_cloud2
        pcl::PCLPointCloud2 pcl_cloud;
        pcl::toPCLPointCloud2(*result_cloud, pcl_cloud);

        //step 2. convert pcl_cloud2 pc to ROS pc
        pcl_conversions::fromPCL(pcl_cloud, pc2);
    }
    pc2.header.frame_id = "map";
    pc2.header.stamp = ros::Time::now();
    pubCloud.publish(cloud);
    pubFilterCloud.publish(pc2);
    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthimage).toImageMsg();
    depth_msg->header.frame_id = "map";
    pubdepth.publish(depth_msg);

    Mat color(dispf.size(), CV_8UC3);
    GenerateFalseMap(imgDisparity8U, color);//转成彩图
    //imshow("disparity", color);
    //cv::setMouseCallback("disparity", onMouse, 0);
    cv::waitKey(10);
}

int calDispWithSGBMwls(Mat& left, Mat& right, Mat& filtered_disp) {

    if (left.empty() || right.empty())
        return 0;
    const Size imsize = left.size();
    const int32_t dims[3] = {imsize.width, imsize.height, imsize.width};

    Mat leftdpf = Mat::zeros(imsize, CV_32F);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    int cn = left.channels();
    p1 = 8 * cn*sgbmWinSize*sgbmWinSize;
    p2 = 32 * cn*sgbmWinSize*sgbmWinSize;
    //step 1, create left matcher instance
    cv::Ptr<StereoSGBM> left_matcher = cv::StereoSGBM::create(MinDisparity,
            numberOfDisparities,
            sgbmWinSize,
            p1, p2,
            disp12MaxDiff,
            PreFilterCap,
            UniquenessRatio,
            SpeckleWindowSize,
            speckleRange,
            fullDP);

    if(bUseWLSfilter)
    {
        //step 2, create filter instance
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
        wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
        wls_filter->setLambda(Lambda);
        wls_filter->setSigmaColor(SigmaColor);

        //step 3, create right matcher instance
        cv::Ptr<cv::StereoMatcher> right_matcher;
        right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

        //step 4, compute
        cv::Mat left_disp, right_disp;
        left_matcher-> compute(left, right, left_disp);
        right_matcher->compute(right, left, right_disp);

        //step 5, conduct filtering
//        cv::Mat filtered_disp;
        wls_filter->filter(left_disp,left,filtered_disp,right_disp);

        cv::Mat conf_map = cv::Mat(left.rows,left.cols,CV_8U);
        conf_map = Scalar(255);

        conf_map = wls_filter->getConfidenceMap();
        cv::Rect ROI = wls_filter->getROI();

        //step 6, return value
        Mat dmap = Mat(imsize, CV_8UC1, Scalar(0));
        filtered_disp.convertTo(dmap, CV_8UC1, 255. / (numberOfDisparities*16));
        if(isShowDisparity)
        {
            cv::imshow("dipwithwls",dmap);
            cv::Mat colordmap;
            //GenerateFalseMap(dmap,colordmap);
            //cv::imshow("dipwithcolor",colordmap);
            cv::waitKey(10);
        }
        return 1;
    }
    else
    {
        Mat dmap = Mat(imsize, CV_8UC1, Scalar(0));
        left_matcher->compute(left, right, leftdpf);
        filtered_disp = leftdpf;
        leftdpf.convertTo(dmap, CV_8UC1, 255. / (numberOfDisparities*16));
        if(isShowDisparity)
        {
            cv::imshow("dipwithwls",dmap);
            cv::waitKey(10);
        }
        return 1;
    }
}
int calDispWithBMwls(Mat& left, Mat& right, Mat&  filtered_disp) {

    if (left.empty() || right.empty())
        return 0;

    const Size imsize = left.size();
    Mat leftdpf = Mat::zeros(imsize, CV_32F);

    //step 1, create left matcher instance
    cv::Ptr<StereoBM> left_matcher = StereoBM::create(RangeOfDisparity, SizeOfBlockWindow);
    left_matcher->setPreFilterSize(PreFilterSize);
    left_matcher->setPreFilterCap(PreFilterCap);
    left_matcher->setSmallerBlockSize(SmallerBlockSize);
    left_matcher->setMinDisparity(MinDisparity);
    left_matcher->setNumDisparities(numberOfDisparities);
    left_matcher->setTextureThreshold(TextureThreshold);
    left_matcher->setUniquenessRatio(UniquenessRatio);
    left_matcher->setSpeckleWindowSize(SpeckleWindowSize);
    if(bUseWLSfilter)
    {
        //step 2, create filter instance
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
        wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
        wls_filter->setLambda(Lambda);
        wls_filter->setSigmaColor(SigmaColor);

        //step 3, create right matcher instance
        cv::Ptr<cv::StereoMatcher> right_matcher;
        right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

        //step 4, compute
        cv::Mat left_disp, right_disp;
        left_matcher-> compute(left, right, left_disp);
        right_matcher->compute(right, left, right_disp);

        //step 5, conduct filtering
//        cv::Mat filtered_disp;
        wls_filter->filter(left_disp,left,filtered_disp,right_disp);

        cv::Mat conf_map = cv::Mat(left.rows,left.cols,CV_8U);
        conf_map = Scalar(255);

        conf_map = wls_filter->getConfidenceMap();
        cv::Rect ROI = wls_filter->getROI();

        //step 6, return value
        Mat dmap = Mat(imsize, CV_8UC1, Scalar(0));

        //去黑边
        Mat imgL = left.clone();
        Mat imgR = right.clone();
        Mat img1p, img2p;
        Mat disp, dispf;
        disp = filtered_disp.clone();
        copyMakeBorder(imgL, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
        copyMakeBorder(imgR, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
        dispf = disp.colRange(numberOfDisparities, img2p.cols- numberOfDisparities);

        dispf.convertTo(dispf, CV_8UC1, 255 / (numberOfDisparities*16.));
        if(isShowDisparity)
        {
            cv::imshow("dipwithwls",dispf);
            cv::waitKey(10);
        }
        return 1;
    }
    else
    {
        cv::Mat left_disp_;
        Mat dmap = Mat(imsize, CV_8UC1, Scalar(0));
        left_matcher->compute(left, right, left_disp_);
        filtered_disp = left_disp_;
        leftdpf.convertTo(dmap, CV_8UC1, 255 / (numberOfDisparities*16.));
        if(isShowDisparity)
        {
            cv::imshow("dipwithwls",dmap);
            cv::waitKey(10);
        }
        return 1;
    }

}

/*给深度图上色*/
void rectifyImagePaint(cv::Mat rectifyImageL,cv::Mat rectifyImageR, cv::Size imageSize)
{
    //显示在同一张图上
    Mat canvas;
    double sf;
    int w, h;
    sf = 700. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC1);   //注意通道

    //左图像画到画布上
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                              //得到画布的一部分
    resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小
    Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域
        cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
    //rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形

    //右图像画到画布上
    canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分
    resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
    Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
        cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
    //rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
    //画上对应的线条
    for (int i = 0; i < canvas.rows; i += 16)
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
    imshow("rectified", canvas);
}

void caldepth_callback(const sensor_msgs::ImageConstPtr &img_l_msg,
                       const sensor_msgs::ImageConstPtr &img_r_msg)
{
        try
        {
         #ifdef iris_stil
         cv_bridge::CvImagePtr img_l_ptr = cv_bridge::toCvCopy(img_l_msg, sensor_msgs::image_encodings::BGR8);
         cv_bridge::CvImagePtr img_r_ptr = cv_bridge::toCvCopy(img_r_msg, sensor_msgs::image_encodings::BGR8);
         #endif
         #ifdef t265
          cv_bridge::CvImagePtr img_l_ptr = cv_bridge::toCvCopy(img_l_msg, sensor_msgs::image_encodings::MONO8);
          cv_bridge::CvImagePtr img_r_ptr = cv_bridge::toCvCopy(img_r_msg, sensor_msgs::image_encodings::MONO8);
         #endif
          cv::Mat img_ll=img_l_ptr->image;
          cv::Mat img_rr=img_r_ptr->image;
          cv::Size imageSize = img_ll.size();
          cv::Mat img_show(img_ll.rows,img_ll.cols+img_rr.cols,img_ll.type());
          img_ll.copyTo(img_show.colRange(0,img_ll.cols));
          img_rr.copyTo(img_show.colRange(img_ll.cols,img_ll.cols+img_rr.cols));
          cv::imshow("show_org",img_show);
          if(cv::waitKey(20)=='s')
          {
              saveImgIndex ++;
//              stringstream imgl,imgr;
//              std::string imglout,imgrout;
//              imgl<<saveImgIndex;
//              ss>>imglout;
              cv::imwrite("data/left"+std::to_string(saveImgIndex)+".png",img_ll);
              cv::imwrite("data/right"+std::to_string(saveImgIndex)+".png",img_rr);
          }
          cv::waitKey(10);

#ifdef iris_stil
          stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
                  0, imageSize, &validROIL, &validROIR);
          initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_16SC2, mapLx, mapLy);
          initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_16SC2, mapRx, mapRy);
          /*  经过remap之后，左右相机的图像已经共面并且行对准了 */
          remap(img_ll, rectifyImageL, mapLx, mapLy, INTER_LINEAR);//INTER_LINEAR
          remap(img_rr, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
#endif
#ifdef t265
          int max_disp = 112;
          float stereo_fov_rad = 90 * (M_PI/180);
          int stereo_height_px = 300;
//          float stereo_fov_rad = 170 * (M_PI/180);
//          int stereo_height_px = 800;
          float stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2);
          Mat R_left = (Mat_<double>(3, 3) << 1.0,    0,   0,
                                                0,  1.0,   0,
                                                0,    0,  1.0);
          Mat R_right = R;
          int stereo_width_px = stereo_height_px + max_disp;
          cv::Size stereo_size(stereo_width_px, stereo_height_px);
          int stereo_cx = (stereo_height_px - 1)/2 + max_disp;
          int stereo_cy = (stereo_height_px - 1)/2;
          cv::Mat P_left =(Mat_<double>(3, 4) << stereo_focal_px, 0,   stereo_cx, 0,
                                                   0, stereo_focal_px, stereo_cy, 0,
                                                   0,               0,         1, 0);
          cv::Mat P_right = P_left.clone();
          P_right.at<double>(0,3) = T.at<double>(0,0)*stereo_focal_px;
          Q = (Mat_<double>(4, 4) << 1,   0,       0,            -(stereo_cx - max_disp),
                                     0,   1,       0,            -stereo_cy,
                                     0,   0,       0,            stereo_focal_px,
                                     0,   0,   -1.0/T.at<double>(0,0), 0);

//          cv::fisheye::stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, stereo_size,
//                                     R, T, Rl, Rr, Pl, Pr, Q, fisheye::CALIB_FIX_INTRINSIC);

//          cv::fisheye::stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, stereo_size,
//                                     R, T, R_left, R_right, P_left, P_right, Q,0);
          //ROS_INFO("stereoRectify");
          cv::fisheye::initUndistortRectifyMap(cameraMatrixL, distCoeffL, R_left, P_left, stereo_size, CV_32FC1, mapLx, mapLy);
          cv::fisheye::initUndistortRectifyMap(cameraMatrixR, distCoeffR, R_right, P_right, stereo_size, CV_32FC1, mapRx, mapRy);

          //std::cout << "RL:" << Rl << std::endl << "P_right:" << P_right << std::endl<< "Pr:" << Pr ;

          /*  经过remap之后，左右相机的图像已经共面并且行对准了 */
          remap(img_ll, rectifyImageL, mapLx, mapLy, INTER_LINEAR);//INTER_LINEAR
          remap(img_rr, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
          cv::imshow("rectifyImageL",rectifyImageL);
          cv::imshow("rectifyImageR",rectifyImageR);
#endif
          cv::Mat imgDisparity32F = Mat(rectifyImageL.rows, rectifyImageL.cols, CV_32FC1);

          //rectifyImagePaint(rectifyImageL,rectifyImageR,imageSize);
          //rectifyImagePaint(rectifyImageL,rectifyImageR,rectifyImageL.size());
//          calDispWithSGBM(rectifyImageL, rectifyImageR, imgDisparity8U);

          if(method == 0)
          {
              if(calDispWithBMwls(rectifyImageL, rectifyImageR,imgDisparity32F))
                pubPointCloud(imgDisparity32F,rectifyImageL);
          }
          else
          {
              if(calDispWithSGBMwls(rectifyImageL, rectifyImageR,imgDisparity32F))
                pubPointCloud(imgDisparity32F,rectifyImageL);
          }
        }
        catch(cv_bridge::Exception &e)
        {
          ROS_ERROR("cv_bridge Exception %s",e.what());
        }
}

void caldepth_callback1(const sensor_msgs::ImageConstPtr &img_l_msg,
                       const sensor_msgs::ImageConstPtr &img_r_msg)
{
    try
    {
      //ROS_INFO("caldepth_callback1");
      cv_bridge::CvImagePtr img_l_ptr = cv_bridge::toCvCopy(img_l_msg, sensor_msgs::image_encodings::MONO8);
      cv_bridge::CvImagePtr img_r_ptr = cv_bridge::toCvCopy(img_r_msg, sensor_msgs::image_encodings::MONO8);

      cv::Mat img_ll=img_l_ptr->image;
      cv::Mat img_rr=img_r_ptr->image;

      cv::Size imageSize = img_ll.size();
      cv::Mat img_show(img_ll.rows,img_ll.cols+img_rr.cols,img_ll.type());
      img_ll.copyTo(img_show.colRange(0,img_ll.cols));
      img_rr.copyTo(img_show.colRange(img_ll.cols,img_ll.cols+img_rr.cols));

      cv::imshow("show_org",img_show);
     // cv::imshow("show_org1",img_rr);
      cv::waitKey(10);
    }
    catch(cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge Exception %s",e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cal_stereodepth");
    ros::NodeHandle nh;

    ROS_INFO("cal depth!");
    getParameters();
    message_filters::Subscriber<sensor_msgs::Image> sub_img_l(nh, leftImgtopicname, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_r(nh, rightImgtopicname, 1);
    //
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy_img;
    message_filters::Synchronizer<MySyncPolicy_img> sync_img(MySyncPolicy_img(10), sub_img_l, sub_img_r);
    //ros::Subscriber imgsub = nh.subscribe<sensor_msgs::Image>("/camera/fisheye1/image_raw",1,caldepth_callback2);
    sync_img.registerCallback(boost::bind(&caldepth_callback, _1, _2));
    pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/depth_cloud", 1);
    pubFilterCloud = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 1);
    pubdepth = nh.advertise<sensor_msgs::Image> ("/depth_image", 1);
    std::cout<<"-------------Brief-----------------"<<std::endl;
    std::cout<<"disparityMax: "<<disparityMax<<std::endl;
    std::cout<<"disparityMin: "<<disparityMin<<std::endl;
    std::cout<<"distanceMax: "<<distanceMax<<std::endl;
    std::cout<<"distanceMin: "<<distanceMin<<std::endl;

    std::cout<<"RangeOfDisparity: "<<RangeOfDisparity<<std::endl;
    std::cout<<"SizeOfBlockWindow: "<<SizeOfBlockWindow<<std::endl;
    std::cout<<"PreFilterSize: "<<PreFilterSize<<std::endl;
    std::cout<<"PreFilterCap: "<<PreFilterCap<<std::endl;
    std::cout<<"SmallerBlockSize: "<<SmallerBlockSize<<std::endl;
    std::cout<<"MinDisparity: "<<MinDisparity<<endl;
    std::cout<<"NumDisparities: "<<numberOfDisparities<<std::endl;
    std::cout<<"TextureThreshold: "<<TextureThreshold<<std::endl;
    std::cout<<"UniquenessRatio: "<<UniquenessRatio<<std::endl;
    std::cout<<"SpeckleWindowSize: "<<SpeckleWindowSize<<std::endl;

    std::cout<<"Lambda: "<<Lambda<<endl;
    std::cout<<"SigmaColor: "<<SigmaColor<<endl;
    std::cout<<"-----------------------------------"<<endl;
    ros::Rate loop_rate(30);
    while (nh.ok()) {
        //pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        }
}
