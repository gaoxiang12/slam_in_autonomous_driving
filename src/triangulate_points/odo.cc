#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>

// #include "extra.h" // used in opencv2 
using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

 //外参（车辆坐标前左上欧拉角+t）
 
//相机的delta R、t
Mat m_R =(Mat_<double> ( 3,3 )  << 0.9999999736220369, -3.738474711123444e-05, 7.744868119398606e-05,
 3.738455944382238e-05, 0.9999999950149243, 3.500264605118242e-07,
 -7.744877178140787e-05, -3.293732852899532e-07, 0.9999999786062898);
Mat m_t= (Mat_<double> ( 3,1 ) << 0.001340410128, -0.111443887762, 0.230887323070);

Mat m_K = ( Mat_<double> ( 3,3 ) << 281.86059, 0, 640.3191481, 0, 282.02232406291677   , 475.274866, 0.0, 0.0, 1.0 );
Mat m_D = (cv::Mat_<double>(4,1) << 0.12627605754199878,0.03423261902016277,-0.032336878920731164,0.005156524134036756);

// fish eye pixel 2 cam cor
// this is very important because fish eye distortion is very big
Point2f fishEyepixel2cam(const Point2d& p,const Mat& K,const Mat& D);


void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

void pose_estimation_2d2d (
    const std::vector<KeyPoint>& keypoints_1,
    const std::vector<KeyPoint>& keypoints_2,
    const std::vector< DMatch >& matches,
    Mat& R, Mat& t );

void triangulation (
    const vector<KeyPoint>& keypoint_1,
    const vector<KeyPoint>& keypoint_2,
    const std::vector< DMatch >& matches,
    const Mat& R, const Mat& t,
    vector<Point3d>& points
);

// 像素坐标转相机归一化坐标
Point2f pixel2cam( const Point2d& p, const Mat& K );

int main ( int argc, char** argv )
{

    //-- 读取图像
    Mat img_1 = imread ( "/home/hobbit/1.jpg", IMREAD_COLOR  );
    Mat img_2 = imread ( "/home/hobbit/2.jpg", IMREAD_COLOR  );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );

    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    imshow ( "所有匹配点对", img_match );
    waitKey(0);
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    //-- 估计两张图像间运动
    //Mat R,t;
    //pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );

    //-- 三角化
    vector<Point3d> points;
    triangulation( keypoints_1, keypoints_2, matches, m_R, m_t, points );
    
    //-- 验证三角化点与特征点的重投影关系
    Mat K = m_K;

    PointCloud::Ptr pointCloud( new PointCloud ); 

    for ( int i=0; i<matches.size(); i++ )
    {
     
        PointT p ;
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = points[i].z;

        pointCloud->points.push_back( p );
        cout<<"pix 2d is" <<keypoints_1[ matches[i].queryIdx ].pt<<endl;
        cout<<" 3D  points[i]"<< points[i]<<", d="<<points[i].z<<endl;
      
    }
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );

    return 0;
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3 
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2 
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
   // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

void pose_estimation_2d2d (
    const std::vector<KeyPoint>& keypoints_1,
    const std::vector<KeyPoint>& keypoints_2,
    const std::vector< DMatch >& matches,
    Mat& R, Mat& t )
{
    // 相机内参,TUM Freiburg2
   // Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    Mat K = m_K;
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, FM_8POINT );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point ( K.at<double>(0,2), K.at<double>(1,2) );				//相机主点, TUM dataset标定值
    int focal_length = K.at<double>(0,0);						//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
}

void triangulation ( 
    const vector< KeyPoint >& keypoint_1, 
    const vector< KeyPoint >& keypoint_2, 
    const std::vector< DMatch >& matches,
    const Mat& R, const Mat& t, 
    vector< Point3d >& points )
{
    Mat T1 = (Mat_<float> (3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );
    
    Mat K = m_K;
    vector<Point2f> pts_1, pts_2;
    for ( DMatch m:matches )
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back ( fishEyepixel2cam( keypoint_1[m.queryIdx].pt, K,m_D) );
        pts_2.push_back ( fishEyepixel2cam( keypoint_2[m.trainIdx].pt, K,m_D) );
    }
    
    Mat pts_4d;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );
    
    // 转换成非齐次坐标
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        Point3d p (
            x.at<float>(0,0), 
            x.at<float>(1,0), 
            x.at<float>(2,0) 
        );
        points.push_back( p );
    }
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
    );
}

Point2f fishEyepixel2cam(const Point2d& p,const Mat& K,const Mat& D)
{
    double fx =K.at<double>(0,0);
    double fy =K.at<double>(1,1);
    double u0 = K.at<double>(0,2);
    double v0 = K.at<double>(1,2);
    cv::Point2f pw((p.x - u0) / fx, (p.y - v0) / fy);
    float scale = 1.f;
    float theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);  // sin(psi) = yc / r
    theta_d = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);  // 不能超过180度

    if (theta_d > 1e-8)
    {
        // Compensate distortion iteratively
        // θ的初始值定为了θd
        float theta = theta_d;

        // 开始迭代
        for (int j = 0; j < 10; j++)
        {
        float theta2 = theta * theta,
        theta4 = theta2 * theta2,
        theta6 = theta4 * theta2,
        theta8 = theta4 * theta4;
        float k0_theta2 = D.at<double>(0,0) * theta2,
        k1_theta4 = D.at<double>(1,0) * theta4;
        float k2_theta6 = D.at<double>(2,0) * theta6,
        k3_theta8 = D.at<double>(3,0) * theta8;
        float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
        (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
        theta = theta - theta_fix;
        if (fabsf(theta_fix) < 1e-6)  // 如果更新量变得很小，表示接近最终值
        break;
        }
        // scale = theta - theta_d;
        // 求得tan(θ) / θd
        scale = std::tan(theta) / theta_d;
    }
    double norm=sqrt(pw.x * scale*pw.x * scale + pw.y * scale*pw.y * scale + 1);
    // double x=pw.x * scale/norm;
    // double y=pw.y * scale/norm;
    // double z= 1.f/norm;
    double x=pw.x * scale/norm;
    double y=pw.y * scale/norm;
// cout<<"pixel_ord="<<u<<","<<v<<endl;
// cout<<"cam_ord="<<x<<","<<y<<","<<z<<endl;
    return Point2f(x,y);
}