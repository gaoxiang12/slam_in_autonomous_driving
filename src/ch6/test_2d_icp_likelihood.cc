//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "ch6/likelihood_filed.h"
#include "common/io_utils.h"

DEFINE_string(bag_path, "./dataset/sad/2dmapping/floor1.bag", "数据包路径");
DEFINE_string(method, "gauss-newton", "gauss-newton/g2o");

/// 测试2D似然场法的ICP

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    Scan2d::Ptr last_scan = nullptr, current_scan = nullptr;

    /// 我们将上一个scan与当前scan进行配准
    rosbag_io
        .AddScan2DHandle(
            "/pavo_scan_bottom",
            [&](Scan2d::Ptr scan) {
                sad::LikelihoodField lf;
                current_scan = scan;
                SE2 pose;

                if (last_scan == nullptr) {
                    last_scan = current_scan;
                    return true;
                }

                lf.SetTargetScan(last_scan);
                lf.SetSourceScan(current_scan);

                if (FLAGS_method == "gauss-newton") {
                    lf.AlignGaussNewton(pose);
                } else if (FLAGS_method == "g2o") {
                    lf.AlignG2O(pose);
                }

                LOG(INFO) << "aligned pose: " << pose.translation().transpose() << ", " << pose.so2().log();

                cv::Mat image;
                sad::Visualize2DScan(last_scan, SE2(), image, Vec3b(255, 0, 0));    // target是蓝的
                sad::Visualize2DScan(current_scan, pose, image, Vec3b(0, 0, 255));  // source是红的
                cv::imshow("scan", image);

                /// 画出target和它的场函数
                cv::Mat field_image = lf.GetFieldImage();
                sad::Visualize2DScan(last_scan, SE2(), field_image, Vec3b(255, 0, 0), 1000,
                                     20.0);  // target是蓝的
                cv::imshow("field", field_image);
                cv::waitKey(10);

                last_scan = current_scan;
                return true;
            })
        .Go();

    return 0;
}