//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "ch6/occupancy_map.h"
#include "common/io_utils.h"
#include "common/sys_utils.h"

DEFINE_string(bag_path, "./dataset/sad/2dmapping/floor1.bag", "数据包路径");
DEFINE_string(method, "model/bresenham", "填充算法：model/bresenham");

/// 测试2D似然场法的ICP

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);

    /// 测试单个scan生成出来的occupancy grid是否正确
    rosbag_io
        .AddScan2DHandle("/pavo_scan_bottom",
                         [&](Scan2d::Ptr scan) {
                             sad::OccupancyMap oc_map;
                             if (FLAGS_method == "model") {
                                 sad::evaluate_and_call(
                                     [&]() {
                                         oc_map.AddLidarFrame(std::make_shared<sad::Frame>(scan),
                                                              sad::OccupancyMap::GridMethod::MODEL_POINTS);
                                     },
                                     "Occupancy with model points");
                             } else {
                                 sad::evaluate_and_call(
                                     [&]() {
                                         oc_map.AddLidarFrame(std::make_shared<sad::Frame>(scan),
                                                              sad::OccupancyMap::GridMethod::BRESENHAM);
                                     },
                                     "Occupancy with bresenham");
                             }
                             cv::imshow("occupancy map", oc_map.GetOccupancyGridBlackWhite());
                             cv::waitKey(10);
                             return true;
                         })
        .Go();

    return 0;
}