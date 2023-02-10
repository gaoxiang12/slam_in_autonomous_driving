//
// Created by xiang on 2022/7/18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch7/loam-like/loam_like_odom.h"
#include "common/io_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "./dataset/sad/wxb/test1.bag", "path to wxb bag");
DEFINE_string(topic, "/velodyne_packets_1", "topic of lidar packets");
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 测试loam-like odometry的表现
    sad::LoamLikeOdom::Options options;
    options.display_realtime_cloud_ = FLAGS_display_map;
    sad::LoamLikeOdom lo(options);

    LOG(INFO) << "using topic: " << FLAGS_topic;
    sad::RosbagIO bag_io(fLS::FLAGS_bag_path);
    bag_io
        .AddVelodyneHandle(FLAGS_topic,
                           [&](sad::FullCloudPtr cloud) -> bool {
                               sad::common::Timer::Evaluate([&]() { lo.ProcessPointCloud(cloud); }, "Loam-like odom");
                               return true;
                           })
        .Go();

    lo.SaveMap("./data/ch7/loam_map.pcd");

    sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
