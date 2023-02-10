//
// Created by xiang on 2022/7/18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch7/direct_ndt_lo.h"
#include "ch7/ndt_3d.h"
#include "common/dataset_type.h"
#include "common/io_utils.h"
#include "common/timer/timer.h"

/// 本程序以ULHK数据集为例
/// 测试以NDT为主的Lidar Odometry
/// 若使用PCL NDT的话，会重新建立NDT树
DEFINE_string(bag_path, "./dataset/sad/ulhk/test2.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/KITTI/WXB_3D");  // 数据集类型
DEFINE_bool(use_pcl_ndt, false, "use pcl ndt to align?");
DEFINE_bool(use_ndt_nearby_6, false, "use ndt nearby 6?");
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path, sad::Str2DatasetType(FLAGS_dataset_type));

    sad::DirectNDTLO::Options options;
    options.use_pcl_ndt_ = fLB::FLAGS_use_pcl_ndt;
    options.ndt3d_options_.nearby_type_ =
        FLAGS_use_ndt_nearby_6 ? sad::Ndt3d::NearbyType::NEARBY6 : sad::Ndt3d::NearbyType::CENTER;
    options.display_realtime_cloud_ = FLAGS_display_map;
    sad::DirectNDTLO ndt_lo(options);

    rosbag_io
        .AddAutoPointCloudHandle([&ndt_lo](sensor_msgs::PointCloud2::Ptr msg) -> bool {
            sad::common::Timer::Evaluate(
                [&]() {
                    SE3 pose;
                    ndt_lo.AddCloud(sad::VoxelCloud(sad::PointCloud2ToCloudPtr(msg)), pose);
                },
                "NDT registration");
            return true;
        })
        .Go();

    if (FLAGS_display_map) {
        // 把地图存下来
        ndt_lo.SaveMap("./data/ch7/map.pcd");
    }

    sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
