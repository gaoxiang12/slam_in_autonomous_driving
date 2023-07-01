//
// Created by xiang on 22-11-10.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch8/lio-preinteg/lio_preinteg.h"
#include "common/io_utils.h"
#include "common/sys_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "./dataset/sad/nclt/20120115.bag", "path to rosbag");
DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");                   // 数据集类型
DEFINE_string(config, "./config/velodyne_nclt.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path, sad::Str2DatasetType(FLAGS_dataset_type));

    sad::LioPreinteg lio;
    lio.Init(FLAGS_config);

    rosbag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            sad::common::Timer::Evaluate([&]() { lio.PCLCallBack(cloud); }, "Pre-Integration lio");
            return true;
        })
        .AddLivoxHandle([&](const livox_ros_driver::CustomMsg::ConstPtr& msg) -> bool {
            sad::common::Timer::Evaluate([&]() { lio.LivoxPCLCallBack(msg); }, "Pre-Integration lio");
            return true;
        })
        .AddImuHandle([&](IMUPtr imu) {
            lio.IMUCallBack(imu);
            return true;
        })
        .Go();

    lio.Finish();
    sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
