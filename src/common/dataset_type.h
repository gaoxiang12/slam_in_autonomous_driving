//
// Created by xiang on 22-8-16.
//

#ifndef SLAM_IN_AUTO_DRIVING_DATASET_TYPE_H
#define SLAM_IN_AUTO_DRIVING_DATASET_TYPE_H

namespace sad {

/// 枚举本书用到的一些数据集
enum class DatasetType {
    UNKNOWN = -1,
    NCLT = 0,   // NCLT: http://robots.engin.umich.edu/nclt/
    KITTI = 1,  // Kitti:
    ULHK = 3,   // https://github.com/weisongwen/UrbanLoco
    UTBM = 4,   // https://epan-utbm.github.io/utbm_robocar_dataset/
    AVIA = 5,   // https://epan-utbm.github.io/utbm_robocar_dataset/
    WXB_3D,     // 3d wxb
};

inline DatasetType Str2DatasetType(const std::string& name) {
    if (name == "NCLT") {
        return DatasetType::NCLT;
    }
    if (name == "KITTI") {
        return DatasetType::KITTI;
    }
    if (name == "ULHK") {
        return DatasetType::ULHK;
    }
    if (name == "UTBM") {
        return DatasetType::UTBM;
    }
    if (name == "WXB3D") {
        return DatasetType::WXB_3D;
    }
    if (name == "AVIA") {
        return DatasetType::AVIA;
    }

    return DatasetType::UNKNOWN;
}
/// 各种数据集里用的topic名称
const std::string nclt_rtk_topic = "gps_rtk_fix";

const std::string nclt_lidar_topic = "points_raw";
const std::string ulhk_lidar_topic = "/velodyne_points_0";
const std::string wxb_lidar_topic = "/velodyne_packets_1";
const std::string utbm_lidar_topic = "/velodyne_points";
const std::string avia_lidar_topic = "/livox/lidar";

const std::string ulhk_imu_topic = "/imu/data";
const std::string utbm_imu_topic = "/imu/data";
const std::string nclt_imu_topic = "imu_raw";
const std::string wxb_imu_topic = "/ivsensorimu";
const std::string avia_imu_topic = "/livox/imu";
// const std::string wxb_lidar_topic = "/velodyne_packets_1";
// const std::string utbm_lidar_topic = "/velodyne_points";

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_DATASET_TYPE_H
