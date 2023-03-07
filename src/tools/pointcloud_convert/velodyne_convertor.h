#pragma once

#include "common/lidar_utils.h"
#include "common/point_types.h"
#include "tools/pointcloud_convert/packets_parser.h"
#include "tools/pointcloud_convert/velodyne_config.h"

namespace sad::tools {

/// velodyne输出的packets转换成pointcloud格式
/// 实质上只是对packets_parser外面再包了一层
class VelodyneConvertor {
   public:
    explicit VelodyneConvertor(const VelodyneConfig &config = VelodyneConfig());

    /**
     * 将packets msgs转换为FullCloud
     * 同时会根据velodyne_config_中的配置来将激光点云转到IMU系
     * @param packets_msg
     * @param out_cloud
     */
    void ProcessScan(const PacketsMsgPtr &packets_msg, FullCloudPtr &out_cloud);

   private:
    VelodyneConfig velodyne_config_;
    std::shared_ptr<PacketsParser> packets_parser_ = nullptr;
    FullCloudPtr converted_cloud_ = nullptr;
};

}  // namespace sad::tools
