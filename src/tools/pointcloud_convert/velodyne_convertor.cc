//
// Created by idriver on 19-7-19.
//

#include "tools/pointcloud_convert/velodyne_convertor.h"

#include <glog/logging.h>

namespace sad::tools {

VelodyneConvertor::VelodyneConvertor(const VelodyneConfig &config) : velodyne_config_(config) {
    packets_parser_ = std::make_shared<PacketsParser>();
    if (packets_parser_->Setup(velodyne_config_) < 0) {
        LOG(ERROR) << "velodyne point cloud initialize raw data ERROR!!!";
        return;
    }

    converted_cloud_.reset(new FullPointCloudType());
}

void VelodyneConvertor::ProcessScan(const PacketsMsgPtr &packets_msg, FullCloudPtr &out_cloud) {
    packets_parser_->PaddingPointCloud(packets_msg, out_cloud);
}

}  // namespace sad::tools