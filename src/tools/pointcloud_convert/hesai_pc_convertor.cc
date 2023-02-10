//
// Created by pengguoqi on 20-08-24.
//

#include <glog/logging.h>
#include <pcl/filters/filter.h>

#include "utils/pointcloud_convert/hesai_pc_convertor.h"

namespace libfusion::utils {

/*
40P <------> lidar_type = "Pandar40P"
64  <------> lidar_type = "Pandar64"    (默认)
20A <------> lidar_type = "Pandar20A"
20B <------> lidar_type = "Pandar20B"
QT <------> lidar_type = "PandarQT"
40M <------> lidar_type = "Pandar40M"
32 <------> lidar_type = "PandarXT-32"
16 <------> lidar_type = "PandarXT-16"
*/

HeSaiPcConvertor::HeSaiPcConvertor(const HesaiConf &config) {
    std::vector<double> calib_config;
    calib_config.resize(12);
    calib_config[0] = config.car_left;
    calib_config[1] = config.car_right;
    calib_config[2] = config.car_front;
    calib_config[3] = config.car_back;
    calib_config[4] = config.car_top;
    calib_config[5] = config.car_bottom;
    calib_config[6] = config.xoffset;
    calib_config[7] = config.yoffset;
    calib_config[8] = config.zoffset;
    calib_config[9] = config.roll;
    calib_config[10] = config.pitch;
    calib_config[11] = config.yaw;
    LOG(INFO) << " lidar_type : " << config.lidar_type;
    hsdk_ = std::make_shared<hesai::lidar::PandarGeneralSDK>(calib_config, config.lidar_type, config.start_angle,
                                                             config.tz, config.packets_size);
}

int HeSaiPcConvertor::Convert(const HeSaiScanMsg &scan, common::FullCloudPtr &output) {
    hesai::lidar::PPointCloudPtr temp_cloud(new hesai::lidar::PPointCloud());
    output->clear();
    int re = hsdk_->Convert(scan, temp_cloud);
    if (re < 0) {
        return re;
    }

    if (temp_cloud->empty()) {
        LOG(WARNING) << "input cloud is empty";
        return 0;
    }

    common::ToFullCloud<hesai::lidar::PPoint>(temp_cloud, output);
    return 0;
}

}  // namespace libfusion::utils