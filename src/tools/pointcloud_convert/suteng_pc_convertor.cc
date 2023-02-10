//
// Created by pengguoqi on 20-08-12.
//

#include "utils/pointcloud_convert/suteng_pc_convertor.h"
#include "common/point_type.h"

#include <glog/logging.h>

namespace libfusion::utils {

using namespace robosense::lidar;

SuTengPcConvertor::SuTengPcConvertor(const std::string &config_path) : config_path_(config_path) {
    driver_version_ = "v1.1.0";
    LOG(INFO) << " driver version : " << driver_version_;
}

int SuTengPcConvertor::Init() {
    YAML::Node config;
    LOG(INFO) << " config_path : " << config_path_;
    try {
        config = YAML::LoadFile(config_path_);
    } catch (...) {
        LOG(ERROR) << " Config file format wrong! Please check the format or intendation! ";
        return -1;
    }
    manager_ptr_ = std::make_shared<Manager>();
    if (manager_ptr_->Init(config) < 0) {
        LOG(ERROR) << " Manager init failed! ";
        return -2;
    }
    return 0;
}

std::string SuTengPcConvertor::GetSutengLidarType() {
    std::string suteng_type = "RS80";
    if (manager_ptr_) {
        suteng_type = manager_ptr_->GetSutengLidarType();
    } else {
        LOG(ERROR) << "Manager not initial!";
    }
    return suteng_type;
}

int SuTengPcConvertor::Convert(const SuTengScanMsg &scan, const SuTengPacketsMsg &packet,
                               common::FullCloudPtr &output) {
    LOG(INFO) << std::setprecision(16) << "scan msg header: " << scan.header.stamp.toSec();
    RSCloudType::Ptr temp_cloud(new RSCloudType);
    output->clear();
    int re = manager_ptr_->Convert(scan, packet, temp_cloud);
    if (re < 0) {
        return re;
    }

    common::ToFullCloud<robosense::lidar::PointXYZITSu>(temp_cloud, output);
    return 0;
}

}  // namespace libfusion::utils