//
// Created by pengguoqi on 21-01-28.
//

#include "utils/pointcloud_convert/suteng_irad_pc_convertor.h"
#include "common/point_type.h"

#include <glog/logging.h>

namespace libfusion::utils {

using namespace avos::driver;

SuTengIRADPcConvertor::SuTengIRADPcConvertor() {
    driver_version_ = "v1.1.0";
    LOG(INFO) << " driver version : " << driver_version_;
}

SuTengIRADPcConvertor::~SuTengIRADPcConvertor() { manager_ptr_ = nullptr; }

int SuTengIRADPcConvertor::Init(const VelodyneConfig &config) {
    manager_ptr_ = std::make_shared<RsLidarApi>();
    DecoderConfig decoder_config;
    decoder_config.tf_x = config.xoffset;
    decoder_config.tf_y = config.yoffset;
    decoder_config.tf_z = config.zoffset;
    decoder_config.tf_roll = config.roll;
    decoder_config.tf_pitch = config.pitch;
    decoder_config.tf_yaw = config.yaw;
    decoder_config.pre_rot_axis_0 = config.pre_rot_axis_0;
    decoder_config.pre_rot_axis_1 = config.pre_rot_axis_1;
    decoder_config.pre_rot_axis_2 = config.pre_rot_axis_2;
    decoder_config.pre_rot_degree_0 = config.pre_rot_degree_0;
    decoder_config.pre_rot_degree_1 = config.pre_rot_degree_1;
    decoder_config.pre_rot_degree_2 = config.pre_rot_degree_2;
    manager_ptr_->Init(decoder_config);
    return 0;
}

std::string SuTengIRADPcConvertor::GetSutengLidarType() {
    std::string suteng_type = "RS80";
    return suteng_type;
}

int SuTengIRADPcConvertor::Convert(const SuTengIRADPacketsMsg &packet, common::FullCloudPtr &output) {
    IRADCloudType::Ptr temp_cloud(new IRADCloudType);
    output->clear();
    int re = manager_ptr_->Convert(packet, temp_cloud);
    if (re < 0) {
        return re;
    }

    common::ToFullCloud<IRADPoint>(temp_cloud, output);
    return 0;
}

}  // namespace libfusion::utils