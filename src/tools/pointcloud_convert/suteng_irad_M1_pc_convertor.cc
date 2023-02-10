//
// Created by chenyajie on 21-12-06.
//
#include <pcl_conversions/pcl_conversions.h>

#include "common/point_type.h"
#include "utils/pointcloud_convert/suteng_irad_M1_pc_convertor.h"

#include <glog/logging.h>

namespace libfusion::utils {

using namespace avos::driver;

SuTengIRADM1PcConvertor::SuTengIRADM1PcConvertor() {
    driver_version_ = "v1.1.0";
    LOG(INFO) << " driver version : " << driver_version_;
}

SuTengIRADM1PcConvertor::SuTengIRADM1PcConvertor(const common::VehicleCalibrationParam &config) {
    manager_ptr_ = std::make_shared<RsMultiLidarApi>();
    config_map_ = config.config_map;
    driver_version_ = "v1.1.0";
    LOG(INFO) << " driver version : " << driver_version_;
}

SuTengIRADM1PcConvertor::~SuTengIRADM1PcConvertor() { manager_ptr_ = nullptr; }

std::string SuTengIRADM1PcConvertor::GetSutengLidarType() {
    std::string suteng_type = "RS_M1";
    return suteng_type;
}

int SuTengIRADM1PcConvertor::Convert(const SuTengIRADMultiPacketMsg &packet, common::FullCloudPtr &output) {
    output->clear();
    IRADCloudType::Ptr temp_cloud(new IRADCloudType);
    std::map<std::string, IRADCloudType> temp_cloud_map{};

    // IRADCloudType::Ptr cloud_out(new IRADCloudType);
    IRADCloudType::Ptr fusion_cloud(new IRADCloudType);

    if (packet.packets_topic.size() != packet.packets.size()) {
        LOG(WARNING) << "error packet size and string size is not equal " << packet.packets_topic.size() << "|"
                     << packet.packets.size();
        return -1;
    }
    if (packet.packets_topic.size() != 4) {
        LOG(WARNING) << "error packet size!";
        return -2;
    }
    for (int i = 0; i < packet.packets_topic.size(); i++) {
        for (auto &p : config_map_) {
            if (p.second.packets_topic == packet.packets_topic[i]) {
                manager_ptr_->Init(p.second.decoder_config);
                int re = manager_ptr_->Convert(packet.packets[i], temp_cloud);
                if (re < 0) {
                    return re;
                }
                temp_cloud_map[p.second.packets_topic] = *temp_cloud;
                break;
            }
        }
    }

    if (temp_cloud_map.size() != 4) {
        LOG(WARNING) << "driver convert failed!";
        return -2;
    }

    *fusion_cloud = temp_cloud_map["/driver/lidar/rs_M1_packets/front"];
    *fusion_cloud += temp_cloud_map["/driver/lidar/rs_M1_packets/rear"];
    *fusion_cloud += temp_cloud_map["/driver/lidar/rs_M1_packets/left"];
    *fusion_cloud += temp_cloud_map["/driver/lidar/rs_M1_packets/right"];

    common::ToFullCloud<IRADPoint>(fusion_cloud, output);

    return 0;
}

}  // namespace libfusion::utils