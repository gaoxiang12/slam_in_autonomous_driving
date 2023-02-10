
//
// Created by pengguoqi on 20-08-24.
//

#ifndef HESAI_PC_CONVERTOR_H
#define HESAI_PC_CONVERTOR_H

#include "common/message_def.h"
#include "common/point_type.h"

namespace libfusion::utils {

class Compensator;

struct HesaiConf {
    std::string lidar_type = "Pandar64";
    double car_left = 0;
    double car_right = 0;
    double car_front = 0;
    double car_back = 0;
    double car_top = 0;
    double car_bottom = 0;
    double xoffset = 0;
    double yoffset = 0;
    double zoffset = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    uint16_t start_angle = 0;
    int tz = 0;
    int packets_size = 0;
};

//禾赛输出的packets转换成pointcloud格式
class HeSaiPcConvertor {
   public:
    explicit HeSaiPcConvertor(const HesaiConf &config);

    int Convert(const HeSaiScanMsg &scan, common::FullCloudPtr &output);

   private:
    std::shared_ptr<hesai::lidar::PandarGeneralSDK> hsdk_;

    std::shared_ptr<Compensator> compensator_;
};

}  // namespace libfusion::utils

#endif  // HESAI_PC_CONVERTOR_H
