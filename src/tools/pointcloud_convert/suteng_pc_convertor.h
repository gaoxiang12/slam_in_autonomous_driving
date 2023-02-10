
//
// Created by pengguoqi on 20-05-29.
//

#ifndef SUTENG_PC_CONVERTOR_H
#define SUTENG_PC_CONVERTOR_H

#include "common/message_def.h"
#include "common/num_type.h"
#include "common/point_type.h"

namespace libfusion::utils {

class Compensator;

//速腾输出的packets转换成pointcloud格式
class SuTengPcConvertor {
   public:
    /**
     * 传入速腾的yaml路径(默认为local path下的suteng.yaml)
     * @param config_path
     */
    explicit SuTengPcConvertor(const std::string &config_path);

    int Init();

    int Convert(const SuTengScanMsg &scan, const SuTengPacketsMsg &packet, common::FullCloudPtr &output);

    inline std::string GetDriverVersion() const { return driver_version_; }
    inline std::string GetLidarConfigFileName() const { return config_path_; }

    std::string GetSutengLidarType();

   private:
    std::string driver_version_;
    std::string config_path_;

    std::shared_ptr<robosense::lidar::Manager> manager_ptr_;
    std::shared_ptr<Compensator> compensator_;
};

}  // namespace libfusion::utils

#endif  // SUTENG_PC_CONVERTOR_H
