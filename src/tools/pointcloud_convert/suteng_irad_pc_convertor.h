
//
// Created by pengguoqi on 21-01-28.
//

#ifndef SUTENG_IRAD_PC_CONVERTOR_H
#define SUTENG_IRAD_PC_CONVERTOR_H

#include "common/message_def.h"
#include "common/num_type.h"
#include "common/point_type.h"
#include "utils/pointcloud_convert/velodyne_config.h"

namespace libfusion::utils {

//速腾输出的packets转换成pointcloud格式
class SuTengIRADPcConvertor {
   public:
    /**
     * 传入速腾的yaml路径(默认为local path下的suteng.yaml)
     * @param config_path
     */
    explicit SuTengIRADPcConvertor();
    ~SuTengIRADPcConvertor();

    int Init(const VelodyneConfig &config);

    int Convert(const SuTengIRADPacketsMsg &packet, common::FullCloudPtr &output);

    inline std::string GetDriverVersion() const { return driver_version_; }

    std::string GetSutengLidarType();

   private:
    std::string driver_version_;
    std::shared_ptr<avos::driver::RsLidarApi> manager_ptr_;
};

}  // namespace libfusion::utils

#endif  // SUTENG_PC_CONVERTOR_H
