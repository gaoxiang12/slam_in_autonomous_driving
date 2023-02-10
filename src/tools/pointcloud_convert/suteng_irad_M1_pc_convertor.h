
//
// Created by chenyajie on 21-12-06.
//

#ifndef SUTENG_IRAD_M1_PC_CONVERTOR_H
#define SUTENG_IRAD_M1_PC_CONVERTOR_H

#include "common/message_def.h"
#include "common/num_type.h"
#include "common/point_type.h"
#include "common/vehicle_calib_param.h"

namespace libfusion::utils {

//速腾输出的固态激光packet转换成pointcloud格式
class SuTengIRADM1PcConvertor {
   public:
    /**
     * 传入速腾的yaml路径(默认为local path下的suteng.yaml)
     * @param config_path
     */
    explicit SuTengIRADM1PcConvertor();
    explicit SuTengIRADM1PcConvertor(const common::VehicleCalibrationParam &config);
    virtual ~SuTengIRADM1PcConvertor();

    int Convert(const SuTengIRADMultiPacketMsg &packet, common::FullCloudPtr &output);

    inline std::string GetDriverVersion() const { return driver_version_; }

    std::string GetSutengLidarType();

   private:
    std::string driver_version_;
    std::shared_ptr<avos::driver::RsMultiLidarApi> manager_ptr_;
    std::map<std::string, common::RsLidarDriverConfig> config_map_;
};

}  // namespace libfusion::utils

#endif  // SUTENG_PC_CONVERTOR_H
