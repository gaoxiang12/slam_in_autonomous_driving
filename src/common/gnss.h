//
// Created by xiang on 2022/1/4.
//

#ifndef SLAM_IN_AUTO_DRIVING_GNSS_H
#define SLAM_IN_AUTO_DRIVING_GNSS_H

#include "common/eigen_types.h"
#include "common/message_def.h"

namespace sad {

/// GNSS状态位信息
/// 通常由GNSS厂商提供，这里使用千寻提供的状态位
enum class GpsStatusType {
    GNSS_FLOAT_SOLUTION = 5,         // 浮点解（cm到dm之间）
    GNSS_FIXED_SOLUTION = 4,         // 固定解（cm级）
    GNSS_PSEUDO_SOLUTION = 2,        // 伪距差分解（分米级）
    GNSS_SINGLE_POINT_SOLUTION = 1,  // 单点解（10m级）
    GNSS_NOT_EXIST = 0,              // GPS无信号
    GNSS_OTHER = -1,                 // 其他
};

/// UTM 坐标
struct UTMCoordinate {
    UTMCoordinate() = default;
    explicit UTMCoordinate(int zone, const Vec2d& xy = Vec2d::Zero(), bool north = true)
        : zone_(zone), xy_(xy), north_(north) {}

    int zone_ = 0;              // utm 区域
    Vec2d xy_ = Vec2d::Zero();  // utm xy
    double z_ = 0;              // z 高度（直接来自于gps）
    bool north_ = true;         // 是否在北半球
};

/// 一个GNSS读数结构
struct GNSS {
    GNSS() = default;
    GNSS(double unix_time, int status, const Vec3d& lat_lon_alt, double heading, bool heading_valid)
        : unix_time_(unix_time), lat_lon_alt_(lat_lon_alt), heading_(heading), heading_valid_(heading_valid) {
        status_ = GpsStatusType(status);
    }

    /// 从ros的NavSatFix进行转换
    /// NOTE 这个只有位置信息而没有朝向信息，UTM坐标请从ch3的代码进行转换
    GNSS(sensor_msgs::NavSatFix::Ptr msg) {
        unix_time_ = msg->header.stamp.toSec();
        // 状态位
        if (int(msg->status.status) >= int(sensor_msgs::NavSatStatus::STATUS_FIX)) {
            status_ = GpsStatusType::GNSS_FIXED_SOLUTION;
        } else {
            status_ = GpsStatusType::GNSS_OTHER;
        }
        // 经纬度
        lat_lon_alt_ << msg->latitude, msg->longitude, msg->altitude;
    }

    double unix_time_ = 0;                                  // unix系统时间
    GpsStatusType status_ = GpsStatusType::GNSS_NOT_EXIST;  // GNSS 状态位
    Vec3d lat_lon_alt_ = Vec3d::Zero();                     // 经度、纬度、高度，前二者单位为度
    double heading_ = 0.0;                                  // 双天线读到的方位角，单位为度
    bool heading_valid_ = false;                            // 方位角是否有效

    UTMCoordinate utm_;       // UTM 坐标（区域之类的也在内）
    bool utm_valid_ = false;  // UTM 坐标是否已经计算（若经纬度给出错误数值，此处也为false）

    SE3 utm_pose_;  // 用于后处理的6DoF Pose
};

}  // namespace sad

using GNSSPtr = std::shared_ptr<sad::GNSS>;

#endif  // SLAM_IN_AUTO_DRIVING_GNSS_H
