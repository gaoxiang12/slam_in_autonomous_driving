//
// Created by xiang on 2022/1/4.
//

#ifndef SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H
#define SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H

#include "common/gnss.h"

namespace sad {

/**
 * 计算本书的GNSS读数对应的UTM pose和六自由度Pose
 * @param gnss_reading  输入gnss读数
 * @param antenna_pos   安装位置
 * @param antenna_angle 安装偏角
 * @param map_origin    地图原点，指定时，将从UTM位置中减掉坐标原点
 * @return
 */
bool ConvertGps2UTM(GNSS& gnss_reading, const Vec2d& antenna_pos, const double& antenna_angle,
                    const Vec3d& map_origin = Vec3d::Zero());

/**
 * 仅转换平移部分的经纬度，不作外参和角度处理
 * @param gnss_reading
 * @return
 */
bool ConvertGps2UTMOnlyTrans(GNSS& gnss_reading);

/**
 * 经纬度转UTM
 * NOTE 经纬度单位为度数
 * @param latlon
 * @param utm_coor
 * @return
 */
bool LatLon2UTM(const Vec2d& latlon, UTMCoordinate& utm_coor);

/**
 * UTM转经纬度
 * @param utm_coor
 * @param latlon
 * @return
 */
bool UTM2LatLon(const UTMCoordinate& utm_coor, Vec2d& latlon);

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H
