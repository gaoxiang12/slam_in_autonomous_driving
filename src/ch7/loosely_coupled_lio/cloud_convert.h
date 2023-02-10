#pragma once

#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/point_types.h"

namespace sad {

/**
 * 预处理雷达点云
 *
 * 将Velodyne, ouster, avia等数据转到FullCloud
 * 该类由MessageSync类持有，负责将收到的雷达消息与IMU同步并预处理后，再交给LO/LIO算法
 */
class CloudConvert {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum class LidarType {
        AVIA = 1,  // 大疆的固态雷达
        VELO32,    // Velodyne 32线
        OUST64,    // ouster 64线
    };

    CloudConvert() = default;
    ~CloudConvert() = default;

    /**
     * 处理livox avia 点云
     * @param msg
     * @param pcl_out
     */
    void Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, FullCloudPtr &pcl_out);

    /**
     * 处理sensor_msgs::PointCloud2点云
     * @param msg
     * @param pcl_out
     */
    void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, FullCloudPtr &pcl_out);

    /// 从YAML中读取参数
    void LoadFromYAML(const std::string &yaml);

   private:
    void AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    FullPointCloudType cloud_full_, cloud_out_;  // 输出点云
    LidarType lidar_type_ = LidarType::AVIA;     // 雷达类型
    int point_filter_num_ = 1;                   // 跳点
    int num_scans_ = 6;                          // 扫描线数
    float time_scale_ = 1e-3;                    // 雷达点的时间字段与秒的比例
};
}  // namespace sad