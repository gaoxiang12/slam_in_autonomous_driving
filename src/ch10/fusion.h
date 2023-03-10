//
// Created by xiang on 22-12-20.
//

#ifndef SLAM_IN_AUTO_DRIVING_FUSION_H
#define SLAM_IN_AUTO_DRIVING_FUSION_H

#include "ch3/eskf.hpp"
#include "ch3/static_imu_init.h"
#include "common/eigen_types.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/message_def.h"
#include "common/point_types.h"

#include "ch7/loosely_coupled_lio/cloud_convert.h"
#include "ch7/loosely_coupled_lio/measure_sync.h"

#include "tools/ui/pangolin_window.h"

#include <pcl/registration/ndt.h>
#include <sensor_msgs/PointCloud2.h>

namespace sad {

/**
 * 第10章显示的高精度融合定位，融合IMU、RTK、激光点云定位功能
 *
 * - NOTE 一些IMU的异常处理没有加在这里，有可能会被IMU带歪。
 */
class Fusion {
   public:
    explicit Fusion(const std::string& config_yaml);

    enum class Status {
        WAITING_FOR_RTK,  // 等待初始的RTK
        WORKING,          // 正常工作
    };

    /// 初始化，读取参数
    bool Init();

    /// 处理输入的RTK
    void ProcessRTK(GNSSPtr gnss);
    void ProcessIMU(IMUPtr imu);
    void ProcessPointCloud(sensor_msgs::PointCloud2::Ptr cloud);

   private:
    /// 读取某个点附近的地图
    void LoadMap(const SE3& pose);

    /// 处理同步之后的IMU和雷达数据
    void ProcessMeasurements(const MeasureGroup& meas);

    /// 读取地图的索引文件
    void LoadMapIndex();

    /// 网格搜索时的结构
    struct GridSearchResult {
        SE3 pose_;
        SE3 result_pose_;
        double score_ = 0.0;
    };

    /// 在初始RTK附近搜索车辆位置
    bool SearchRTK();

    /// 对网格搜索的某个点进行配准，得到配准后位姿与配准分值
    void AlignForGrid(GridSearchResult& gr);

    /// 激光定位
    bool LidarLocalization();

    /// 使用IMU初始化
    void TryInitIMU();

    /// 利用IMU预测状态信息
    /// 这段时间的预测数据会放入imu_states_里
    void Predict();

    /// 对measures_中的点云去畸变
    void Undistort();

    /// 执行一次配准和观测
    void Align();

    /// 标志位
    Status status_ = Status::WAITING_FOR_RTK;

    /// 数据
    std::string config_yaml_;                          // 配置文件路径
    Vec3d map_origin_ = Vec3d::Zero();                 // 地图原点
    std::string data_path_;                            // 地图数据目录
    std::set<Vec2i, less_vec<2>> map_data_index_;      // 哪些格子存在地图数据
    std::map<Vec2i, CloudPtr, less_vec<2>> map_data_;  // 第9章建立的地图数据

    std::shared_ptr<MessageSync> sync_ = nullptr;  // 消息同步器
    StaticIMUInit imu_init_;                       // IMU静止初始化

    /// 滤波器
    ESKFD eskf_;
    std::vector<NavStated> imu_states_;  // ESKF预测期间的状态

    FullCloudPtr scan_undistort_{new FullPointCloudType()};  // scan after undistortion
    CloudPtr current_scan_ = nullptr;

    SE3 TIL_;
    MeasureGroup measures_;  // sync IMU and lidar scan
    GNSSPtr last_gnss_ = nullptr;

    bool init_has_failed_ = false;  // 初始化是否失败过
    SE3 last_searched_pos_;         // 上次搜索的GNSS位置

    /// 激光定位
    bool imu_need_init_ = true;     // 是否需要估计IMU初始零偏
    CloudPtr ref_cloud_ = nullptr;  // NDT用于参考的点云
    pcl::NormalDistributionsTransform<PointType, PointType> ndt_;

    /// 参数
    double rtk_search_min_score_ = 4.5;

    // ui
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_FUSION_H
