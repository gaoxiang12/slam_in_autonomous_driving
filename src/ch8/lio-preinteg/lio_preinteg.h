#ifndef SAD_CH8_LIO_PREINTEG_H
#define SAD_CH8_LIO_PREINTEG_H

#include <livox_ros_driver/CustomMsg.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

/// 部分类直接使用ch7的结果
#include "ch3/static_imu_init.h"
#include "ch4/imu_preintegration.h"
#include "ch7/loosely_coupled_lio/cloud_convert.h"
#include "ch7/loosely_coupled_lio/measure_sync.h"
#include "ch7/ndt_inc.h"

#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

namespace sad {

/**
 * 第8章 基于预积分系统的LIO
 * 框架与前文一致，但之前由IEKF处理的部分变为预积分优化
 */
class LioPreinteg {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct Options {
        Options() {}
        bool with_ui_ = true;  // 是否带着UI
        bool verbose_ = true;  // 打印调试信息

        double bias_gyro_var_ = 1e-2;           // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-2;           // 加计零偏游走标准差
        Mat3d bg_rw_info_ = Mat3d::Identity();  // 陀螺随机游走信息阵
        Mat3d ba_rw_info_ = Mat3d::Identity();  // 加计随机游走信息阵

        double ndt_pos_noise_ = 0.1;                   // NDT位置方差
        double ndt_ang_noise_ = 2.0 * math::kDEG2RAD;  // NDT角度方差
        Mat6d ndt_info_ = Mat6d::Identity();           // 6D NDT 信息矩阵

        sad::IMUPreintegration::Options preinteg_options_;  // 预积分参数
        IncNdt3d::Options ndt_options_;                     // NDT 参数
    };

    LioPreinteg(Options options = Options());
    ~LioPreinteg() = default;

    /// init without ros
    bool Init(const std::string& config_yaml);

    /// 点云回调函数
    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg);

    /// IMU回调函数
    void IMUCallBack(IMUPtr msg_in);

    /// 结束程序，退出UI
    void Finish();

   private:
    bool LoadFromYAML(const std::string& yaml_file);

    /// 处理同步之后的IMU和雷达数据
    void ProcessMeasurements(const MeasureGroup& meas);

    /// 尝试让IMU初始化
    void TryInitIMU();

    /// 利用IMU预测状态信息
    /// 这段时间的预测数据会放入imu_states_里
    void Predict();

    /// 对measures_中的点云去畸变
    void Undistort();

    /// 执行一次配准和观测
    void Align();

    /// 执行预积分+NDT pose优化
    void Optimize();

    /// 将速度限制在正常区间
    void NormalizeVelocity();

    /// modules
    std::shared_ptr<MessageSync> sync_ = nullptr;
    StaticIMUInit imu_init_;

    /// point clouds data
    FullCloudPtr scan_undistort_{new FullPointCloudType()};  // scan after undistortion
    CloudPtr current_scan_ = nullptr;

    // optimize相关
    NavStated last_nav_state_, current_nav_state_;  // 上一时刻状态与本时刻状态
    Mat15d prior_info_ = Mat15d::Identity();        // 先验约束
    std::shared_ptr<IMUPreintegration> preinteg_ = nullptr;

    IMUPtr last_imu_ = nullptr;

    /// NDT数据
    IncNdt3d ndt_;
    SE3 ndt_pose_;
    SE3 last_ndt_pose_;

    // flags
    bool imu_need_init_ = true;
    bool flg_first_scan_ = true;
    int frame_num_ = 0;

    MeasureGroup measures_;  // sync IMU and lidar scan
    std::vector<NavStated> imu_states_;
    SE3 TIL_;  // Lidar与IMU之间外参

    Options options_;
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace sad

#endif  // FASTER_LIO_LASER_MAPPING_H