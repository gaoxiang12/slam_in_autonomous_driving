#ifndef IMUTYPES_H
#define IMUTYPES_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include <utility>
#include <vector>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/nav_state.h"

namespace sad {

/**
 * IMU 预积分器
 *
 * 调用Integrate来插入新的IMU读数，然后通过Get函数得到预积分的值
 * 雅可比也可以通过本类获得，可用于构建g2o的边类
 */
class IMUPreintegration {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// 参数配置项
    /// 初始的零偏需要设置，其他可以不改
    struct Options {
        Options() {}
        Vec3d init_bg_ = Vec3d::Zero();  // 初始零偏
        Vec3d init_ba_ = Vec3d::Zero();  // 初始零偏
        double noise_gyro_ = 1e-2;       // 陀螺噪声，标准差
        double noise_acce_ = 1e-1;       // 加计噪声，标准差
    };

    IMUPreintegration(Options options = Options());

    /**
     * 插入新的IMU数据
     * @param imu   imu 读数
     * @param dt    时间差
     */
    void Integrate(const IMU &imu, double dt);

    /**
     * 从某个起始点开始预测积分之后的状态
     * @param start 起始时时刻状态
     * @return  预测的状态
     */
    NavStated Predict(const NavStated &start, const Vec3d &grav = Vec3d(0, 0, -9.81)) const;

    /// 获取修正之后的观测量，bias可以与预积分时期的不同，会有一阶修正
    SO3 GetDeltaRotation(const Vec3d &bg);
    Vec3d GetDeltaVelocity(const Vec3d &bg, const Vec3d &ba);
    Vec3d GetDeltaPosition(const Vec3d &bg, const Vec3d &ba);

   public:
    double dt_ = 0;                          // 整体预积分时间
    Mat9d cov_ = Mat9d::Zero();              // 累计噪声矩阵
    Mat6d noise_gyro_acce_ = Mat6d::Zero();  // 测量噪声矩阵

    // 零偏
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();

    // 预积分观测量
    SO3 dR_;
    Vec3d dv_ = Vec3d::Zero();
    Vec3d dp_ = Vec3d::Zero();

    // 雅可比矩阵
    Mat3d dR_dbg_ = Mat3d::Zero();
    Mat3d dV_dbg_ = Mat3d::Zero();
    Mat3d dV_dba_ = Mat3d::Zero();
    Mat3d dP_dbg_ = Mat3d::Zero();
    Mat3d dP_dba_ = Mat3d::Zero();
};

}  // namespace sad

#endif  // IMUTYPES_H
