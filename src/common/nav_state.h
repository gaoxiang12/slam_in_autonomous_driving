//
// Created by xiang on 2021/7/19.
//

#ifndef SAD_NAV_STATE_H
#define SAD_NAV_STATE_H

#include <sophus/so3.hpp>
#include "common/eigen_types.h"

namespace sad {

/**
 * 导航用的状态变量
 * @tparam T    标量类型
 *
 * 这是个封装好的类，部分程序使用本结构体，也有一部分程序使用散装的pvq，都是可以的
 */
template <typename T>
struct NavState {
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using SO3 = Sophus::SO3<T>;

    NavState() = default;

    // from time, R, p, v, bg, ba
    explicit NavState(double time, const SO3& R = SO3(), const Vec3& t = Vec3::Zero(), const Vec3& v = Vec3::Zero(),
                      const Vec3& bg = Vec3::Zero(), const Vec3& ba = Vec3::Zero())
        : timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba) {}

    // from pose and vel
    NavState(double time, const SE3& pose, const Vec3& vel = Vec3::Zero())
        : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {}

    /// 转换到Sophus
    Sophus::SE3<T> GetSE3() const { return SE3(R_, p_); }

    friend std::ostream& operator<<(std::ostream& os, const NavState<T>& s) {
        os << "p: " << s.p_.transpose() << ", v: " << s.v_.transpose()
           << ", q: " << s.R_.unit_quaternion().coeffs().transpose() << ", bg: " << s.bg_.transpose()
           << ", ba: " << s.ba_.transpose();
        return os;
    }

    double timestamp_ = 0;    // 时间
    SO3 R_;                   // 旋转
    Vec3 p_ = Vec3::Zero();   // 平移
    Vec3 v_ = Vec3::Zero();   // 速度
    Vec3 bg_ = Vec3::Zero();  // gyro 零偏
    Vec3 ba_ = Vec3::Zero();  // acce 零偏
};

using NavStated = NavState<double>;
using NavStatef = NavState<float>;

}  // namespace sad

#endif
