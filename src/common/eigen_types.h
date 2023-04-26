//
// Created by xiang on 2021/7/16.
//

#ifndef MAPPING_EIGEN_TYPES_H
#define MAPPING_EIGEN_TYPES_H

// 引入Eigen头文件与常用类型
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "sophus/se2.hpp"
#include "sophus/se3.hpp"

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec3b = Eigen::Matrix<char, 3, 1>;

using Vec2d = Eigen::Vector2d;
using Vec2f = Eigen::Vector2f;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec4d = Eigen::Vector4d;
using Vec4f = Eigen::Vector4f;
using Vec5d = Eigen::Matrix<double, 5, 1>;
using Vec5f = Eigen::Matrix<float, 5, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec9d = Eigen::Matrix<double, 9, 1>;
using Vec15d = Eigen::Matrix<double, 15, 15>;
using Vec18d = Eigen::Matrix<double, 18, 1>;

using Mat1d = Eigen::Matrix<double, 1, 1>;
using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat23d = Eigen::Matrix<double, 2, 3>;
using Mat32d = Eigen::Matrix<double, 3, 2>;
using Mat3d = Eigen::Matrix3d;
using Mat3f = Eigen::Matrix3f;
using Mat4d = Eigen::Matrix4d;
using Mat4f = Eigen::Matrix4f;
using Mat5d = Eigen::Matrix<double, 5, 5>;
using Mat5f = Eigen::Matrix<float, 5, 5>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat9d = Eigen::Matrix<double, 9, 9>;
using Mat96d = Eigen::Matrix<double, 9, 6>;
using Mat15d = Eigen::Matrix<double, 15, 15>;
using Mat18d = Eigen::Matrix<double, 18, 18>;

using VecXd = Eigen::Matrix<double, -1, 1>;
using MatXd = Eigen::Matrix<double, -1, -1>;
using MatX18d = Eigen::Matrix<double, -1, 18>;

using Quatd = Eigen::Quaterniond;
using Quatf = Eigen::Quaternionf;

const Mat3d Eye3d = Mat3d::Identity();
const Mat3f Eye3f = Mat3f::Identity();
const Vec3d Zero3d(0, 0, 0);
const Vec3f Zero3f(0, 0, 0);

// pose represented as sophus structs
using SE2 = Sophus::SE2d;
using SE2f = Sophus::SE2f;
using SO2 = Sophus::SO2d;
using SE3 = Sophus::SE3d;
using SE3f = Sophus::SE3f;
using SO3 = Sophus::SO3d;

using IdType = unsigned long;

// Vec2i 可用于索引，定义它的小于号，用于构建以它为key的maps
namespace sad {

/// 矢量比较
template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

/// 矢量哈希
template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

// 实现2D和3D的比较
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v1, const Eigen::Matrix<int, 3, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) || (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

/// @see Optimized Spatial Hashing for Collision Detection of Deformable Objects, Matthias Teschner et. al., VMV 2003
template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943)) % 10000000);
}

template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
}

constexpr auto less_vec2i = [](const Vec2i& v1, const Vec2i& v2) {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
};

template <typename S>
inline SE3 Mat4ToSE3(const Eigen::Matrix<S, 4, 4>& m) {
    /// 对R做归一化，防止sophus里的检查不过
    Quatd q(m.template block<3, 3>(0, 0).template cast<double>());
    q.normalize();
    return SE3(q, m.template block<3, 1>(0, 3).template cast<double>());
}

}  // namespace sad

#endif  // MAPPING_EIGEN_TYPES_H
