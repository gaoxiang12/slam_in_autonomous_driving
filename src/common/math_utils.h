//
// Created by xiang on 2021/7/19.
//

#ifndef MAPPING_MATH_UTILS_H
#define MAPPING_MATH_UTILS_H

#include <glog/logging.h>
#include <boost/array.hpp>
#include <boost/math/tools/precision.hpp>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <opencv2/core.hpp>

/// 常用的数学函数
namespace sad::math {

// 常量定义
constexpr double kDEG2RAD = M_PI / 180.0;  // deg->rad
constexpr double kRAD2DEG = 180.0 / M_PI;  // rad -> deg
constexpr double G_m_s2 = 9.81;            // 重力大小

// 非法定义
constexpr size_t kINVALID_ID = std::numeric_limits<size_t>::max();

/**
 * 计算一个容器内数据的均值与对角形式协方差
 * @tparam C    容器类型
 * @tparam D    结果类型
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个D类型
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
    // clang-format on
}

/**
 * 计算一个容器内数据的均值与矩阵形式协方差
 * @tparam C    容器类型
 * @tparam int 　数据维度
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个Eigen::Matrix<double, dim,1> 矢量类型
 */
template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C& data, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov,
                       Getter&& getter) {
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = data.size();
    assert(len > 1);

    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), Eigen::Matrix<double, dim, 1>::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                          [&mean, &getter](const E& sum, const auto& data) -> E {
                              D v = getter(data) - mean;
                              return sum + v * v.transpose();
                          }) / (len - 1);
    // clang-format on
}

/**
 * 高斯分布合并
 * @tparam S    scalar type
 * @tparam D    dimension
 * @param hist_m        历史点数
 * @param curr_n        当前点数
 * @param hist_mean     历史均值
 * @param hist_var      历史方差
 * @param curr_mean     当前均值
 * @param curr_var      当前方差
 * @param new_mean      新的均值
 * @param new_var       新的方差
 */
template <typename S, int D>
void UpdateMeanAndCov(int hist_m, int curr_n, const Eigen::Matrix<S, D, 1>& hist_mean,
                      const Eigen::Matrix<S, D, D>& hist_var, const Eigen::Matrix<S, D, 1>& curr_mean,
                      const Eigen::Matrix<S, D, D>& curr_var, Eigen::Matrix<S, D, 1>& new_mean,
                      Eigen::Matrix<S, D, D>& new_var) {
    assert(hist_m > 0);
    assert(curr_n > 0);
    new_mean = (hist_m * hist_mean + curr_n * curr_mean) / (hist_m + curr_n);
    new_var = (hist_m * (hist_var + (hist_mean - new_mean) * (hist_mean - new_mean).template transpose()) +
               curr_n * (curr_var + (curr_mean - new_mean) * (curr_mean - new_mean).template transpose())) /
              (hist_m + curr_n);
}

template <typename C, typename D, typename Getter>
void ComputeMedian(const C& data, D& median, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    std::vector<D> d;
    std::transform(data.begin(), data.end(), std::back_inserter(d),
                   [&getter](const auto& data) { return getter(data); });
    size_t n = d.size() / 2;
    std::nth_element(d.begin(), d.begin() + n, d.end());
    median = d[n];
}

template <typename S>
bool FitPlane(std::vector<Eigen::Matrix<S, 3, 1>>& data, Eigen::Matrix<S, 4, 1>& plane_coeffs, double eps = 1e-2) {
    if (data.size() < 3) {
        return false;
    }

    Eigen::MatrixXd A(data.size(), 4);
    for (int i = 0; i < data.size(); ++i) {
        A.row(i).head<3>() = data[i].transpose();
        A.row(i)[3] = 1.0;
    }

    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    plane_coeffs = svd.matrixV().col(3);

    // check error eps
    for (int i = 0; i < data.size(); ++i) {
        double err = plane_coeffs.template head<3>().dot(data[i]) + plane_coeffs[3];
        if (err * err > eps) {
            return false;
        }
    }

    return true;
}

template <typename S>
bool FitLine(std::vector<Eigen::Matrix<S, 3, 1>>& data, Eigen::Matrix<S, 3, 1>& origin, Eigen::Matrix<S, 3, 1>& dir,
             double eps = 0.2) {
    if (data.size() < 2) {
        return false;
    }

    origin = std::accumulate(data.begin(), data.end(), Eigen::Matrix<S, 3, 1>::Zero().eval()) / data.size();

    Eigen::MatrixXd Y(data.size(), 3);
    for (int i = 0; i < data.size(); ++i) {
        Y.row(i) = (data[i] - origin).transpose();
    }

    Eigen::JacobiSVD svd(Y, Eigen::ComputeFullV);
    dir = svd.matrixV().col(0);

    // check eps
    for (const auto& d : data) {
        if (dir.template cross(d - origin).template squaredNorm() > eps) {
            return false;
        }
    }

    return true;
}

template <typename S>
bool FitLine2D(const std::vector<Eigen::Matrix<S, 2, 1>>& data, Eigen::Matrix<S, 3, 1>& coeffs) {
    if (data.size() < 2) {
        return false;
    }

    Eigen::MatrixXd A(data.size(), 3);
    for (int i = 0; i < data.size(); ++i) {
        A.row(i).head<2>() = data[i].transpose();
        A.row(i)[2] = 1.0;
    }

    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    coeffs = svd.matrixV().col(2);
    return true;
}

/// 将角度保持在正负PI以内
inline void KeepAngleInPI(double& angle) {
    while (angle < -M_PI) {
        angle = angle + 2 * M_PI;
    }
    while (angle > M_PI) {
        angle = angle - 2 * M_PI;
    }
}

// bilinear interpolation
template <typename T>
inline float GetPixelValue(const cv::Mat& img, float x, float y) {
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    const T* data = &img.at<T>(floor(y), floor(x));
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] + (1 - xx) * yy * data[img.step / sizeof(T)] +
                 xx * yy * data[img.step / sizeof(T) + 1]);
}

template <typename S, int rows, int cols>
bool CheckNaN(const Eigen::Matrix<S, rows, cols>& m) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (std::isnan(m(i, j))) {
                LOG(ERROR) << "matrix has nan: \n" << m;
                return true;
            }
        }
    }
    return false;
}

/// 正态分布pdf
template <typename T, int N = 2>
inline double GaussianPDF(const Eigen::Matrix<T, N, 1>& mean, const Eigen::Matrix<T, N, N>& cov,
                          const Eigen::Matrix<T, N, 1>& x) {
    double det = std::fabs(cov.determinant());
    double exp_part = ((x - mean).transpose() * cov.inverse() * (x - mean))(0, 0);
    return exp(-0.5 * exp_part) / (2 * M_PI * sqrt(det));
}

/// PCD点转Eigen
template <typename PointT, typename S = double, int N>
inline Eigen::Matrix<S, N, 1> ToEigen(const PointT& pt) {
    Eigen::Matrix<S, N, 1> v;
    if (N == 2) {
        v << pt.x, pt.y;
    } else if (N == 3) {
        v << pt.x, pt.y, pt.z;
    }

    return v;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const Eigen::Matrix<T, 3, 1>& v) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return m;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const T& v1, const T& v2, const T& v3) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v3, v2, v3, 0.0, -v1, -v2, v1, 0.0;
    return m;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>&& ang) {
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K = SKEW_SYM_MATRIX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T& v1, const T& v2, const T& v3) {
    T&& norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001) {
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(v1 / norm, v2 / norm, v3 / norm);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    } else {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R) {
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3>& rot) {
    T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if (!singular) {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    } else {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

/// xt: 经验证，此转换与ros中tf::createQuaternionFromRPY结果一致
template <typename T>
Eigen::Matrix<T, 3, 3> RpyToRotM2(const T r, const T p, const T y) {
    using AA = Eigen::AngleAxis<T>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    return Eigen::Matrix<T, 3, 3>(AA(y, Vec3::UnitZ()) * AA(p, Vec3::UnitY()) * AA(r, Vec3::UnitX()));
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<S>& v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<S>& v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename T>
T rad2deg(const T& radians) {
    return radians * 180.0 / M_PI;
}

template <typename T>
T deg2rad(const T& degrees) {
    return degrees * M_PI / 180.0;
}

/**
 * 将某个数限定在范围内
 * @tparam T
 * @param num
 * @param min_limit
 * @param max_limit
 * @return
 */
template <typename T, typename T2>
void limit_in_range(T&& num, T2&& min_limit, T2&& max_limit) {
    if (num < min_limit) {
        num = min_limit;
    }
    if (num >= max_limit) {
        num = max_limit;
    }
}

/**
 * 估计平面：基于colPivHouseholderQr解平面方程，使用动态矩阵保存点云坐标
 */
template <typename T>
inline bool esti_plane_dynamic(Eigen::Matrix<T, 4, 1>& abcd, const std::vector<Vec3d>& point, const double& threshold) {
    if (point.size() < 3) {
        LOG(ERROR) << "the number of points should not be less than 3, given " << point.size();
        return false;
    }
    // Marked by xiaotao:
    // 矩阵A和b的类型别动（包括double和Eigen::Dynamic），当points.size()为3或4时可能会出问题，原因不知。
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(point.size(), 3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> b(point.size(), 1);
    A.setZero();
    b.setConstant(-1.0f);
    for (int j = 0; j < point.size(); j++) {
        A(j, 0) = point[j][0];
        A(j, 1) = point[j][1];
        A(j, 2) = point[j][2];
    }
    Eigen::Matrix<double, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    const double len_inv = 1.0f / normvec.norm();
    abcd(0, 0) = normvec(0, 0) = normvec(0, 0) * len_inv;
    abcd(1, 0) = normvec(1, 0) = normvec(1, 0) * len_inv;
    abcd(2, 0) = normvec(2, 0) = normvec(2, 0) * len_inv;
    abcd(3, 0) = len_inv;

    bool res = (((A * normvec).array() + len_inv).abs() <= threshold).all();
    return res;
}

/**
 * 高斯分布的增量更新
 * @see https://www.cnblogs.com/yoyaprogrammer/p/delta_variance.html
 * @param hist_n     历史数据个数
 * @param hist_mean 历史均值
 * @param hist_var2 历史方差
 * @param curr_n    当前数据个数
 * @param curr_mean 当前真值
 * @param curr_var2 当前方差
 * @param new_mean 合并后均值
 * @param new_var2 合并后方差
 */
inline void HistoryMeanAndVar(size_t hist_n, float hist_mean, float hist_var2, size_t curr_n, float curr_mean,
                              float curr_var2, float& new_mean, float& new_var2) {
    new_mean = (hist_n * hist_mean + curr_n * curr_mean) / (hist_n + curr_n);
    new_var2 = (hist_n * (hist_var2 + (new_mean - hist_mean) * (new_mean - hist_mean)) +
                curr_n * (curr_var2 + (new_mean - curr_mean) * (new_mean - curr_mean))) /
               (hist_n + curr_n);
}

/**
 * pose 插值算法
 * @tparam T    数据类型
 * @tparam C 数据容器类型
 * @tparam FT 获取时间函数
 * @tparam FP 获取pose函数
 * @param query_time 查找时间
 * @param data  数据容器
 * @param take_pose_func 从数据中取pose的谓词，接受一个数据，返回一个SE3
 * @param result 查询结果
 * @param best_match_iter 查找到的最近匹配
 *
 * NOTE 要求query_time必须在data最大时间和最小时间之间(容许0.5s内误差)
 * data的map按时间排序
 * @return
 */
template <typename T, typename C, typename FT, typename FP>
inline bool PoseInterp(double query_time, C&& data, FT&& take_time_func, FP&& take_pose_func, SE3& result,
                       T& best_match, float time_th = 0.5) {
    if (data.empty()) {
        LOG(INFO) << "cannot interp because data is empty. ";
        return false;
    }

    double last_time = take_time_func(*data.rbegin());
    if (query_time > last_time) {
        if (query_time < (last_time + time_th)) {
            // 尚可接受
            result = take_pose_func(*data.rbegin());
            best_match = *data.rbegin();
            return true;
        }
        return false;
    }

    auto match_iter = data.begin();
    for (auto iter = data.begin(); iter != data.end(); ++iter) {
        auto next_iter = iter;
        next_iter++;

        if (take_time_func(*iter) < query_time && take_time_func(*next_iter) >= query_time) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_n = match_iter;
    match_iter_n++;

    double dt = take_time_func(*match_iter_n) - take_time_func(*match_iter);
    double s = (query_time - take_time_func(*match_iter)) / dt;  // s=0 时为第一帧，s=1时为next
    // 出现了 dt为0的bug
    if (fabs(dt) < 1e-6) {
        best_match = *match_iter;
        result = take_pose_func(*match_iter);
        return true;
    }

    SE3 pose_first = take_pose_func(*match_iter);
    SE3 pose_next = take_pose_func(*match_iter_n);
    result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
              pose_first.translation() * (1 - s) + pose_next.translation() * s};
    best_match = s < 0.5 ? *match_iter : *match_iter_n;
    return true;
}

/**
 * Calculate cosine and sinc of sqrt(x2).
 * @param x2 the squared angle must be non-negative
 * @return a pair containing cos and sinc of sqrt(x2)
 */
template <class scalar>
inline std::pair<scalar, scalar> cos_sinc_sqrt(const scalar& x2) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    static scalar const taylor_0_bound = boost::math::tools::epsilon<scalar>();
    static scalar const taylor_2_bound = sqrt(taylor_0_bound);
    static scalar const taylor_n_bound = sqrt(taylor_2_bound);

    assert(x2 >= 0 && "argument must be non-negative");

    // FIXME check if bigger bounds are possible
    if (x2 >= taylor_n_bound) {
        // slow fall-back solution
        scalar x = sqrt(x2);
        return std::make_pair(cos(x), sin(x) / x);  // x is greater than 0.
    }

    // FIXME Replace by Horner-Scheme (4 instead of 5 FLOP/term, numerically more stable, theoretically cos and sinc can
    // be calculated in parallel using SSE2 mulpd/addpd)
    // TODO Find optimal coefficients using Remez algorithm
    static scalar const inv[] = {1 / 3., 1 / 4., 1 / 5., 1 / 6., 1 / 7., 1 / 8., 1 / 9.};
    scalar cosi = 1., sinc = 1;
    scalar term = -1 / 2. * x2;
    for (int i = 0; i < 3; ++i) {
        cosi += term;
        term *= inv[2 * i];
        sinc += term;
        term *= -inv[2 * i + 1] * x2;
    }

    return std::make_pair(cosi, sinc);
}

inline double exp(Vec3d& result, const Vec3d& vec, const double& scale = 1) {
    double norm2 = vec.squaredNorm();
    std::pair<double, double> cos_sinc = cos_sinc_sqrt(scale * scale * norm2);
    double mult = cos_sinc.second * scale;
    result = mult * vec;
    return cos_sinc.first;
}

inline SO3 exp(const Vec3d& vec, const double& scale = 1) {
    double norm2 = vec.squaredNorm();
    std::pair<double, double> cos_sinc = cos_sinc_sqrt(scale * scale * norm2);
    double mult = cos_sinc.second * scale;
    Vec3d result = mult * vec;
    return SO3(Quatd(cos_sinc.first, result[0], result[1], result[2]));
}

inline Eigen::Matrix<double, 2, 3> PseudoInverse(const Eigen::Matrix<double, 3, 2>& X) {
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 2>> svd(X, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Vec2d sv = svd.singularValues();
    Eigen::Matrix<double, 3, 2> U = svd.matrixU().block<3, 2>(0, 0);
    Eigen::Matrix<double, 2, 2> V = svd.matrixV();
    Eigen::Matrix<double, 2, 3> U_adjoint = U.adjoint();
    double tolerance = std::numeric_limits<double>::epsilon() * 3 * std::abs(sv(0, 0));
    sv(0, 0) = std::abs(sv(0, 0)) > tolerance ? 1.0 / sv(0, 0) : 0;
    sv(1, 0) = std::abs(sv(1, 0)) > tolerance ? 1.0 / sv(1, 0) : 0;

    return V * sv.asDiagonal() * U_adjoint;
}

/**
 * 这个看着像是SO3的Jacobian，不过我还不确定是个什么jb玩意
 * @param v
 * @return
 */
inline Eigen::Matrix<double, 3, 3> A_matrix(const Vec3d& v) {
    Eigen::Matrix<double, 3, 3> res;
    double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    double norm = std::sqrt(squaredNorm);
    if (norm < 1e-5) {
        res = Eigen::Matrix<double, 3, 3>::Identity();
    } else {
        res = Eigen::Matrix<double, 3, 3>::Identity() + (1 - std::cos(norm)) / squaredNorm * SO3::hat(v) +
              (1 - std::sin(norm) / norm) / squaredNorm * SO3::hat(v) * SO3::hat(v);
    }
    return res;
}

/**
 * 边缘化
 * @param H
 * @param start
 * @param end
 * @return
 */
inline Eigen::MatrixXd Marginalize(const Eigen::MatrixXd& H, const int& start, const int& end) {
    // Goal
    // a  | ab | ac       a*  | 0 | ac*
    // ba | b  | bc  -->  0   | 0 | 0
    // ca | cb | c        ca* | 0 | c*

    // Size of block before block to marginalize
    const int a = start;
    // Size of block to marginalize
    const int b = end - start + 1;
    // Size of block after block to marginalize
    const int c = H.cols() - (end + 1);

    // Reorder as follows:
    // a  | ab | ac       a  | ac | ab
    // ba | b  | bc  -->  ca | c  | cb
    // ca | cb | c        ba | bc | b

    Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
    if (a > 0) {
        Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
        Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
        Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
    }
    if (a > 0 && c > 0) {
        Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
        Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
    }
    if (c > 0) {
        Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
        Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
        Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
    }
    Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

    // Perform marginalization (Schur complement)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
    for (int i = 0; i < b; ++i) {
        if (singularValues_inv(i) > 1e-6) singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
    Hn.block(0, 0, a + c, a + c) =
        Hn.block(0, 0, a + c, a + c) - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
    Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
    Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
    Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

    // Inverse reorder
    // a*  | ac* | 0       a*  | 0 | ac*
    // ca* | c*  | 0  -->  0   | 0 | 0
    // 0   | 0   | 0       ca* | 0 | c*
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
    if (a > 0) {
        res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
        res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
        res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
    }
    if (a > 0 && c > 0) {
        res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
        res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
    }
    if (c > 0) {
        res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
        res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
        res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
    }

    res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

    return res;
}

/**
 * pose 插值算法
 * @tparam T    数据类型
 * @param query_time 查找时间
 * @param data  数据
 * @param take_pose_func 从数据中取pose的谓词
 * @param result 查询结果
 * @param best_match_iter 查找到的最近匹配
 *
 * NOTE 要求query_time必须在data最大时间和最小时间之间，不会外推
 * data的map按时间排序
 * @return 插值是否成功
 */
template <typename T>
bool PoseInterp(double query_time, const std::map<double, T>& data, const std::function<SE3(const T&)>& take_pose_func,
                SE3& result, T& best_match) {
    if (data.empty()) {
        LOG(INFO) << "data is empty";
        return false;
    }

    if (query_time > data.rbegin()->first) {
        LOG(INFO) << "query time is later than last, " << std::setprecision(18) << ", query: " << query_time
                  << ", end time: " << data.rbegin()->first;

        return false;
    }

    auto match_iter = data.begin();
    for (auto iter = data.begin(); iter != data.end(); ++iter) {
        auto next_iter = iter;
        next_iter++;

        if (iter->first < query_time && next_iter->first >= query_time) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_n = match_iter;
    match_iter_n++;
    assert(match_iter_n != data.end());

    double dt = match_iter_n->first - match_iter->first;
    double s = (query_time - match_iter->first) / dt;  // s=0 时为第一帧，s=1时为next

    SE3 pose_first = take_pose_func(match_iter->second);
    SE3 pose_next = take_pose_func(match_iter_n->second);
    result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
              pose_first.translation() * (1 - s) + pose_next.translation() * s};
    best_match = s < 0.5 ? match_iter->second : match_iter_n->second;
    return true;
}

}  // namespace sad::math

#endif  // MAPPING_MATH_UTILS_H
