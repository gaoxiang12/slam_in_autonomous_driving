//
// Created by xiang on 2022/7/7.
//

#ifndef SLAM_IN_AUTO_DRIVING_ICP_3D_H
#define SLAM_IN_AUTO_DRIVING_ICP_3D_H

#include "ch5/kdtree.h"

namespace sad {

/**
 * 3D 形式的ICP
 * 先SetTarget, 再SetSource, 然后调用Align方法获取位姿
 *
 * ICP 求解R,t 将source点云配准到target点云上
 * 如果 p 是source点云中的点，那么R*p+t就得到target中的配对点
 *
 * 使用第5章的Kd树来求取最近邻
 */
class Icp3d {
   public:
    struct Options {
        int max_iteration_ = 20;                // 最大迭代次数
        double max_nn_distance_ = 1.0;          // 点到点最近邻查找时阈值
        double max_plane_distance_ = 0.05;      // 平面最近邻查找时阈值
        double max_line_distance_ = 0.5;        // 点线最近邻查找时阈值
        int min_effective_pts_ = 10;            // 最近邻点数阈值
        double eps_ = 1e-2;                     // 收敛判定条件
        bool use_initial_translation_ = false;  // 是否使用初始位姿中的平移估计
    };

    Icp3d() {}
    Icp3d(Options options) : options_(options) {}

    /// 设置目标的Scan
    void SetTarget(CloudPtr target) {
        target_ = target;
        BuildTargetKdTree();

        // 计算点云中心
        target_center_ = std::accumulate(target->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
                                         [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                         target_->size();
        LOG(INFO) << "target center: " << target_center_.transpose();
    }

    /// 设置被配准的Scan
    void SetSource(CloudPtr source) {
        source_ = source;
        source_center_ = std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                                         [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                         source_->size();
        LOG(INFO) << "source center: " << source_center_.transpose();
    }

    void SetGroundTruth(const SE3& gt_pose) {
        gt_pose_ = gt_pose;
        gt_set_ = true;
    }

    /// 使用gauss-newton方法进行配准, 点到点
    bool AlignP2P(SE3& init_pose);

    /// 基于gauss-newton的点线ICP
    bool AlignP2Line(SE3& init_pose);

    /// 基于gauss-newton的点面ICP
    bool AlignP2Plane(SE3& init_pose);

   private:
    // 建立目标点云的Kdtree
    void BuildTargetKdTree();

    std::shared_ptr<KdTree> kdtree_ = nullptr;  // 第5章的kd树

    CloudPtr target_ = nullptr;
    CloudPtr source_ = nullptr;

    Vec3d target_center_ = Vec3d::Zero();
    Vec3d source_center_ = Vec3d::Zero();

    bool gt_set_ = false;  // 真值是否设置
    SE3 gt_pose_;

    Options options_;
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_ICP_3D_H
