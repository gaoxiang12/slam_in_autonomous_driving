//
// Created by xiang on 2022/3/15.
//

#ifndef SLAM_IN_AUTO_DRIVING_ICP_2D_H
#define SLAM_IN_AUTO_DRIVING_ICP_2D_H

#include "common/eigen_types.h"
#include "common/lidar_utils.h"

#include <pcl/search/kdtree.h>

namespace sad {

/**
 * 第六章谈到的各种类型的ICP代码实现
 * 用法：先SetTarget, 此时构建target点云的KD树；再SetSource，然后调用Align*方法
 */
class Icp2d {
   public:
    using Point2d = pcl::PointXY;
    using Cloud2d = pcl::PointCloud<Point2d>;
    Icp2d() {}

    /// 设置目标的Scan
    void SetTarget(Scan2d::Ptr target) {
        target_scan_ = target;
        BuildTargetKdTree();
    }

    /// 设置被配准的Scan
    void SetSource(Scan2d::Ptr source) { source_scan_ = source; }

    /// 使用高斯牛顿法进行配准
    bool AlignGaussNewton(SE2& init_pose);

    /// 使用高斯牛顿法进行配准, Point-to-Plane
    bool AlignGaussNewtonPoint2Plane(SE2& init_pose);

   private:
    // 建立目标点云的Kdtree
    void BuildTargetKdTree();

    pcl::search::KdTree<Point2d> kdtree_;
    Cloud2d::Ptr target_cloud_;  // PCL 形式的target cloud

    Scan2d::Ptr target_scan_ = nullptr;
    Scan2d::Ptr source_scan_ = nullptr;
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_ICP_2D_H
