//
// Created by xiang on 2022/3/18.
//

#ifndef SLAM_IN_AUTO_DRIVING_LIKELIHOOD_FILED_H
#define SLAM_IN_AUTO_DRIVING_LIKELIHOOD_FILED_H

#include <opencv2/core.hpp>

#include "common/eigen_types.h"
#include "common/lidar_utils.h"

namespace sad {

class LikelihoodField {
   public:
    /// 2D 场的模板，在设置target scan或map的时候生成
    struct ModelPoint {
        ModelPoint(int dx, int dy, float res) : dx_(dx), dy_(dy), residual_(res) {}
        int dx_ = 0;
        int dy_ = 0;
        float residual_ = 0;
    };

    LikelihoodField() { BuildModel(); }

    /// 增加一个2D的目标scan
    void SetTargetScan(Scan2d::Ptr scan);

    /// 设置被配准的那个scan
    void SetSourceScan(Scan2d::Ptr scan);

    /// 从占据栅格地图生成一个似然场地图
    void SetFieldImageFromOccuMap(const cv::Mat& occu_map);

    /// 使用高斯牛顿法配准
    bool AlignGaussNewton(SE2& init_pose);

    /**
     * 使用g2o配准
     * @param init_pose 初始位姿 NOTE 使用submap时，给定相对于该submap的位姿，估计结果也是针对于这个submap的位姿
     * @return
     */
    bool AlignG2O(SE2& init_pose);

    /// 获取场函数，转换为RGB图像
    cv::Mat GetFieldImage();

    bool HasOutsidePoints() const { return has_outside_pts_; }

    void SetPose(const SE2& pose) { pose_ = pose; }

   private:
    void BuildModel();

    SE2 pose_;  // T_W_S
    Scan2d::Ptr target_ = nullptr;
    Scan2d::Ptr source_ = nullptr;

    std::vector<ModelPoint> model_;  // 2D 模板
    cv::Mat field_;                  // 场函数
    bool has_outside_pts_ = false;   // 是否含有出了这个场的点

    // 参数配置
    inline static const float resolution_ = 20;  // 每米多少个像素
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_LIKELIHOOD_FILED_H
