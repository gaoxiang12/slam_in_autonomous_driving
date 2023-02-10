//
// Created by xiang on 2022/4/7.
//

#ifndef SLAM_IN_AUTO_DRIVING_MULTI_RESOLUTION_LIKELIHOOD_FILED_H
#define SLAM_IN_AUTO_DRIVING_MULTI_RESOLUTION_LIKELIHOOD_FILED_H

#include <opencv2/core.hpp>

#include "common/eigen_types.h"
#include "common/lidar_utils.h"

namespace sad {

/// 多分辨率的似然场配准方法
class MRLikelihoodField {
   public:
    /// 2D 场的模板，在设置target scan或map的时候生成
    struct ModelPoint {
        ModelPoint(int dx, int dy, float res) : dx_(dx), dy_(dy), residual_(res) {}
        int dx_ = 0;
        int dy_ = 0;
        float residual_ = 0;
    };

    MRLikelihoodField() { BuildModel(); }

    /// 从占据栅格地图生成一个似然场地图
    void SetFieldImageFromOccuMap(const cv::Mat& occu_map);

    /// 使用g2o配准，内部调用不同层级的图像进行配准
    bool AlignG2O(SE2& init_pose);

    /// 获取场函数，转换为RGB图像
    std::vector<cv::Mat> GetFieldImage();

    /// 设置中心（通常即submap中心）
    void SetPose(const SE2& pose) { pose_ = pose; }

    /// 设置匹配源
    void SetSourceScan(Scan2d::Ptr scan) { source_ = scan; }

    float Resolution(int level = 0) const { return resolution_[level]; }

    int Levels() const { return levels_; }

   private:
    /**
     * 在某一层图像中配准
     * @param level
     * @param init_pose
     * @return
     */
    bool AlignInLevel(int level, SE2& init_pose);

    void BuildModel();

    SE2 pose_;

    Scan2d::Ptr source_ = nullptr;

    std::vector<ModelPoint> model_;  // 2D 模板
    std::vector<cv::Mat> field_;     // 场函数

    std::vector<int> num_inliers_;
    std::vector<double> inlier_ratio_;

    // 参数配置
    inline static const int levels_ = 4;                                       // 金字塔层数
    inline static const std::vector<float> resolution_ = {2.5, 5, 10, 20};     // 每米多少个像素
    inline static const std::vector<float> ratios_ = {0.125, 0.25, 0.5, 1.0};  // 相比占据栅格图的比例尺
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_MULTI_RESOLUTION_LIKELIHOOD_FILED_H
