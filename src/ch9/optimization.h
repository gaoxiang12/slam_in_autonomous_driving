//
// Created by xiang on 22-12-7.
//

#ifndef SLAM_IN_AUTO_DRIVING_OPTIMIZATION_H
#define SLAM_IN_AUTO_DRIVING_OPTIMIZATION_H

#include <g2o/core/sparse_optimizer.h>

#include "common/eigen_types.h"
#include "common/g2o_types.h"
#include "common/math_utils.h"
#include "keyframe.h"
#include "loopclosure.h"

namespace sad {

// 后端优化
class Optimization {
   public:
    explicit Optimization(const std::string& yaml);

    /// 初始化，指定为第一轮还是第二轮
    bool Init(int stage = 1);

    /// 执行优化
    void Run();

   private:
    /// 如果RTK全程不带旋转，那么先对激光和RTK做一次ICP来对齐整条轨迹
    void InitialAlign();

    /// 构建优化问题
    void BuildProblem();

    /// 添加顶点
    void AddVertices();

    /// 添加各种位姿观测
    void AddRTKEdges();
    void AddLidarEdges();
    void AddLoopEdges();

    /// 求解问题
    void Solve();

    /// 移除异常值
    void RemoveOutliers();

    /// 保存优化结果
    void SaveResults();

    /// 读取回环候选
    void LoadLoopCandidates();

    /// 保存g2o文件
    void SaveG2O(const std::string& file_name);

    std::string yaml_;
    std::map<IdType, KFPtr> keyframes_;
    bool rtk_has_rot_ = false;
    int stage_ = 1;  // 优化阶段
    SE3 TBG_;        // body 到 gnss 的外参

    std::vector<LoopCandidate> loop_candidates_;  // 回环候选

    std::map<IdType, VertexPose*> vertices_;

    g2o::SparseOptimizer optimizer_;
    std::vector<EdgeGNSS*> gnss_edge_;
    std::vector<EdgeGNSSTransOnly*> gnss_trans_edge_;

    std::vector<EdgeRelativeMotion*> lidar_edge_;
    std::vector<EdgeRelativeMotion*> loop_edge_;

    // 参数
    double rtk_outlier_th_ = 1.0;   // RTK 异常阈值
    int lidar_continuous_num_ = 3;  // 雷达相邻位姿数量
    double rtk_pos_noise_ = 0.5;
    double rtk_ang_noise_ = 2.0 * math::kDEG2RAD;
    double rtk_height_noise_ratio_ = 20.0;
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_OPTIMIZATION_H
