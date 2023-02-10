#ifndef FUSION_UI_STAMPED_POSE_H
#define FUSION_UI_STAMPED_POSE_H

#include "common/eigen_types.h"

namespace sad::ui {

struct StampedPose {
    uint64_t stamp_ = 0;   // 时间戳，以us为单位
    SE3 T_odom_baselink_;  // lio给出的相对定位
    SE3 T_map_baselink_;   // 激光/视觉定位，或pgo融合定位
    SE3 T_map_odom_;       // = T_map_baselink_ * T_odom_baselink_.inverse()
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct LocResult : public StampedPose {
    double confidence_ = 0.0;     // 置信度/得分
    std::vector<double> scores_;  // 栅格搜索得分
    std::size_t best_index_;      // 栅格搜索最佳得分的索引

    LocResult() = default;

    explicit LocResult(const common::LocalizationResult& loc_res) {
        stamp_ = 1e6 * loc_res.timestamp_;                           // 时间戳，us为单位
        T_map_baselink_ = loc_res.pose_;                             // 位姿
        T_odom_baselink_ = loc_res.rel_pose_;                        // 位姿
        T_map_odom_ = T_map_baselink_ * T_odom_baselink_.inverse();  // 位姿
        confidence_ = loc_res.confidence_;                           // 置信度/得分
        scores_ = loc_res.grid_scores_;                              // 栅格搜索得分
        best_index_ = loc_res.best_grid_search_;                     // 栅格搜索最佳得分的索引
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct PgoResult : public StampedPose {
    PgoResult() = default;

    /// 从定位结果中，摘取UI中使用的部分
    explicit PgoResult(const common::LocalizationResult& loc_res) {
        stamp_ = 1e6 * loc_res.timestamp_;                           // 时间戳，us为单位
        T_map_baselink_ = loc_res.pose_;                             // 位姿
        T_odom_baselink_ = loc_res.rel_pose_;                        // 位姿
        T_map_odom_ = T_map_baselink_ * T_odom_baselink_.inverse();  // 位姿
        global_pose_status_ = loc_res.status_;                       // 融合定位状态

        // pgo优化的图
        const int rows = common::options::ui::pgo_res_rows;
        const int cols = loc_res.pgo_res_.size() / rows;
        assert(loc_res.pgo_res_.size() % rows == 0);
        graph_vertexes_.resize(rows, cols);
        for (int i = 0; i < cols; i++) {
            for (int j = 0; j < rows; j++) {
                graph_vertexes_(j, i) = loc_res.pgo_res_[i * rows + j];
            }
        }
    }

    Eigen::MatrixXf graph_vertexes_;                                                 // pgo的优化图
    common::GlobalPoseStatus global_pose_status_ = common::GlobalPoseStatus::OTHER;  // pgo给出的定位状态
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace sad::ui

#endif