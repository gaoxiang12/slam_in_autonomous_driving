//
// Created by xiang on 2022/7/21.
//

#ifndef SLAM_IN_AUTO_DRIVING_FEATURE_EXTRACTION_H
#define SLAM_IN_AUTO_DRIVING_FEATURE_EXTRACTION_H

#include "common/point_types.h"

namespace sad {

/**
 * 根据线束信息来提取特征
 * 需要知道雷达的线数分布，目前只支持velodyne的雷达
 */
class FeatureExtraction {
    /// 一个线ID+曲率的结构
    struct IdAndValue {
        IdAndValue() {}
        IdAndValue(int id, double value) : id_(id), value_(value) {}
        int id_ = 0;
        double value_ = 0;  // 曲率
    };

   public:
    FeatureExtraction() {}

    /**
     * 提取角点和平面点
     * @param pc_in         输入点云（全量信息）
     * @param pc_out_edge   输出角点的点云
     * @param pc_out_surf   输出平面的点云
     */
    void Extract(FullCloudPtr pc_in, CloudPtr pc_out_edge, CloudPtr pc_out_surf);

    /**
     * 对单独一段区域提取角点和面点
     * @param pc_in
     * @param cloud_curvature
     * @param pc_out_edge
     * @param pc_out_surf
     */
    void ExtractFromSector(const CloudPtr& pc_in, std::vector<IdAndValue>& cloud_curvature, CloudPtr& pc_out_edge,
                           CloudPtr& pc_out_surf);

   private:
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_FEATURE_EXTRACTION_H
