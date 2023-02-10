//
// Created by xiang on 22-12-6.
//

#ifndef SLAM_IN_AUTO_DRIVING_KEYFRAME_H
#define SLAM_IN_AUTO_DRIVING_KEYFRAME_H

#include "common/eigen_types.h"
#include "common/point_types.h"

#include <map>

namespace sad {

struct Keyframe {
    Keyframe() {}
    Keyframe(double time, IdType id, const SE3& lidar_pose, CloudPtr cloud)
        : timestamp_(time), id_(id), lidar_pose_(lidar_pose), cloud_(cloud) {}

    /// 将本帧点云存盘，从内存中清除
    void SaveAndUnloadScan(const std::string& path);

    void LoadScan(const std::string& path);

    /// 保存至文本文件
    void Save(std::ostream& os);

    /// 从文件读取
    void Load(std::istream& is);

    double timestamp_ = 0;            // 时间戳
    IdType id_ = 0;                   // 关键帧id，唯一
    SE3 lidar_pose_;                  // 雷达位姿
    SE3 rtk_pose_;                    // rtk 位姿
    SE3 opti_pose_1_;                 // 第一阶段优化pose
    SE3 opti_pose_2_;                 // 第二阶段优化pose
    bool rtk_heading_valid_ = false;  // rtk是否含有旋转
    bool rtk_valid_ = true;           // rtk原始状态是否有效
    bool rtk_inlier_ = true;          // rtk在优化过程中是否为正常值

    CloudPtr cloud_ = nullptr;
};

bool LoadKeyFrames(const std::string& path, std::map<IdType, std::shared_ptr<Keyframe>>& keyframes);
}  // namespace sad

using KFPtr = std::shared_ptr<sad::Keyframe>;

#endif  // SLAM_IN_AUTO_DRIVING_KEYFRAME_H
