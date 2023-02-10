//
// Created by xiang on 2022/3/23.
//

#ifndef SLAM_IN_AUTO_DRIVING_FRAME_H
#define SLAM_IN_AUTO_DRIVING_FRAME_H

#include "common/eigen_types.h"
#include "common/lidar_utils.h"

namespace sad {

/**
 * 一次2D lidar扫描
 */
struct Frame {
    Frame() {}
    Frame(Scan2d::Ptr scan) : scan_(scan) {}

    /// 将当前帧存入文本文件以供离线调用
    void Dump(const std::string& filename);

    /// 从文件读取frame数据
    void Load(const std::string& filename);

    size_t id_ = 0;               // scan id
    size_t keyframe_id_ = 0;      // 关键帧 id
    double timestamp_ = 0;        // 时间戳，一般不用
    Scan2d::Ptr scan_ = nullptr;  // 激光扫描数据
    SE2 pose_;                    // 位姿，world to scan, T_w_c
    SE2 pose_submap_;             // 位姿，submap to scan, T_s_c
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_FRAME_H
