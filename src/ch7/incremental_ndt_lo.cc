//
// Created by xiang on 2022/7/20.
//

#include "ch7/incremental_ndt_lo.h"
#include "common/math_utils.h"
#include "common/timer/timer.h"

namespace sad {

void IncrementalNDTLO::AddCloud(CloudPtr scan, SE3& pose, bool use_guess) {
    if (first_frame_) {
        // 第一个帧，直接加入local map
        pose = SE3();
        last_kf_pose_ = pose;
        ndt_.AddCloud(scan);
        first_frame_ = false;
        return;
    }

    // 此时local map位于NDT内部，直接配准即可
    SE3 guess;
    ndt_.SetSource(scan);
    if (estimated_poses_.size() < 2) {
        ndt_.AlignNdt(guess);
    } else {
        if (!use_guess) {
            // 从最近两个pose来推断
            SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
            SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
            guess = T1 * (T2.inverse() * T1);
        } else {
            guess = pose;
        }

        ndt_.AlignNdt(guess);
    }

    pose = guess;
    estimated_poses_.emplace_back(pose);

    CloudPtr scan_world(new PointCloudType);
    pcl::transformPointCloud(*scan, *scan_world, guess.matrix().cast<float>());

    if (IsKeyframe(pose)) {
        last_kf_pose_ = pose;
        cnt_frame_ = 0;
        // 放入ndt内部的local map
        ndt_.AddCloud(scan_world);
    }

    if (viewer_ != nullptr) {
        viewer_->SetPoseAndCloud(pose, scan_world);
    }
    cnt_frame_++;
}

bool IncrementalNDTLO::IsKeyframe(const SE3& current_pose) {
    if (cnt_frame_ > 10) {
        return true;
    }

    SE3 delta = last_kf_pose_.inverse() * current_pose;
    return delta.translation().norm() > options_.kf_distance_ ||
           delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD;
}

void IncrementalNDTLO::SaveMap(const std::string& map_path) {
    if (viewer_) {
        viewer_->SaveMap(map_path);
    }
}

}  // namespace sad