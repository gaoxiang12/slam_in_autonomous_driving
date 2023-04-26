//
// Created by xiang on 2022/7/18.
//

#include "ch7/direct_ndt_lo.h"
#include "common/math_utils.h"
#include "tools/pcl_map_viewer.h"

#include <pcl/common/transforms.h>

namespace sad {

void DirectNDTLO::AddCloud(CloudPtr scan, SE3& pose) {
    if (local_map_ == nullptr) {
        // 第一个帧，直接加入local map
        local_map_.reset(new PointCloudType);
        // operator += 用来拼接点云
        *local_map_ += *scan;
        pose = SE3();
        last_kf_pose_ = pose;

        if (options_.use_pcl_ndt_) {
            ndt_pcl_.setInputTarget(local_map_);
        } else {
            ndt_.SetTarget(local_map_);
        }

        return;
    }

    // 计算scan相对于local map的位姿
    pose = AlignWithLocalMap(scan);
    CloudPtr scan_world(new PointCloudType);
    pcl::transformPointCloud(*scan, *scan_world, pose.matrix().cast<float>());

    if (IsKeyframe(pose)) {
        last_kf_pose_ = pose;

        // 重建local map
        scans_in_local_map_.emplace_back(scan_world);
        if (scans_in_local_map_.size() > options_.num_kfs_in_local_map_) {
            scans_in_local_map_.pop_front();
        }

        local_map_.reset(new PointCloudType);
        for (auto& scan : scans_in_local_map_) {
            *local_map_ += *scan;
        }

        if (options_.use_pcl_ndt_) {
            ndt_pcl_.setInputTarget(local_map_);
        } else {
            ndt_.SetTarget(local_map_);
        }
    }

    if (viewer_ != nullptr) {
        viewer_->SetPoseAndCloud(pose, scan_world);
    }
}

bool DirectNDTLO::IsKeyframe(const SE3& current_pose) {
    // 只要与上一帧相对运动超过一定距离或角度，就记关键帧
    SE3 delta = last_kf_pose_.inverse() * current_pose;
    return delta.translation().norm() > options_.kf_distance_ ||
           delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD;
}

SE3 DirectNDTLO::AlignWithLocalMap(CloudPtr scan) {
    if (options_.use_pcl_ndt_) {
        ndt_pcl_.setInputSource(scan);
    } else {
        ndt_.SetSource(scan);
    }

    CloudPtr output(new PointCloudType());

    SE3 guess;
    bool align_success = true;
    if (estimated_poses_.size() < 2) {
        if (options_.use_pcl_ndt_) {
            ndt_pcl_.align(*output, guess.matrix().cast<float>());
            guess = Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
        } else {
            align_success = ndt_.AlignNdt(guess);
        }
    } else {
        // 从最近两个pose来推断
        SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
        SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
        guess = T1 * (T2.inverse() * T1);

        if (options_.use_pcl_ndt_) {
            ndt_pcl_.align(*output, guess.matrix().cast<float>());
            guess = Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
        } else {
            align_success = ndt_.AlignNdt(guess);
        }
    }

    LOG(INFO) << "pose: " << guess.translation().transpose() << ", "
              << guess.so3().unit_quaternion().coeffs().transpose();

    if (options_.use_pcl_ndt_) {
        LOG(INFO) << "trans prob: " << ndt_pcl_.getTransformationProbability();
    }

    estimated_poses_.emplace_back(guess);
    return guess;
}

void DirectNDTLO::SaveMap(const std::string& map_path) {
    if (viewer_) {
        viewer_->SaveMap(map_path);
    }
}

}  // namespace sad